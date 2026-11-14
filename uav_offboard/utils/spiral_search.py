"""Arc-length-parametrized outward spiral with bounded speed and acceleration."""
import math

import numpy as np
from shapely.geometry import LineString, Point
from shapely.ops import substring


def _connect_boundary(region, start, end):
    """Connect a clipped gap directly, or along its common boundary ring."""
    direct = LineString([start, end])
    if region.covers(direct):
        return list(direct.coords)
    # Both ends of one outside interval lie on the same boundary component.
    # Boundary arcs also handle holes, without a quadratic visibility graph.
    for ring in (region.exterior, *region.interiors):
        boundary = LineString(ring.coords)
        if max(boundary.distance(Point(start)), boundary.distance(Point(end))) > 1e-7:
            continue
        a, b = boundary.project(Point(start)), boundary.project(Point(end))
        low, high = sorted((a, b))
        forward = list(substring(boundary, low, high).coords)
        around = (list(substring(boundary, high, boundary.length).coords)
                  + list(substring(boundary, 0., low).coords)[1:])
        if a > b:
            forward.reverse()
        else:
            around.reverse()
        candidates = [[tuple(start), *arc, tuple(end)] for arc in (forward, around)]
        for route in sorted(candidates, key=lambda p: LineString(p).length):
            # Clipping uses a numerical inset, so this tolerance cannot consume
            # the actual clearance. The complete route is checked again below.
            if region.buffer(1e-8).covers(LineString(route)):
                return route
    raise ValueError('Cannot connect clipped spiral sections inside the geofence')


def _clip_spiral(points, region, radius):
    """Retain ordered spiral sections reachable inside the requested radius."""
    original = LineString(points[:, :2])
    if region.covers(original):
        return None
    # Leave numerical room at boundary joins while retaining the caller's full
    # physical clearance. Detours must also stay within the requested radius.
    working = region.buffer(-1e-5, resolution=128).intersection(
        Point(0., 0.).buffer(radius, resolution=128))
    components = [working] if working.geom_type == 'Polygon' else list(getattr(working, 'geoms', []))
    working = next((p for p in components if p.geom_type == 'Polygon' and p.covers(Point(0., 0.))), None)
    if working is None:
        raise ValueError('Spiral start has no connected area inside the geofence clearance')
    clipped = original.intersection(working)
    parts = [clipped] if clipped.geom_type == 'LineString' else list(getattr(clipped, 'geoms', []))
    sections = []
    for part in parts:
        if part.geom_type != 'LineString' or part.length <= 1e-9:
            continue
        coordinates = list(part.coords)
        a, b = original.project(Point(coordinates[0])), original.project(Point(coordinates[-1]))
        if a > b:
            coordinates.reverse()
        sections.append((min(a, b), coordinates))
    route, stops = [(0., 0.)], {0}
    def append(coordinates):
        for p in coordinates:
            if math.dist(route[-1], p) > 1e-9:
                route.append(p)
    for _, coordinates in sorted(sections):
        if math.dist(route[-1], coordinates[0]) > 1e-9:
            stops.add(len(route)-1)
            append(_connect_boundary(working, route[-1], coordinates[0]))
            stops.add(len(route)-1)
        append(coordinates)
        stops.add(len(route)-1)
    if len(route) < 2:
        raise ValueError('No non-zero spiral sections inside the geofence')
    # Stop at polygon corners, but retain curvature-limited motion on finely
    # sampled rounded boundaries. Every clipped-section join is already a stop.
    edges = np.diff(route, axis=0)
    turns = np.arctan2(abs(edges[:-1,0]*edges[1:,1] - edges[:-1,1]*edges[1:,0]),
                      np.sum(edges[:-1]*edges[1:], axis=1))
    stops.update((np.flatnonzero(turns > .05) + 1).tolist())
    result, stop_indices = [route[0]], {0}
    for i, (a, b) in enumerate(zip(route, route[1:])):
        count = max(1, math.ceil(math.dist(a, b)/.1))
        if i in stops and i+1 in stops:
            count = max(2, count)  # Accelerate/decelerate between two zero-speed ends.
        result.extend((np.asarray(a) + (np.asarray(b)-a)*j/count).tolist() for j in range(1,count+1))
        if i+1 in stops:
            stop_indices.add(len(result)-1)
    points = np.column_stack((result, np.zeros(len(result))))
    if not region.covers(LineString(points[:, :2])):
        raise ValueError('Clipped spiral or connection violates the geofence clearance')
    return points, stop_indices


class SpiralPath:
    """Freeze a horizontal path; accelerate gently and slow through tight turns."""

    def __init__(self, radius, turn_spacing, speed, acceleration=0.4, region=None):
        if not all(math.isfinite(v) and v > 0
                   for v in (radius, turn_spacing, speed, acceleration)):
            raise ValueError('Spiral parameters must be finite and positive')
        # A radial pitch larger than the diameter is not a useful spiral search.
        if turn_spacing > 2 * radius:
            raise ValueError('Spiral turn spacing must not exceed its diameter')
        b = turn_spacing / (2 * math.pi)
        end = radius / b
        theta = np.linspace(0., end, max(1001, int(end * 200) + 1))
        r = b * theta
        self.points = np.column_stack((r * np.cos(theta), r * np.sin(theta), np.zeros_like(r)))
        # Normal acceleration v^2 * curvature <= acceleration, including centre.
        curvature = (r * r + 2 * b * b) / (r * r + b * b) ** 1.5
        clipped = _clip_spiral(self.points, region, radius) if region is not None else None
        self.clipped = clipped is not None
        stops = {0, len(self.points)-1}
        if clipped is not None:
            self.points, stops = clipped
            edges = np.diff(self.points[:, :2], axis=0)
            lengths = np.linalg.norm(edges, axis=1)
            cross = abs(edges[:-1,0]*edges[1:,1] - edges[:-1,1]*edges[1:,0])
            denominator = lengths[:-1]*lengths[1:]*np.linalg.norm(edges[:-1]+edges[1:], axis=1)
            curvature = np.r_[0., np.divide(2*cross, denominator,
                                           out=np.full_like(cross, np.inf), where=denominator>0), 0.]
        ds = np.linalg.norm(np.diff(self.points, axis=0), axis=1)
        v = np.minimum(speed, np.sqrt(acceleration / np.maximum(curvature, 1e-12)))
        v[list(stops)] = 0.
        v[0] = v[-1] = 0.
        for i in range(1, len(v)):
            v[i] = min(v[i], math.sqrt(v[i-1]**2 + 2 * acceleration * ds[i-1]))
        for i in range(len(v)-2, -1, -1):
            v[i] = min(v[i], math.sqrt(v[i+1]**2 + 2 * acceleration * ds[i]))
        self.speeds, self.distances = v, ds
        if np.any(v[:-1] + v[1:] <= 0) or np.any(ds <= 0):
            raise ValueError('Clipped spiral cannot be timed without stationary segments')
        self.times = np.r_[0., np.cumsum(2 * ds / (v[:-1] + v[1:]))]
        self.duration = float(self.times[-1])

    def offset(self, elapsed):
        if elapsed <= 0:
            return self.points[0].copy()
        if elapsed >= self.duration:
            return self.points[-1].copy()
        i = int(np.searchsorted(self.times, elapsed, side='right') - 1)
        dt = elapsed - self.times[i]
        segment_time = self.times[i+1] - self.times[i]
        acceleration = (self.speeds[i+1] - self.speeds[i]) / segment_time
        distance = self.speeds[i] * dt + .5 * acceleration * dt * dt
        fraction = distance / self.distances[i]
        return self.points[i] + fraction * (self.points[i+1] - self.points[i])
