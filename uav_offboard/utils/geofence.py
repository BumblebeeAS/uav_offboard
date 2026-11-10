"""Fit a horizontal search disk within PX4's uploaded polygon/circle fences."""

import math

import numpy as np
from pygeodesy.ltp import LocalCartesian


def bounded_spiral_radius(fence, reference, centre, requested_radius, margin):
    """Conservatively respect every inclusion and exclusion, with a metre margin.

    Working in PX4's local reference keeps fence geometry in the same NED frame
    as trajectory setpoints. The entire disk fits, including interpolation
    between spiral samples. PX4's home-distance/altitude parameters are separate.
    """
    if not fence.polygons and not fence.circles:
        raise ValueError('Spiral requires a non-empty uploaded geofence')
    projection = LocalCartesian(reference[0], reference[1], 0.)
    centre = np.asarray(centre[:2], dtype=float)

    def project(point):
        lat, lon = point.latitude, point.longitude
        if not (math.isfinite(lat) and math.isfinite(lon) and -90 <= lat <= 90 and -180 <= lon <= 180):
            raise ValueError('Invalid geofence coordinates')
        p = projection.forward(lat, lon, 0.)
        return np.array([p.y, p.x])  # ENU -> PX4 north/east.

    clearances = []
    for circle in fence.circles:
        if not math.isfinite(circle.radius_m) or circle.radius_m <= 0:
            raise ValueError('Invalid geofence circle radius')
        distance = float(np.linalg.norm(centre - project(circle.center)))
        clearances.append(circle.radius_m - distance if circle.inclusion else distance - circle.radius_m)
    for polygon in fence.polygons:
        vertices = np.array([project(p) for p in polygon.vertices])
        if len(vertices) > 1 and np.array_equal(vertices[0], vertices[-1]):
            vertices = vertices[:-1]
        if len(vertices) < 3:
            raise ValueError('Geofence polygon needs at least three vertices')
        ends = np.roll(vertices, -1, axis=0)
        edges = ends - vertices
        lengths = np.sum(edges * edges, axis=1)
        area = abs(np.sum(vertices[:, 0] * ends[:, 1] - vertices[:, 1] * ends[:, 0]))
        if np.any(lengths <= 1e-12) or area <= 1e-9:
            raise ValueError('Degenerate geofence polygon')
        fractions = np.clip(np.sum((centre - vertices) * edges, axis=1) / lengths, 0., 1.)
        distance = float(np.min(np.linalg.norm(centre - (vertices + fractions[:, None] * edges), axis=1)))
        inside = False
        x, y = centre
        for (ax, ay), (bx, by) in zip(vertices, ends):
            if (ay > y) != (by > y) and x < (bx - ax) * (y - ay) / (by - ay) + ax:
                inside = not inside
        clearances.append(distance if inside == polygon.inclusion else -distance)
    radius = min(requested_radius, min(clearances) - margin)
    if not math.isfinite(radius) or radius <= 0:
        raise ValueError('No room for a spiral inside the geofence margin')
    return radius
