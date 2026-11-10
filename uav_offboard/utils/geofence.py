"""Fit a horizontal search disk within PX4's uploaded polygon/circle fences."""

import math

from pygeodesy.ltp import LocalCartesian

from shapely.geometry import Point, Polygon


def bounded_spiral_radius(fence, reference, centre, requested_radius, margin):
    """Conservatively respect every inclusion and exclusion, with a metre margin.

    Working in PX4's local reference keeps fence geometry in the same NED frame
    as trajectory setpoints. The entire disk fits, including interpolation
    between spiral samples. PX4's home-distance/altitude parameters are separate.
    """
    if not fence.polygons and not fence.circles:
        raise ValueError('Spiral requires a non-empty uploaded geofence')
    projection = LocalCartesian(reference[0], reference[1], 0.)
    centre = Point(centre[:2])

    def project(point):
        lat, lon = point.latitude, point.longitude
        if not (math.isfinite(lat) and math.isfinite(lon)
                and -90 <= lat <= 90 and -180 <= lon <= 180):
            raise ValueError('Invalid geofence coordinates')
        p = projection.forward(lat, lon, 0.)
        return p.y, p.x  # ENU -> PX4 north/east.

    clearances = []
    for circle in fence.circles:
        if not math.isfinite(circle.radius_m) or circle.radius_m <= 0:
            raise ValueError('Invalid geofence circle radius')
        distance = centre.distance(Point(project(circle.center)))
        clearances.append(
            circle.radius_m - distance if circle.inclusion else distance - circle.radius_m)
    for polygon in fence.polygons:
        vertices = [project(p) for p in polygon.vertices]
        if len(vertices) > 1 and vertices[0] == vertices[-1]:
            vertices.pop()
        if len(vertices) < 3 or any(
                math.dist(a, b) <= 1e-6 for a, b in zip(vertices, vertices[1:] + vertices[:1])):
            raise ValueError('Degenerate geofence polygon')
        shape = Polygon(vertices)
        if not shape.is_valid or shape.area <= 5e-10:
            raise ValueError('Invalid geofence polygon')
        distance = centre.distance(shape.boundary)
        clearances.append(distance if shape.contains(centre) == polygon.inclusion else -distance)
    radius = min(requested_radius, min(clearances) - margin)
    if not math.isfinite(radius) or radius <= 0:
        raise ValueError('No room for a spiral inside the geofence margin')
    return radius
