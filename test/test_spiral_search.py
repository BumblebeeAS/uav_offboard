"""Spiral geometry and lifecycle; no vehicle or live ROS domain."""
import asyncio
import numpy as np
import pytest
from rclpy.time import Time
from bb_uav_msgs.action import SpiralSearch
from bb_uav_msgs.msg import Geofence, GeofenceCircle, GeofencePolygon
from geographic_msgs.msg import GeoPoint
from pygeodesy.ltp import LocalCartesian
from rclpy.action import GoalResponse
from shapely.geometry import LineString, Point, Polygon, box

from types import SimpleNamespace
from unittest.mock import Mock
from uav_offboard.offboard_node import OffboardNode
from uav_offboard.utils.spiral_search import SpiralPath


def setup_node(monkeypatch, mode='follow'):
    node = OffboardNode.__new__(OffboardNode)
    node.current_position = np.array([12., 23., -7.])
    node.home_position = np.array([10., 20., -5.])
    node.use_home_position = True
    node.position_valid = True
    node.offboard_mode = SimpleNamespace(is_position=True)
    node.is_goal_active = False
    node.nav_state, node.arming_state = 14, 2
    node.spiral_geofence_margin, node.spiral_geofence_max_age = 1., 5.
    reference = (1.28, 103.85, 1, 0, 0)
    node._spiral_origin = (reference, node.current_position.copy())
    node._geofence = Geofence(circles=[GeofenceCircle(
        inclusion=True, center=GeoPoint(latitude=reference[0], longitude=reference[1]), radius_m=50.)])
    node._sweep_position_valid = Mock(return_value=True)
    node.get_logger = Mock()
    node.publish_trajectory_setpoint = Mock()
    seconds = [0.]
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=seconds[0]))
    def advance():
        if mode == 'follow':
            node.current_position = node.absolute_target.copy()
        seconds[0] += .02
    node.create_rate = Mock(return_value=SimpleNamespace(sleep=advance))
    node.destroy_rate = Mock()
    handle = Mock()
    handle.is_cancel_requested = mode == 'cancel'
    monkeypatch.setattr('uav_offboard.offboard_node.rclpy.ok', lambda: True)
    return node, handle


@pytest.mark.parametrize('fault', ['missing', 'stale', 'reset', 'tightened', 'safe_change'])
def test_spiral_requires_live_bounds_and_holds_when_they_change(monkeypatch, fault):
    node, handle = setup_node(monkeypatch)
    handle.request = SpiralSearch.Goal(radius=.1, turn_spacing=.1, speed=.2,
                                      settle_threshold=.02, settle_timeout=.2)
    if fault in ('missing', 'stale'):
        if fault == 'missing':node._geofence = None
        else:node._geofence.stamp.sec = -10
        assert node.spiral_search_goal_callback(handle.request) == GoalResponse.REJECT
        return
    advance = node.create_rate.return_value.sleep
    def change_bounds():
        advance()
        if fault == 'reset':
            reference, position = node._spiral_origin
            node._spiral_origin = ((*reference[:3], 1, 0), position)
        else:
            p = LocalCartesian(1.28, 103.85, 0.).reverse(23., 12.05 if fault == 'safe_change' else 12., 0.)
            node._geofence = Geofence(circles=[GeofenceCircle(
                inclusion=True, center=GeoPoint(latitude=p.lat, longitude=p.lon),
                radius_m=1.125 if fault == 'safe_change' else 1.05)])
    node.create_rate.return_value.sleep = change_bounds
    held = []
    node.publish_trajectory_setpoint = lambda: held.append(node.absolute_target.copy())
    result = asyncio.run(node.execute_spiral_search_callback(handle)).result
    if fault == 'safe_change':
        assert result.success
        handle.succeed.assert_called_once()
        return
    assert not result.success
    handle.abort.assert_called_once()
    np.testing.assert_allclose(held[-1], node.current_position)


@pytest.mark.parametrize('shape,extent', [('polygon', 5.), ('circle', 3.), ('exclusion', 5.)])
def test_clipping_uses_px4_axes_and_one_metre_clearance(monkeypatch, shape, extent):
    node, _ = setup_node(monkeypatch)
    reference, centre = node._spiral_origin
    projection = LocalCartesian(*reference[:2], 0.)
    def point(east, north):
        p = projection.reverse(east, north, 0.)
        return GeoPoint(latitude=p.lat, longitude=p.lon)
    if shape == 'circle':
        fence = Geofence(circles=[GeofenceCircle(inclusion=True, center=point(23,12), radius_m=4.)])
    else:
        corners = [(18,10),(28,10),(28,20),(18,20)] if shape == 'polygon' else [(0,0),(40,0),(40,30),(0,30)]
        fence = Geofence(polygons=[GeofencePolygon(inclusion=True, vertices=[point(*p) for p in corners])])
        if shape == 'exclusion':
            fence.circles.append(GeofenceCircle(inclusion=False, center=point(27,12), radius_m=1.))
    request = SpiralSearch.Goal(radius=5., turn_spacing=1., speed=1.5)
    path, safe, route = node._spiral_path(request, fence, reference, centre)
    assert safe.covers(route)
    assert request.radius == 5.
    assert np.max(np.linalg.norm(path.points[:,:2], axis=1)) == pytest.approx(extent, abs=.0002)
    assert np.all(path.points[:,2] == 0.)


@pytest.mark.parametrize('region', [
    Polygon([(-2,-3),(6,-3),(6,-1),(1,-1),(1,1),(6,1),(6,3),(-2,3)]),
    box(-6,-6,6,6).difference(Point(2,0).buffer(.6, resolution=128)),
    box(-2,-2,2,2).union(box(3,-2,6,2)),
])
def test_clipped_route_and_sampled_motion_stay_safe(region):
    path = SpiralPath(5., 1., 1.5, region=region)
    assert path.clipped and region.covers(LineString(path.points[:,:2]))
    assert np.max(np.linalg.norm(path.points[:,:2], axis=1)) <= 5.+1e-8
    assert np.all(np.diff(path.times)>0) and np.isfinite(path.duration)
    samples = np.array([path.offset(t) for t in np.arange(0.,path.duration,.02)])
    velocity = np.diff(samples, axis=0)/.02
    assert np.max(np.linalg.norm(velocity, axis=1)) <= 1.5+1e-8
    # Tangential and normal acceleration each have a 0.4 m/s² budget.
    assert np.max(np.linalg.norm(np.diff(velocity, axis=0)/.02, axis=1)) < .65
    edges = np.diff(path.points[:,:2], axis=0)
    turns = np.arctan2(abs(edges[:-1,0]*edges[1:,1]-edges[:-1,1]*edges[1:,0]),
                      np.sum(edges[:-1]*edges[1:], axis=1))
    assert np.all(path.speeds[1:-1][turns>.05] == 0.)


@pytest.mark.parametrize('speed_limit', [1., 1.5])
def test_spiral_radius_altitude_spacing_and_speed(speed_limit):
    path = SpiralPath(5., 1., speed_limit)
    t = np.linspace(0, path.duration, 20001)
    points = np.array([path.offset(x) for x in t])
    radii = np.linalg.norm(points[:, :2], axis=1)
    assert np.all(np.diff(radii) >= -1e-7)
    assert radii[0] == 0 and radii[-1] == pytest.approx(5.)
    assert np.all(points[:, 2] == 0)
    speed = np.linalg.norm(np.diff(points, axis=0), axis=1) / (t[1]-t[0])
    assert max(speed) <= speed_limit + .00001
    assert speed[0] < .05 and speed[-1] < .05
    assert (80 < path.duration < 100) if speed_limit == 1. else (70 < path.duration < 80)
    for turns in range(1, 6):
        index = np.argmin(np.linalg.norm(path.points[:, :2] - [turns, 0], axis=1))
        assert np.linalg.norm(path.points[index, :2] - [turns, 0]) < .01


@pytest.mark.parametrize('args', [(0,1,1), (1,0,1), (1,1,-1), (1,3,1), (float('nan'),1,1)])
def test_invalid_parameters(args):
    with pytest.raises(ValueError):SpiralPath(*args)


@pytest.mark.parametrize('mode', ['follow', 'cancel', 'stale', 'stationary'])
def test_spiral_action_lifecycle(monkeypatch, mode):
    node, handle = setup_node(monkeypatch, mode)
    handle.request = SpiralSearch.Goal(radius=.1, turn_spacing=.1, speed=.2,
                                      settle_threshold=.02, settle_timeout=.2)
    node.nav_state = 14
    node.arming_state = 2
    node.publish_trajectory_setpoint = lambda: None
    if mode == 'stale':node._sweep_position_valid.return_value = False
    result = asyncio.run(node.execute_spiral_search_callback(handle)).result
    assert node.absolute_target is None and not node.is_goal_active
    if mode == 'follow':
        assert result.success
        handle.succeed.assert_called_once()
    elif mode == 'cancel':
        handle.canceled.assert_called_once()
        assert not result.success
    else:
        handle.abort.assert_called_once()
        assert not result.success


def test_tracking_margin_excursion_keeps_following_planned_spiral(monkeypatch):
    node, handle = setup_node(monkeypatch)
    handle.request = SpiralSearch.Goal(radius=.1, turn_spacing=.1, speed=.2,
                                      settle_threshold=.02, settle_timeout=.2)
    advance = node.create_rate.return_value.sleep
    steps = [0]
    targets = []
    def drift_once():
        targets.append(node.absolute_target.copy())
        advance()
        steps[0] += 1
        if steps[0] == 1:
            node.current_position = np.array([12., 49.1, -7.])
    node.create_rate.return_value.sleep = drift_once
    result = asyncio.run(node.execute_spiral_search_callback(handle)).result
    assert result.success
    handle.abort.assert_not_called()
    # Measured overshoot must not replace the clipped route with a recovery target.
    assert max(np.linalg.norm(target[:2] - np.array([12., 23.])) for target in targets) <= .1 + 1e-8
    assert not node.get_logger().warning.called
