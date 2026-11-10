"""Spiral geometry and lifecycle; no vehicle or live ROS domain."""
import asyncio
import numpy as np
import pytest
from rclpy.time import Time
from bb_uav_msgs.action import SpiralSearch
from bb_uav_msgs.msg import Geofence, GeofenceCircle
from geographic_msgs.msg import GeoPoint

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
    node._local_reference = (1.2809, 103.8555, 1)
    node.spiral_geofence_margin = .5
    node.spiral_geofence_max_age = 1000.
    node._geofence = Geofence(circles=[GeofenceCircle(
        center=GeoPoint(latitude=1.2809, longitude=103.8555), radius_m=100., inclusion=True)])
    node._sweep_position_valid = Mock(return_value=True)
    node.get_logger = Mock()
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
