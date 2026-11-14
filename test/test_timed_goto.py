"""Timed goto uses the existing action lifecycle with an interpolated target."""
import asyncio
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from rclpy.time import Time
from bb_uav_msgs.action import GoToPosition
from uav_offboard.offboard_node import OffboardNode
from uav_offboard.utils.goto import GeneralGoal


def scenario(monkeypatch, duration, cancel=False):
    node = OffboardNode.__new__(OffboardNode)
    node.current_position = np.array([2., 3., -3.])
    node.position_valid = True
    node.use_home_position = False
    node.is_goal_active = False
    node.get_logger = Mock()
    now = [0.]
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=now[0]))
    targets = []
    handle = Mock()
    handle.is_cancel_requested = False
    def sleep():
        targets.append((now[0], node.absolute_target.copy()))
        node.current_position = node.absolute_target.copy()
        now[0] += .05
        if cancel and now[0] >= 1.:
            handle.is_cancel_requested = True
    node.create_rate = Mock(return_value=SimpleNamespace(sleep=sleep))
    node.destroy_rate = Mock()
    monkeypatch.setattr('uav_offboard.offboard_node.rclpy.ok', lambda: now[0] < 8)
    request = GoToPosition.Goal(x=2., y=3., z=-2., duration=duration,
                               x_threshold=.03, y_threshold=.03, z_threshold=.03)
    return node, handle, GeneralGoal.from_position_goal(request), targets


@pytest.mark.parametrize('duration', [0., 5.])
def test_timed_goto_reaches_frozen_endpoint_without_lateral_motion(monkeypatch, duration):
    node, handle, goal, samples = scenario(monkeypatch, duration)
    result = asyncio.run(node.execute_callback(handle, goal, Mock()))
    assert result.success
    handle.succeed.assert_called_once()
    assert not node.is_goal_active
    assert node.absolute_target is None
    assert result.time_elapsed >= duration
    for t, target in samples:
        np.testing.assert_allclose(target[:2], [2, 3])
        assert target[2] == pytest.approx(-3. + min(t / 5., 1.)) if duration else target[2] == -2.
    if duration:
        np.testing.assert_allclose(samples[0][1], [2, 3, -3])
        midpoint = min(samples, key=lambda row: abs(row[0] - 2.5))
        assert midpoint[1][2] == pytest.approx(-2.5)


def test_timed_goto_cancels_during_ramp(monkeypatch):
    node, handle, goal, samples = scenario(monkeypatch, 5., cancel=True)
    node.nav_state, node.arming_state = 14, 2
    node._sweep_position_valid = Mock(return_value=True)
    held = []
    node.publish_trajectory_setpoint = lambda: held.append(node.absolute_target.copy())
    result = asyncio.run(node.execute_callback(handle, goal, Mock()))
    assert not result.success
    handle.canceled.assert_called_once()
    assert node.absolute_target is None
    assert not node.is_goal_active
    assert len(held) == 1
    np.testing.assert_allclose(held[0], node.current_position)


@pytest.mark.parametrize('duration', [-1., float('nan'), float('inf')])
def test_server_rejects_invalid_duration(monkeypatch, duration):
    node, handle, goal, samples = scenario(monkeypatch, 0.)
    goal.duration = duration
    assert not node.validate_goal(goal)


@pytest.mark.parametrize('speed', [2., 15.])
def test_normal_goto_keeps_distant_endpoint_without_ramp_or_travel_timeout(monkeypatch, speed):
    node, handle, goal, samples = scenario(monkeypatch, 0.)
    start = node.current_position.copy()
    endpoint = start + [100., 0., 0.]
    goal.x, goal.y, goal.z = endpoint
    goal.relative = False
    now = [0.]
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=now[0]))
    commanded = []
    def sleep():
        commanded.append(node.absolute_target.copy())
        delta = node.absolute_target - node.current_position
        distance = np.linalg.norm(delta)
        node.current_position += delta * min(1., speed * .05 / distance)
        now[0] += .05
    node.create_rate = Mock(return_value=SimpleNamespace(sleep=sleep))
    monkeypatch.setattr('uav_offboard.offboard_node.rclpy.ok', lambda: now[0] < 60.)
    result = asyncio.run(node.execute_callback(handle, goal, Mock()))
    assert result.success
    handle.succeed.assert_called_once()
    assert result.time_elapsed == pytest.approx(100. / speed, abs=.1)
    np.testing.assert_allclose(commanded, np.tile(endpoint, (len(commanded), 1)))
    node.create_rate.assert_called_once_with(20)
