"""Sweep geometry and action lifecycle without publishing to a vehicle."""

import asyncio
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from bb_uav_msgs.action import CircleSweep

from uav_offboard.offboard_node import OffboardNode


def setup_node(monkeypatch, mode='follow'):
    node = OffboardNode.__new__(OffboardNode)
    node.current_position = np.array([12., 23., -7.])
    node.home_position = np.array([10., 20., -5.])
    node.use_home_position = True
    node.position_valid = True
    node.offboard_mode = SimpleNamespace(is_position=True)
    node.is_goal_active = False
    node._sweep_position_valid = Mock(return_value=True)
    node.get_logger = Mock()
    seconds = [0.]
    node.get_clock = lambda: SimpleNamespace(now=lambda: FakeTime(seconds[0]))
    def advance():
        if mode == 'follow':
            node.current_position = node.absolute_target.copy()
        seconds[0] += .02
    node.create_rate = Mock(return_value=SimpleNamespace(sleep=advance))
    node.destroy_rate = Mock()
    handle = Mock()
    handle.is_cancel_requested = mode == 'cancel'
    handle.request = CircleSweep.Goal(radius=.05, duration=3.,
                                     return_threshold=.015, return_timeout=.2)
    monkeypatch.setattr('uav_offboard.offboard_node.rclpy.ok', lambda: True)
    return node, handle


class FakeTime:
    def __init__(self, seconds):
        self.nanoseconds = int(seconds * 1e9)

    def __sub__(self, other):
        return SimpleNamespace(nanoseconds=self.nanoseconds - other.nanoseconds)


@pytest.mark.parametrize('mode', ['follow', 'cancel', 'stationary', 'stale', 'return_timeout'])
def test_action_success_cancel_and_failure_cleanup(monkeypatch, mode):
    node, handle = setup_node(monkeypatch, mode)
    if mode == 'stale':
        node._sweep_position_valid.return_value = False
    if mode == 'return_timeout':
        # Follow the circle but stay 2 cm off centre during the return phase.
        ticks = [0]
        original_sleep = node.create_rate.return_value.sleep
        def advance():
            node.current_position = node.absolute_target.copy()
            if ticks[0] >= 175:
                node.current_position[0] += .02
            ticks[0] += 1
            original_sleep()
        node.create_rate.return_value.sleep = advance
    result = asyncio.run(node.execute_circle_sweep_callback(handle)).result
    assert not node.is_goal_active
    assert node.absolute_target is None
    if mode in ('follow', 'stationary'):
        assert result.success
        assert result.time_elapsed >= 4.
        handle.succeed.assert_called_once()
        handle.abort.assert_not_called()
        if mode == 'stationary':
            # Tracking lag is feedback only, even when larger than the old 4 cm guard.
            assert max(call.args[0].feedback.distance_to_goal
                       for call in handle.publish_feedback.call_args_list) > .04
        np.testing.assert_allclose(node.current_position, [12, 23, -7], atol=.015)
    elif mode == 'cancel':
        assert not result.success
        handle.canceled.assert_called_once()
        handle.succeed.assert_not_called()
    else:
        assert not result.success
        handle.abort.assert_called_once()
        handle.succeed.assert_not_called()
        if mode == 'return_timeout':
            assert 'timed out' in result.message


def test_circle_edge_action_freezes_start_and_finishes_there(monkeypatch):
    node, handle = setup_node(monkeypatch)
    start = node.current_position.copy()
    targets = []
    advance = node.create_rate.return_value.sleep
    def capture():
        targets.append(node.absolute_target.copy())
        advance()
    node.create_rate.return_value.sleep = capture
    handle.request.start_on_circle = True
    handle.request.radius = 1.
    handle.request.duration = 5.
    result = asyncio.run(node.execute_circle_sweep_callback(handle)).result
    assert result.success
    assert 5. <= result.time_elapsed < 5.1
    np.testing.assert_allclose(targets[0], start, atol=1e-12)
    np.testing.assert_allclose(node.current_position, start, atol=.015)
    centre = start - np.array([1., 0., 0.])
    targets = np.array(targets)
    np.testing.assert_allclose(np.linalg.norm(targets[:, :2] - centre[:2], axis=1), 1.)
    np.testing.assert_allclose(targets[:, 2], start[2])
    handle.succeed.assert_called_once()
