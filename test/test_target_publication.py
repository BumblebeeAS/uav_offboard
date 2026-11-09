"""Deterministic reentrant-action cleanup during a timer's target publication."""
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from rclpy.time import Time

from uav_offboard.offboard_node import OffboardNode


def make_node(target):
    node = OffboardNode.__new__(OffboardNode)
    node.absolute_target = target
    node.is_goal_active = True
    node.goal_pose_frame = 'odom_ned'
    node.publisher_trajectory = Mock()
    node.publisher_goal_pose = Mock()
    node.publisher_offboard_mode = Mock()
    node.nav_state = 14
    node.arming_state = 2
    node.offboard_mode = SimpleNamespace(is_position=True, is_acceleration=False,
                                         get_mode=lambda: dict(position=True, velocity=False,
                                             acceleration=False, attitude=False, body_rate=False,
                                             thrust_and_torque=False, direct_actuator=False))
    node.get_clock = Mock(return_value=SimpleNamespace(now=lambda: Time(seconds=1)))
    return node


@pytest.mark.parametrize('replacement', [None, np.array([99., 98., 97.])])
def test_cleanup_or_new_goal_during_publish_keeps_coherent_xyz(replacement):
    node = make_node(np.array([1., 2., 3.]))
    clock = node.get_clock.return_value
    def during_publish():
        node.absolute_target = replacement
        node.is_goal_active = False
        return clock
    node.get_clock.side_effect = during_publish
    node.publish_trajectory_setpoint()
    message = node.publisher_trajectory.publish.call_args.args[0]
    np.testing.assert_allclose(message.position, [1, 2, 3])
    node.publisher_goal_pose.publish.assert_called_once()


def test_cleanup_after_control_loop_guard_does_not_stop_heartbeat():
    node = make_node(np.array([1., 2., 3.]))
    original = node.publish_trajectory_setpoint
    def cleanup_after_guard():
        node.reset_internal_state()
        original()
    node.publish_trajectory_setpoint = cleanup_after_guard
    node.control_loop_callback()
    # The next tick also survives with no active target, maintaining heartbeat.
    node.control_loop_callback()
    assert node.publisher_offboard_mode.publish.call_count == 2
    node.publisher_trajectory.publish.assert_not_called()
