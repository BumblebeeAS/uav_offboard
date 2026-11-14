"""Landing poses use fixed extrinsics and reject stale detections."""
from math import sqrt
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.time import Time
from rclpy.clock import ClockType
from tf2_ros import Buffer

from uav_offboard.landing_target_pose_node import LandingTargetPoseNode


def instance():
    buffer = Buffer()
    transform = TransformStamped()
    transform.header.frame_id = 'body'
    transform.child_frame_id = 'camera'
    transform.transform.translation.x = 1.0
    transform.transform.rotation.z = sqrt(0.5)
    transform.transform.rotation.w = sqrt(0.5)
    buffer.set_transform_static(transform, 'test')
    pose = PoseStamped()
    pose.header.frame_id = 'camera'
    pose.header.stamp.sec = 100
    pose.pose.position.x = 2.0
    pose.pose.orientation.w = 1.0
    clock = Mock()
    clock.now.return_value = Time(seconds=100, nanoseconds=100_000_000,
                                  clock_type=ClockType.ROS_TIME)
    return SimpleNamespace(use_pose_topic=True, target_pose=pose,
        max_pose_age=0.3, source_frame='body', tf_buffer=buffer,
        get_clock=Mock(return_value=clock))


def test_pose_to_frd_preserves_capture_time():
    node = instance()
    position, timestamp = LandingTargetPoseNode.get_target_position(node)
    assert position.x == pytest.approx(1.0)
    assert position.y == pytest.approx(2.0)
    assert timestamp == 100_000_000


@pytest.mark.parametrize('invalid', ['missing', 'old', 'future', 'zero', 'nan'])
def test_invalid_pose_never_falls_back_to_stale_tf(invalid):
    node = instance()
    if invalid == 'missing': node.target_pose = None
    if invalid == 'old': node.target_pose.header.stamp.sec = 99
    if invalid == 'future': node.target_pose.header.stamp.sec = 101
    if invalid == 'zero': node.target_pose.header.stamp.sec = 0
    if invalid == 'nan': node.target_pose.pose.position.x = float('nan')
    assert LandingTargetPoseNode.get_target_position(node) is None
