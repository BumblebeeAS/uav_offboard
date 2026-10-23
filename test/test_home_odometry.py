"""Verify home translation, TF consistency and publication gating."""

from copy import deepcopy
from types import SimpleNamespace
from unittest.mock import Mock

from nav_msgs.msg import Odometry
from px4_msgs.msg import HomePosition
from rclpy.impl.rcutils_logger import RcutilsLogger

from uav_offboard.home_odometry_node import (
    HomeOdometryNode, home_transform, to_home_odometry,
)


def sample_odom():
    msg = Odometry()
    msg.header.frame_id = 'odom_ned'
    msg.header.stamp.sec = 123
    msg.child_frame_id = 'uav/base_link_frd'
    msg.pose.pose.position.x = 12.0
    msg.pose.pose.position.y = 23.0
    msg.pose.pose.position.z = -7.0
    msg.pose.pose.orientation.w = 1.0
    msg.twist.twist.linear.x = 2.0
    msg.pose.covariance[0] = 0.2
    msg.twist.covariance[0] = 0.3
    return msg


def test_translation_preserves_other_fields_and_input():
    raw = sample_odom()
    original = deepcopy(raw)
    home = HomePosition(x=10.0, y=20.0, z=-5.0, valid_lpos=True)
    output = to_home_odometry(raw, home, 'home_ned')
    assert raw == original
    assert output.header.frame_id == 'home_ned'
    assert output.header.stamp == raw.header.stamp
    assert output.child_frame_id == raw.child_frame_id
    assert output.pose.pose.orientation == raw.pose.pose.orientation
    assert list(output.pose.covariance) == list(raw.pose.covariance)
    assert output.twist == raw.twist
    assert (output.pose.pose.position.x, output.pose.pose.position.y,
            output.pose.pose.position.z) == (2.0, 3.0, -2.0)


def test_tf_reconstructs_raw_pose():
    raw = sample_odom()
    home = HomePosition(x=10.0, y=20.0, z=-5.0, valid_lpos=True)
    transformed = to_home_odometry(raw, home, 'home_ned')
    tf = home_transform(raw, home, 'home_ned')
    assert tf.header == raw.header
    assert tf.child_frame_id == 'home_ned'
    assert tf.transform.rotation.w == 1.0
    for axis in ('x', 'y', 'z'):
        assert (getattr(transformed.pose.pose.position, axis)
                + getattr(tf.transform.translation, axis)
                == getattr(raw.pose.pose.position, axis))


def test_callbacks_gate_output_and_follow_home_changes():
    node = SimpleNamespace(
        home=None, local_frame='odom_ned', home_frame='home_ned',
        publisher=Mock(), broadcaster=Mock(),
        get_logger=Mock(return_value=Mock(spec_set=RcutilsLogger)),
    )
    raw = sample_odom()
    HomeOdometryNode.odom_callback(node, raw)
    node.publisher.publish.assert_not_called()
    node.broadcaster.sendTransform.assert_not_called()
    HomeOdometryNode.home_callback(node, HomePosition(x=12.0, valid_lpos=True))
    HomeOdometryNode.odom_callback(node, raw)
    assert node.publisher.publish.call_args.args[0].pose.pose.position.x == 0.0
    HomeOdometryNode.home_callback(node, HomePosition(x=10.0, valid_lpos=True))
    HomeOdometryNode.odom_callback(node, raw)
    assert node.publisher.publish.call_args.args[0].pose.pose.position.x == 2.0
    assert node.publisher.publish.call_count == 2
    raw.header.frame_id = 'unexpected_frame'
    HomeOdometryNode.odom_callback(node, raw)
    assert node.publisher.publish.call_count == 2
    HomeOdometryNode.home_callback(node, HomePosition(valid_lpos=False))
    raw.header.frame_id = 'odom_ned'
    HomeOdometryNode.odom_callback(node, raw)
    assert node.publisher.publish.call_count == 2
