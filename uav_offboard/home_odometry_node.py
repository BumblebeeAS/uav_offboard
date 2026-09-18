"""Publish a separate home-relative view of PX4 local odometry."""

from copy import deepcopy
from math import isfinite

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import HomePosition
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster
from uav_offboard.utils.qos_profiles import QOS_PROFILE_HOME


def valid_home(home):
    """Require a finite local home reference."""
    return home is not None and home.valid_lpos and all(
        isfinite(value) for value in (home.x, home.y, home.z)
    )


def to_home_odometry(odom, home, home_frame):
    """Translate the pose, preserving timestamp, orientation and body twist."""
    result = deepcopy(odom)
    result.header.frame_id = home_frame
    result.pose.pose.position.x -= home.x
    result.pose.pose.position.y -= home.y
    result.pose.pose.position.z -= home.z
    return result


def home_transform(odom, home, home_frame):
    """Express the home origin in the raw local frame."""
    transform = TransformStamped()
    transform.header = deepcopy(odom.header)
    transform.child_frame_id = home_frame
    transform.transform.translation.x = home.x
    transform.transform.translation.y = home.y
    transform.transform.translation.z = home.z
    transform.transform.rotation.w = 1.0
    return transform


class HomeOdometryNode(Node):
    """Publish only while a valid home and matching local odometry are available."""

    def __init__(self):
        super().__init__('home_odometry_node')
        odom_topic = self.declare_parameter('odom_topic', 'odom_ned').value
        output_topic = self.declare_parameter('output_topic', 'odom_home_ned').value
        home_topic = self.declare_parameter(
            'home_position_topic', '/fmu/out/home_position_v1'
        ).value
        self.local_frame = self.declare_parameter('local_frame', 'odom_ned').value
        self.home_frame = self.declare_parameter('home_frame', 'home_ned').value
        if self.local_frame == self.home_frame:
            raise ValueError('Local and home frames must differ')
        self.home = None
        self.publisher = self.create_publisher(Odometry, output_topic, 10)
        self.broadcaster = TransformBroadcaster(self)
        self.home_sub = self.create_subscription(
            HomePosition, home_topic, self.home_callback, QOS_PROFILE_HOME
        )
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, qos_profile_sensor_data
        )

    def home_callback(self, msg):
        """Invalidate the reference if PX4 reports that home is unavailable."""
        self.home = msg if valid_home(msg) else None

    def odom_callback(self, msg):
        """Publish the translated odometry and the matching origin transform."""
        home = self.home
        if home is None:
            self.get_logger().warn('Waiting for valid PX4 local home', throttle_duration_sec=5.0)
            return
        if msg.header.frame_id != self.local_frame:
            self.get_logger().error(
                f'Expected odometry frame {self.local_frame}, got {msg.header.frame_id}',
                throttle_duration_sec=5.0,
            )
            return
        self.publisher.publish(to_home_odometry(msg, home, self.home_frame))
        # Keep the existing local -> body TF publisher as the sole body authority.
        self.broadcaster.sendTransform(home_transform(msg, home, self.home_frame))


def main(args=None):
    rclpy.init(args=args)
    node = HomeOdometryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
