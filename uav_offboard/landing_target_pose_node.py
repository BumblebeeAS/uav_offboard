import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from px4_msgs.msg import LandingTargetPose, VehicleStatus
from uav_offboard.utils.qos_profiles import QOS_PROFILE_PUB, QOS_PROFILE_SUB


class LandingTargetPoseNode(Node):
    """ROS node to handle landing target pose estimation and control."""

    def __init__(self):
        super().__init__("landing_target_pose_node")

        landing_target_pose_topic = (
            self.declare_parameter(
                "landing_target_pose_topic", "/fmu/in/landing_target_pose"
            )
            .get_parameter_value()
            .string_value
        )
        vehicle_status_topic = (
            self.declare_parameter("vehicle_status_topic", "/fmu/out/vehicle_status")
            .get_parameter_value()
            .string_value
        )

        # Declare transform frame parameters
        self.source_frame = (
            self.declare_parameter("source_frame", "uav/base_link_frd")
            .get_parameter_value()
            .string_value
        )
        self.target_frame = (
            self.declare_parameter("target_frame", "aruco_board")
            .get_parameter_value()
            .string_value
        )

        # Initialize TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.status_sub = self.create_subscription(
            VehicleStatus,
            vehicle_status_topic,
            self.vehicle_status_callback,
            QOS_PROFILE_SUB,
        )

        self.landing_target_pose_pub = self.create_publisher(
            LandingTargetPose, landing_target_pose_topic, QOS_PROFILE_PUB
        )

        self.timer_ = self.create_timer(0.1, self.publish_landing_target_pose)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED

        self.get_logger().info(
            f"Landing target pose node started. Looking for transform from '{self.source_frame}' to '{self.target_frame}'"
        )

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def publish_landing_target_pose(self):
        if (
            self.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_PRECLAND
            or self.arming_state != VehicleStatus.ARMING_STATE_ARMED
        ):
            return

        try:
            # Look up the transform from base_link_frd to aruco_board
            transform = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.target_frame,
                Time(),
                timeout=Duration(nanoseconds=100_000_000),  # 0.1 seconds
            )

            # Extract position from transform (NED/FRD frame)
            # In FRD: X = Forward, Y = Right, Z = Down
            x_rel = transform.transform.translation.x
            y_rel = transform.transform.translation.y
            z_rel = transform.transform.translation.z

            msg = LandingTargetPose()
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            msg.is_static = True
            msg.rel_pos_valid = True
            msg.rel_vel_valid = False  # We don't compute velocity

            # Relative position in FRD/NED frame
            msg.x_rel = float(x_rel)
            msg.y_rel = float(y_rel)
            msg.z_rel = float(z_rel)

            # Velocity (set to zero since we're not computing it)
            msg.vx_rel = 0.0
            msg.vy_rel = 0.0

            # TODO: Subscribe to home position topic and fill these fields accordingly
            msg.x_abs = 0.0
            msg.y_abs = 0.0
            msg.z_abs = 0.0
            msg.abs_pos_valid = True

            self.landing_target_pose_pub.publish(msg)

            self.get_logger().debug(
                f"Published landing target: x={x_rel:.2f}, y={y_rel:.2f}, z={z_rel:.2f}"
            )

        except TransformException as ex:
            self.get_logger().warn(
                f"Could not get transform from '{self.source_frame}' to '{self.target_frame}': {ex}",
                throttle_duration_sec=5.0,
            )
            return


def main():
    rclpy.init(args=None)
    node = LandingTargetPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
