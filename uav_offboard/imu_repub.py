from operator import attrgetter

import rclpy
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu

from uav_offboard.utils.qos_profiles import QOS_PROFILE_SUB


class ImuRepubNode(Node):
    def __init__(self):
        super().__init__("imu_repub")

        self.sub = self.create_subscription(
            Odometry, "/uav/odom_ned", self.callback, QOS_PROFILE_SUB
        )

        self.pub = self.create_publisher(Imu, "/imu", 10)
        self.prev_velocity = None
        self.prev_time = None

    def callback(self, msg: Odometry):
        if self.prev_time is None and self.prev_velocity is None:
            self.prev_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            self.prev_velocity = Vector3()
            self.prev_velocity.x = msg.twist.twist.linear.x
            self.prev_velocity.y = msg.twist.twist.linear.y
            self.prev_velocity.z = msg.twist.twist.linear.z
            return
        assert self.prev_time is not None
        assert self.prev_velocity is not None

        imu = Imu()
        imu.header.stamp = msg.header.stamp
        imu.header.frame_id = msg.child_frame_id

        v_x, v_y, v_z = attrgetter("x", "y", "z")(msg.twist.twist.linear)
        dv_x = v_x - self.prev_velocity.x
        dv_y = v_y - self.prev_velocity.y
        dv_z = v_z - self.prev_velocity.z

        dt = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9 - self.prev_time
        imu.linear_acceleration = self.compute_linear_accel(dv_x, dv_y, dv_z, dt)

        imu.orientation = msg.pose.pose.orientation

        imu.angular_velocity = msg.twist.twist.angular

        imu.orientation_covariance[0] = -1.0
        imu.angular_velocity_covariance[0] = -1.0
        imu.linear_acceleration_covariance[0] = -1.0

        self.prev_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.prev_velocity = Vector3()
        self.prev_velocity.x = v_x
        self.prev_velocity.y = v_y
        self.prev_velocity.z = v_z

        self.pub.publish(imu)

    def compute_linear_accel(self, dx, dy, dz, dt):
        linear_accel = Vector3()

        linear_accel.x = dx / dt
        linear_accel.y = dy / dt
        linear_accel.z = dz / dt
        return linear_accel


def main():
    rclpy.init()
    node = ImuRepubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
