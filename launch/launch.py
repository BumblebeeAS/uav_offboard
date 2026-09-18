import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("uav_offboard"),
        "config",
        "params.yaml",
    )

    nodes = [
        PushRosNamespace("uav"),
        Node(
            package="uav_offboard",
            executable="offboard_node",
            name="offboard_node",
            parameters=[config],
        ),
        Node(
            package="uav_offboard",
            executable="landing_target_pose_node",
            name="landing_target_pose_node",
            parameters=[config],
        ),
        Node(
            package="uav_offboard",
            executable="actuator_control_node",
            name="actuator_control_node",
            parameters=[config],
        )
    ]

    return LaunchDescription(nodes)
