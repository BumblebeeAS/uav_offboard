"""Run the home odometry view alongside an existing raw odometry publisher."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='uav'),
        DeclareLaunchArgument('home_position_topic', default_value='/fmu/out/home_position_v1'),
        DeclareLaunchArgument('tf_publish_rate', default_value='15.0'),
        Node(
            package='uav_offboard',
            executable='home_odometry_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'home_position_topic': LaunchConfiguration('home_position_topic'),
                'tf_publish_rate': ParameterValue(
                    LaunchConfiguration('tf_publish_rate'), value_type=float),
            }],
            output='screen',
        ),
    ])
