from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("rx1_ik"), "config", "ik.ros2.yaml"]
            ),
        ),
        Node(
            package="rx1_ik",
            executable="rx1_ik_node",
            name="rx1_ik",
            output="screen",
            parameters=[params_file],
        ),
    ])
