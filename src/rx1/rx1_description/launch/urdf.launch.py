from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable


def generate_launch_description():
    model = LaunchConfiguration("model")
    rvizconfig = LaunchConfiguration("rvizconfig")

    robot_description = {
        "robot_description": Command([FindExecutable(name="xacro"), " ", model])
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            "model",
            default_value=PathJoinSubstitution(
                [FindPackageShare("rx1_description"), "urdf", "rx1.urdf"]
            ),
        ),
        DeclareLaunchArgument(
            "rvizconfig",
            default_value=PathJoinSubstitution(
                [FindPackageShare("rx1_description"), "rviz", "urdf.rviz"]
            ),
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_aggregator",
            output="screen",
            parameters=[
                {
                    "source_list": [
                        "/right_arm_joint_states",
                        "/left_arm_joint_states",
                        "/torso_joint_states",
                        "/head_joint_states",
                        "/command_joint_states",
                        "/visualize_joint_states",
                    ]
                }
            ],
            remappings=[("/joint_states", "/aggregated_joint_states")],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description, {"publish_frequency": 50.0}],
            remappings=[("/joint_states", "/aggregated_joint_states")],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rvizconfig],
            output="screen",
        ),
    ])
