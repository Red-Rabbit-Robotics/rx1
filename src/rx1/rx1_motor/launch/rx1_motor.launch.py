from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    servo_port = LaunchConfiguration("servo_port")

    return LaunchDescription([
        DeclareLaunchArgument("servo_port", default_value="/dev/ttyUSB-arduino3.3"),
        Node(
            package="rx1_motor",
            executable="rx1_motor_node",
            name="rx1_motor_node",
            output="screen",
            parameters=[{"servo_port": servo_port}],
        ),
    ])
