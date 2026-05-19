from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_camera = LaunchConfiguration("launch_camera")
    servo_port = LaunchConfiguration("servo_port")

    return LaunchDescription([
        DeclareLaunchArgument("launch_camera", default_value="false"),
        DeclareLaunchArgument("servo_port", default_value="/dev/ttyUSB-arduino3.3"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("rx1_motor"), "launch", "rx1_motor.launch.py"]
                )
            ),
            launch_arguments={"servo_port": servo_port}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("rx1_description"), "launch", "urdf.launch.py"]
                )
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("rx1_bringup"), "launch", "includes", "gemini2.launch.py"]
                )
            ),
            condition=IfCondition(launch_camera),
        ),
    ])
