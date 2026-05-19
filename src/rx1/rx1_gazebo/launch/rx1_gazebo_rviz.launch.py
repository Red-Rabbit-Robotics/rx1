import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rx1_gazebo_share = get_package_share_directory("rx1_gazebo")
    rx1_description_share = get_package_share_directory("rx1_description")

    model = LaunchConfiguration("model")
    world = LaunchConfiguration("world")
    robot_name = LaunchConfiguration("robot_name")
    robot_x = LaunchConfiguration("robot_x")
    robot_y = LaunchConfiguration("robot_y")
    robot_z = LaunchConfiguration("robot_z")

    gazebo_with_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rx1_gazebo_share, "launch", "rx1_gazebo.launch.py")
        ),
        launch_arguments={
            "model": model,
            "world": world,
            "robot_name": robot_name,
            "robot_x": robot_x,
            "robot_y": robot_y,
            "robot_z": robot_z,
            "use_rviz": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value=os.path.join(
                    rx1_description_share, "urdf", "rx1_optimized.harmonic.urdf.xacro"
                ),
            ),
            DeclareLaunchArgument(
                "world",
                default_value=os.path.join(
                    rx1_gazebo_share, "worlds", "rx1_table_world.sdf"
                ),
            ),
            DeclareLaunchArgument("robot_name", default_value="rx1"),
            DeclareLaunchArgument("robot_x", default_value="0.0"),
            DeclareLaunchArgument("robot_y", default_value="0.0"),
            DeclareLaunchArgument("robot_z", default_value="0.0"),
            gazebo_with_rviz,
        ]
    )
