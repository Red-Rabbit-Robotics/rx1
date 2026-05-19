import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model = LaunchConfiguration("model")
    world = LaunchConfiguration("world")
    robot_name = LaunchConfiguration("robot_name")
    robot_x = LaunchConfiguration("robot_x")
    robot_y = LaunchConfiguration("robot_y")
    robot_z = LaunchConfiguration("robot_z")
    use_rviz = LaunchConfiguration("use_rviz")

    rx1_gazebo_share = get_package_share_directory("rx1_gazebo")
    rx1_description_share = get_package_share_directory("rx1_description")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    description_resource_root = os.path.dirname(rx1_description_share)
    gazebo_resource_root = os.path.dirname(rx1_gazebo_share)
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_path_entries = [
        entry
        for entry in [
            existing_resource_path,
            description_resource_root,
            gazebo_resource_root,
        ]
        if entry
    ]

    controllers_file = os.path.join(
        rx1_gazebo_share, "config", "rx1_harmonic_controllers.yaml"
    )
    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            "--verbosity",
            " 0 ",
            model,
            " ",
            "controllers_file:=",
            controllers_file,
            " ",
            "rgbd_topic_root:=/rx1/rgbd_camera",
        ]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": ["-r ", world]}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "robot_description": robot_description,
            }
        ],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        parameters=[
            {
                "name": robot_name,
                "topic": "robot_description",
                "x": robot_x,
                "y": robot_y,
                "z": robot_z,
            }
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/rx1/rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/rx1/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/rx1/rgbd_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/rx1/rgbd_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
            "--switch-timeout",
            "30.0",
        ],
        output="screen",
    )

    trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rx1_joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
            "--switch-timeout",
            "30.0",
        ],
        output="screen",
    )

    launch_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            # Gazebo may report entity creation before gz_ros2_control has fully
            # brought up controller_manager services. A short delay avoids a flaky
            # race where the spawner exits before /controller_manager is ready.
            on_exit=[TimerAction(period=5.0, actions=[joint_state_broadcaster_spawner])],
        )
    )

    launch_trajectory_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[trajectory_controller_spawner],
        )
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(rx1_gazebo_share, "rviz", "gazebo.rviz"),
        ],
        parameters=[
            {
                "use_sim_time": True,
                "robot_description": robot_description,
            }
        ],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    actions = [
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
        DeclareLaunchArgument("use_rviz", default_value="false"),
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=":".join(resource_path_entries),
        ),
        gazebo,
        robot_state_publisher,
        bridge,
        spawn_robot,
        launch_joint_state_broadcaster,
        launch_trajectory_controller,
        rviz,
    ]

    return LaunchDescription(actions)
