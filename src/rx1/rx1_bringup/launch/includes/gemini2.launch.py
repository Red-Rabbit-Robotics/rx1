from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arg_names = {
        "camera_name": "camera",
        "depth_registration": "true",
        "serial_number": "",
        "usb_port": "",
        "device_num": "1",
        "vendor_id": "0x2bc5",
        "product_id": "",
        "enable_point_cloud": "true",
        "enable_colored_point_cloud": "true",
        "connection_delay": "100",
        "color_width": "640",
        "color_height": "360",
        "color_fps": "15",
        "enable_color": "true",
        "color_format": "MJPG",
        "flip_color": "false",
        "enable_color_auto_exposure": "true",
        "depth_width": "640",
        "depth_height": "400",
        "depth_fps": "15",
        "enable_depth": "true",
        "depth_format": "Y14",
        "flip_depth": "false",
        "depth_precision": "1mm",
        "ir_width": "640",
        "ir_height": "400",
        "ir_fps": "15",
        "enable_ir": "true",
        "ir_format": "Y8",
        "flip_ir": "false",
        "enable_ir_auto_exposure": "true",
        "enable_accel": "false",
        "accel_rate": "100hz",
        "accel_range": "4g",
        "enable_gyro": "false",
        "gyro_rate": "100hz",
        "gyro_range": "1000dps",
        "liner_accel_cov": "0.01",
        "angular_vel_cov": "0.01",
        "publish_tf": "true",
        "tf_publish_rate": "10.0",
        "ir_info_uri": "",
        "color_info_uri": "",
        "log_level": "none",
        "enable_d2c_viewer": "false",
        "enable_pipeline": "true",
        "enable_ldp": "true",
        "enable_soft_filter": "true",
        "soft_filter_max_diff": "-1",
        "soft_filter_speckle_size": "-1",
        "sync_mode": "free_run",
        "depth_delay_us": "0",
        "color_delay_us": "0",
        "trigger2image_delay_us": "0",
        "trigger_out_delay_us": "0",
        "trigger_out_enabled": "false",
        "depth_work_mode": "",
        "enable_frame_sync": "false",
    }

    declare_args = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in arg_names.items()
    ]

    parameters = {name: LaunchConfiguration(name) for name in arg_names}
    camera_name = LaunchConfiguration("camera_name")

    return LaunchDescription(
        declare_args
        + [
            Node(
                package="orbbec_camera",
                executable="orbbec_camera_node",
                namespace=camera_name,
                name="camera",
                output="screen",
                parameters=[parameters],
                remappings=[
                    (
                        ["/", camera_name, "/depth/color/points"],
                        ["/", camera_name, "/depth_registered/points"],
                    )
                ],
            )
        ]
    )
