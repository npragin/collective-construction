from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("realsense2_camera"),
                "launch",
                "rs_launch.py"
            ])
        ),
        launch_arguments={
            "align_depth.enable": "true",
            "enable_color": "true",
            "enable_depth": "true",
            "enable_infra1": "false",
            "enable_infra2": "false",
            "enable_gyro": "false",
            "enable_accel": "false",
            "rgb_camera.profile": "640x480x15",
            "depth_module.profile": "640x480x15",
        }.items()
    )

    aruco_node = Node(
        package="aruco_depth_ros2",
        executable="aruco_depth_node",
        name="aruco_depth_node",
        output="screen",
        parameters=[
            {
                "color_topic": "/j100_0897/sensors/camera_0/color/image",
                "depth_topic": "/j100_0897/sensors/camera_0/depth/image",
                "camera_info_topic": "/j100_0897/sensors/camera_0/color/camera_info",
                "marker_size": 0.05,
                "aruco_dictionary": "original",
                "target_id": -1,
                "publish_tf": True,
                "publish_rviz_markers": True,
                "arm_base_frame": "arm_0_base_link",
                "world_frame": "base_link",
                "show_window": False,
            }
        ]
    )

    return LaunchDescription([
        realsense_launch,
        aruco_node,
    ])
