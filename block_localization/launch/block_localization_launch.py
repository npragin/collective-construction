import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ns = LaunchConfiguration("namespace")
    rs_launch = os.path.join(
        get_package_share_directory("realsense2_camera"),
        "launch", "rs_launch.py",
    )
    return LaunchDescription([

        DeclareLaunchArgument("namespace", default_value="sierra"),
        
        # realsense
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch),
            launch_arguments={
                "camera_namespace":           ns,
                "rgb_camera.color_profile":   "1280x720x30",
                "depth_module.depth_profile": "1280x720x30",
                "enable_sync":                "true",
                "align_depth.enable":         "true",
                "initial_reset":              "true",
            }.items(),
        ),

        # block localization node
        Node(
            package="block_localization",
            executable="block_localization",
            name="block_localization",
            namespace=ns,
            output="screen",
            parameters=[{
                "aruco_dict":    "DICT_APRILTAG_16h5",
                "marker_size_m": 0.0413,
                "camera_xyz":    [0.10, 0.0, 0.30],
                "camera_rpy":    [0.0, 0.0, 0.0],
            }],
        ),
    ])
