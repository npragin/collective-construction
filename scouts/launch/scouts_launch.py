from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import os


# Launch argument
namespace_arg = DeclareLaunchArgument(
    "namespace",
    default_value="sierra",
    description="Namespace for the robot",
)

namespace = LaunchConfiguration("namespace")

# RealSense launch file
realsense_pkg_dir = get_package_share_directory("realsense2_camera")
realsense_launch = os.path.join(
    realsense_pkg_dir,
    "launch",
    "rs_launch.py",
)

scouts_pkg_dir = get_package_share_directory('scouts')

map_yaml = os.path.join(scouts_pkg_dir, 'nav2_files', 'map.yaml')

nav2_params = os.path.join(scouts_pkg_dir, 'nav2_files', 'nav2_params.yaml')

nav2_launch = os.path.join(scouts_pkg_dir, 'launch', 'bringup_launch.py')

def generate_launch_description():

    return LaunchDescription([
        namespace_arg,

        # map -> odom TF broadcaster
        Node(
            package="scouts",
            executable="map2odom_tf",
            name="map2odom_tf",
            namespace=namespace,
            output="screen",
        ),

        # Rosaria pioneer driver
        Node(
            package="rosaria2",
            executable="rosaria2_debug",
            name="rosaria2_node",
            namespace=namespace,
            output="screen",
            remappings=[
                ("pose", "odom"),
            ],
            parameters=[
                {
                    "port": "/dev/ttyUSB0",
                    "frame_id": "sierra/base_link",
                    "odom_frame_id": "sierra/odom",
                    "tf_prefix": "rename_when_launching",
                }
            ],
            arguments=[
                "--ros-args",
                "--log-level",
                "warn",
            ],
        ),

        # # RealSense camera
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(realsense_launch),
        #     launch_arguments={
        #         "initial_reset": "true",
        #         "camera_namespace": namespace,
        #         "depth_module.depth_profile": "1280x720x30",
        #         "rgb_camera.color_profile": "1280x720x30",
        #         "enable_sync": "true",
        #     }.items(),
        # ),

        # Node(
        #     package='scouts',
        #     executable='check_tags',
        #     name='check_tags',
        #     namespace=namespace,
        #     output='screen'
        # )

        # Node(
        #     package="scouts",
        #     executable="waypoint_server",
        #     name="waypoint_server",
        #     namespace=namespace,
        #     output="screen",
        # ),

    ])