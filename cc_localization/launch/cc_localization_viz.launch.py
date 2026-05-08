"""Launch the ArUco tf node alongside RViz."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = Path(get_package_share_directory("cc_localization"))
    default_rviz = str(pkg_share / "rviz" / "aruco_tf.rviz")

    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
                description="Path to the RViz config file.",
            ),
            Node(
                package="cc_localization",
                executable="cc_localization",
                name="cc_localization",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
