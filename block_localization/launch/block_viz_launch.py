from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ns = LaunchConfiguration("namespace")
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="sierra"),
        Node(
            package="block_localization",
            executable="block_viz",
            name="block_viz",
            namespace=ns,
            output="screen",
            parameters=[{"port": 8556}],
        ),
    ])
