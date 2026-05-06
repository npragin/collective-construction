from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Include Packages

    # Declare launch files
    launch_file_lidar2d_0 = '/home/jn2-alt/college/2025-2026/spring/ROB599_multi_robot/collective-construction/clearpath_jackal_description/sensors/launch/lidar2d_0.launch.py'
    launch_file_lidar2d_1 = '/home/jn2-alt/college/2025-2026/spring/ROB599_multi_robot/collective-construction/clearpath_jackal_description/sensors/launch/lidar2d_1.launch.py'
    launch_file_camera_0 = '/home/jn2-alt/college/2025-2026/spring/ROB599_multi_robot/collective-construction/clearpath_jackal_description/sensors/launch/camera_0.launch.py'

    # Include launch files
    launch_lidar2d_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_lidar2d_0]),
    )

    launch_lidar2d_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_lidar2d_1]),
    )

    launch_camera_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_camera_0]),
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_lidar2d_0)
    ld.add_action(launch_lidar2d_1)
    ld.add_action(launch_camera_0)
    return ld
