from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Include Packages
    pkg_clearpath_sensors = FindPackageShare('clearpath_sensors')

    # Declare launch files
    launch_file_intel_realsense = PathJoinSubstitution([
        pkg_clearpath_sensors, 'launch', 'intel_realsense.launch.py'])

    # Include launch files
    launch_intel_realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_intel_realsense]),
        launch_arguments=
            [
                (
                    'parameters'
                    ,
                    '/home/jn2-alt/college/2025-2026/spring/ROB599_multi_robot/collective-construction/clearpath_jackal_description/sensors/config/camera_0.yaml'
                )
                ,
                (
                    'namespace'
                    ,
                    'j100_0897/sensors/camera_0'
                )
                ,
                (
                    'robot_namespace'
                    ,
                    'j100_0897'
                )
                ,
            ]
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_intel_realsense)
    return ld
