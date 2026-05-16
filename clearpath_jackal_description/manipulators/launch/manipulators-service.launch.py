from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Include Packages
    pkg_clearpath_manipulators = FindPackageShare('clearpath_manipulators')

    # Declare launch files
    launch_file_manipulators = PathJoinSubstitution([
        pkg_clearpath_manipulators, 'launch', 'manipulators.launch.py'])

    # Include launch files
    launch_manipulators = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_manipulators]),
        launch_arguments=
            [
                (
                    'setup_path'
                    ,
                    '/home/jn2-alt/college/2025-2026/spring/ROB599_multi_robot/collective-construction/clearpath_jackal_description'
                )
                ,
                (
                    'use_sim_time'
                    ,
                    'false'
                )
                ,
                (
                    'namespace'
                    ,
                    'j100_0897'
                )
                ,
                (
                    'launch_moveit'
                    ,
                    'false'
                )
                ,
                (
                    'delay_moveit'
                    ,
                    '5.0'
                )
                ,
            ]
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_manipulators)
    return ld
