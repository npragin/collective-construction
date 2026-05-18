import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import ParseMultiRobotPose
from launch.conditions import IfCondition

from launch_ros.actions import Node




def generate_launch_description():
    """
    Bring up the multi-robots with given launch arguments.

    Launch arguments consist of robot name(which is namespace) and pose for initialization.
    Keep general yaml format for pose information.
    ex) robots:='robot1={x: 1.0, y: 1.0, yaw: 1.5707}; robot2={x: 1.0, y: 1.0, yaw: 1.5707}'
    ex) robots:='robot3={x: 1.0, y: 1.0, z: 1.0, roll: 0.0, pitch: 1.5707, yaw: 1.5707};
                 robot4={x: 1.0, y: 1.0, z: 1.0, roll: 0.0, pitch: 1.5707, yaw: 1.5707}'
    """

    bringup_dir = get_package_share_directory('nav2_bringup')

    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_settings = LaunchConfiguration('log_settings', default='true')


    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='/home/liam-bouffard/Desktop/multiple_robot_systems/collective-construction/collective-construction/maps/koz_map.yaml',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        # default_value='/home/liam-bouffard/Desktop/multiple_robot_systems/collective-construction/nav2_params.yaml',
        default_value='/home/liam-bouffard/Desktop/multiple_robot_systems/collective-construction/nav2_params.yaml',

    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
    )

    """ParseMultiRobotPose bypasses the normal ROS2 launch argument system entirely — it reads directly from sys.argv (the raw command line) itself."""
    # robots_list = ParseMultiRobotPose('robots').value()
    robots_list = ['robot_0', 'robot_1']

    ld = LaunchDescription()

    tf_relay_nodes = []
    bringup_cmd_group = []
    for robot_name in robots_list:
        # init_pose = robots_list[robot_name]

        tf_relay_nodes.append(Node(
            package='scouts',
            executable='tf_relay',
            name=f'tf_relay_{robot_name}',
            parameters=[{'robot_name': robot_name}]
        ))

        group = GroupAction([
            # LogInfo(msg=['Launching namespace=', robot_name, ' init_pose=', str(init_pose)]),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    '/home/liam-bouffard/Desktop/multiple_robot_systems/collective-construction/collective-construction/scouts/launch/bringup_launch.py'
                ),
                launch_arguments={
                    'namespace': robot_name,
                    'use_namespace': 'true',
                    'use_sim_time': 'true',
                    'map': map_yaml_file,
                    'params_file': params_file,
                    'autostart': autostart
                }.items()
            ),
        ])

        bringup_cmd_group.append(group)

    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(LogInfo(msg=['number_of_robots=', str(len(robots_list))]))

    ld.add_action(LogInfo(msg=['number_of_robots=', str(len(robots_list))]))
    
    ld.add_action(
        LogInfo(condition=IfCondition(log_settings), msg=['map yaml: ', map_yaml_file])
    )
    ld.add_action(
        LogInfo(condition=IfCondition(log_settings), msg=['params yaml: ', params_file])
    )

    for node in tf_relay_nodes:
        ld.add_action(node)

    for cmd in bringup_cmd_group:
        ld.add_action(cmd)

    return ld