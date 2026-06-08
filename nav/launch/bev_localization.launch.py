"""Launch the BEV-fused global EKF that reconnects world -> odom -> base_link.

Starts a second robot_localization EKF (world frame) plus the bev_pose_bridge
that feeds it the overhead-camera pose. The robot's existing local EKF
(odom -> base_link) and the cc_localization overhead node are launched
elsewhere; this file only adds the global fusion layer.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use simulation clock'),
    DeclareLaunchArgument('namespace', default_value='j100_0897',
                          description='Robot namespace'),
    DeclareLaunchArgument('tag_id', default_value='15',
                          description='ArUco tag id mounted on this robot'),
    DeclareLaunchArgument('marker_offset_x', default_value='-0.365',
                          description='base_link->marker x offset [m]'),
    DeclareLaunchArgument('marker_offset_y', default_value='0.0',
                          description='base_link->marker y offset [m]'),
    DeclareLaunchArgument('marker_offset_yaw', default_value='0.0',
                          description='base_link->marker yaw offset [rad]'),
]


def generate_launch_description():
    pkg_nav = get_package_share_directory('nav')
    ekf_params = os.path.join(pkg_nav, 'config', 'ekf_world.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')

    group = GroupAction([
        PushRosNamespace(namespace),

        # Global EKF: publishes world -> odom by fusing odom/imu + BEV pose.
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_world',
            output='screen',
            parameters=[ekf_params, {'use_sim_time': use_sim_time}],
            remappings=[
                ('odometry/filtered', 'platform/odom/filtered/global'),
                # robot_localization uses ABSOLUTE /tf; pull it into the
                # namespace (like Clearpath's bringup) so the EKF reads
                # odom->base_link from /j100_0897/tf and publishes
                # world->odom back to the same topic.
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],
        ),

        # Overhead-camera marker -> base_link pose feeding the global EKF.
        Node(
            package='nav',
            executable='bev_pose_bridge',
            name='bev_pose_bridge',
            output='screen',
            # The camera (cc_localization) publishes world -> aruco_<tag> on the
            # GLOBAL /tf. The tf2 listener subscribes to the relative 'tf',
            # which would resolve to /j100_0897/tf under the namespace, so remap
            # it back out to the global /tf where `world` actually lives.
            remappings=[
                ('tf', '/tf'),
                ('tf_static', '/tf_static'),
            ],
            parameters=[{
                'use_sim_time': use_sim_time,
                'tag_id': ParameterValue(
                    LaunchConfiguration('tag_id'), value_type=int),
                'marker_offset_x': ParameterValue(
                    LaunchConfiguration('marker_offset_x'), value_type=float),
                'marker_offset_y': ParameterValue(
                    LaunchConfiguration('marker_offset_y'), value_type=float),
                'marker_offset_yaw': ParameterValue(
                    LaunchConfiguration('marker_offset_yaw'), value_type=float),
            }],
        ),

        # Relay the camera's world->build onto the robot /tf so robot-side
        # nodes can transform build against the arm. NO tf remap here: it
        # subscribes to the global /tf (absolute) and broadcasts to the
        # namespaced /tf (relative).
        Node(
            package='nav',
            executable='tf_relay',
            name='tf_relay',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'frames': ['build'],
                'robot_namespace': namespace,
            }],
        ),
    ])

    return LaunchDescription(ARGUMENTS + [group])
