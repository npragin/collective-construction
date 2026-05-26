"""Bring up the AbsoluteMove action server. Run this once after MoveIt's
move_group is reachable; then send goals (e.g. via `pick`) from a separate
terminal.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='j100_0897',
        description='Namespace move_group lives under.',
    )

    namespace = LaunchConfiguration('namespace')

    absolute_move = Node(
        package='manipulator',
        executable='absolute_move',
        name='absolute_move',
        output='screen',
        parameters=[{'namespace': namespace}],
        remappings=[
            ('/tf', ['/', namespace, '/tf']),
            ('/tf_static', ['/', namespace, '/tf_static']),
        ],
    )

    return LaunchDescription([
        namespace_arg,
        absolute_move,
    ])
