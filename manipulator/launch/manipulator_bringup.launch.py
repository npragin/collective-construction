"""Bring up the AbsoluteMove action server and seed the planning scene with the
collision box. Run this once after MoveIt's move_group is reachable; then send
goals (e.g. via `pick`) from a separate terminal.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='j100_0897',
        description='Namespace move_group and apply_planning_scene live under.',
    )
    box_frame_arg = DeclareLaunchArgument(
        'box_frame_id',
        default_value='base_link',
        description='Frame the planning-scene box is anchored to.',
    )
    box_position_arg = DeclareLaunchArgument(
        'box_position',
        default_value='[0.15, 0.0, 0.25]',
        description='Box centre position [x, y, z] in box_frame_id.',
    )
    box_size_arg = DeclareLaunchArgument(
        'box_size',
        default_value='[0.15, 0.40, 0.04]',
        description='Box dimensions [x, y, z].',
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

    # Remove this once collision box added to robot description.
    add_collision_box = Node(
        package='manipulator',
        executable='add_collision_box',
        name='add_collision_box',
        output='screen',
        parameters=[{
            'namespace': namespace,
            'frame_id': LaunchConfiguration('box_frame_id'),
            'position': LaunchConfiguration('box_position'),
            'size': LaunchConfiguration('box_size'),
        }],
    )

    return LaunchDescription([
        namespace_arg,
        box_frame_arg,
        box_position_arg,
        box_size_arg,
        absolute_move,
        add_collision_box,
    ])
