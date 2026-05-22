"""Run the pick state-machine demo.

Requires the AbsoluteMove action server and the gripper controller to already be
up (see manipulator_bringup.launch.py and the Clearpath manipulator service on
the Jackal).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='j100_0897/manipulators',
        description='Namespace the gripper action server lives under '
                    '(e.g. /<namespace>/arm_0_gripper_controller/gripper_cmd).',
    )

    pick = Node(
        package='manipulator',
        executable='pick',
        name='pick',
        output='screen',
        parameters=[{'namespace': LaunchConfiguration('namespace')}],
    )

    return LaunchDescription([
        namespace_arg,
        pick,
    ])
