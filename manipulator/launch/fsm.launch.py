from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='j100_0897/manipulators',
        description='Namespace for robot components',
    )

    fsm = Node(
        package='manipulator', 
        executable='fsm',
        name='fsm',
        output='screen',
        parameters=[{
            'namespace': LaunchConfiguration('namespace')
        }],
    )

    return LaunchDescription([
        namespace_arg,
        fsm,
    ])