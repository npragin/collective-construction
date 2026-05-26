from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='j100_0897',
        description='Namespace for robot components',
    )

    manipulator_namespace_arg = DeclareLaunchArgument(
        'manipulator_namespace',
        default_value='j100_0897/manipulators',
        description='Namespace for manipulator components',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Namespace move_group lives under.',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')


    fsm = Node(
        package='manipulator', 
        executable='fsm',
        name='fsm',
        output='screen',
        parameters=[{
            'namespace': LaunchConfiguration('namespace'),
            'manipulator_namespace': LaunchConfiguration('manipulator_namespace'),
            'use_sim_time': use_sim_time
        }],
    )

    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        manipulator_namespace_arg,
        fsm,
        
    ])