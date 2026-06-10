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

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Namespace move_group lives under.',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    aruco_detector = Node(
        package="aruco_depth_ros2",
        executable="aruco_depth_node",
        name="aruco_depth_node",
        output="screen",
        # This robot publishes TF on the namespaced /j100_0897/tf(_static)
        # topics. Remap so the node's TransformListener (and broadcaster) use
        # them instead of the empty global /tf; otherwise every lookup fails
        # with "frame does not exist".
        remappings=[
            ("/tf", "/j100_0897/tf"),
            ("/tf_static", "/j100_0897/tf_static"),
        ],
        parameters=[
            {
                "color_topic": "/j100_0897/sensors/camera_0/color/image",
                "depth_topic": "/j100_0897/sensors/camera_0/depth/image",
                "camera_info_topic": "/j100_0897/sensors/camera_0/color/camera_info",
                "depth_camera_info_topic": "/j100_0897/sensors/camera_0/depth/camera_info",
                "marker_size": 0.0385,
                "aruco_dictionary": "25h9",
                "target_id": -1,
                "publish_tf": False,
                "publish_rviz_markers": False,
                "arm_base_frame": "arm_0_base_link",
                "world_frame": "world",
                "show_window": False,
                "num_samples": 10,                                                                                      
                "sample_timeout": 5.0,   
            }
        ],
    )
    

    

    absolute_move = Node(
        package='manipulator',
        executable='absolute_move',
        name='absolute_move',
        output='screen',
        parameters=[{
            'namespace': namespace,
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('/tf', ['/', namespace, '/tf']),
            ('/tf_static', ['/', namespace, '/tf_static']),
        ],
    )

    blockscan = Node(
        package='manipulator',
        executable='blockscan',
        name='blockscan',
        output='screen',
        parameters=[{
            'namespace': namespace,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        absolute_move,
        aruco_detector,
        blockscan
    ])
