from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def setup_launch_arguments(context, *args, **kwargs):
    debug_str = LaunchConfiguration('debug', default='False').perform(context)
    debug = debug_str.lower() in ['true', '1', 't', 'yes']
    print(f"Debug mode: {debug}")  
    if debug:
        config = {
            'arguments': ["--ros-args", "--log-level", "debug"],
        }
    else:
        config = {}
    return [Node(
        package="aruco_depth_ros2",
        executable="aruco_depth_node",
        name="aruco_depth_node",
        output="screen",
        parameters=[
            {
                "color_topic": "/j100_0897/sensors/camera_0/color/image",
                "depth_topic": "/j100_0897/sensors/camera_0/depth/image",
                "camera_info_topic": "/j100_0897/sensors/camera_0/color/camera_info",
                "marker_size": 0.10,
                "aruco_dictionary": "4x4_50",
                "target_id": -1,
                "publish_tf": True,
                "publish_rviz_markers": True,
                "arm_base_frame": "arm_0_base_link",
                "world_frame": "base_link",
                "show_window": False,
            }
        ],
        **config
    )]

def generate_launch_description():

    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='False',
        description='Enable debug logging for the ArUco node.',
    )
    aruco_node = OpaqueFunction(function=setup_launch_arguments) 


    

    return LaunchDescription([aruco_node])