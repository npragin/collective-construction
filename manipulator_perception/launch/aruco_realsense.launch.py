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
                "marker_size": 0.054,
                "aruco_dictionary": "25h9",
                "target_id": -1,
                "publish_tf": True,
                "publish_rviz_markers": False,
                "arm_base_frame": "arm_0_base_link",
                "world_frame": "base_link",
                "num_samples": 10,
                "sample_timeout": 5.0,
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