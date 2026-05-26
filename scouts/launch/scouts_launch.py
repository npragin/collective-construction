
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
)
from launch import LaunchDescription


namespace = DeclareLaunchArgument(
    'namespace',
    default_value='sierra',
    description='Namespace for the node'
)

namespace = LaunchConfiguration('namespace')


def generate_launch_description():
    return LaunchDescription(
        [   
        namespace,

        Node(
            package='scouts',
            executable='map2odom_tf',
            name='map2odomo_tf',
        ),

        Node(
            package='scouts',
            executbale='odom2base_tf',
            name='odom2base_tf',
        ),

        Node(
            package="rosaria2",
            executable="rosaria2_debug",
            name="rosaria2_node",
            namespace=namespace,
            output="screen",
            remappings=[("pose", "odom")],
            parameters=[
                {
                    "port": "/dev/ttyUSB0",
                    "frame_id": "base_link",
                    "odom_frame_id": "odom",
                    "tf_prefix": "rename_when_launching",
                }
            ],
            arguments=["--ros-args", "--log-level", "warn"],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file_path),
            launch_arguments={
                "initial_reset": "true",
                "camera_namespace": namespace,
                "depth_module.depth_profile": "1280x720x30",
                "rgb_camera.color_profile": "1280x720x30",
                "enable_sync": "true",
            }.items(),
        ),

        ]

    )

