from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    aruco_node = Node(
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
    )

    return LaunchDescription([aruco_node])