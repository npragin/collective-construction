"""
Launch the planner node alongside the scripted test harness.

Usage
-----
    ros2 launch cc_planner test_planner.launch.py
    ros2 launch cc_planner test_planner.launch.py scenario_file:=/abs/path/to/scenario.yaml
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Generate the launch description for the planner test harness.

    Returns
    -------
    LaunchDescription
        Launch description containing the planner and test harness nodes.

    """
    default_scenario = str(Path(get_package_share_directory("cc_planner")) / "scenarios" / "full_workflow.yaml")

    scenario_arg = DeclareLaunchArgument(
        "scenario_file",
        default_value=default_scenario,
        description="Absolute path to the scenario YAML file to run",
    )
    scenario_file = LaunchConfiguration("scenario_file")

    planner = Node(
        package="cc_planner",
        executable="cc_planner",
        name="planner",
        output="screen",
    )
    harness = Node(
        package="cc_planner",
        executable="test_harness",
        name="test_harness",
        output="screen",
        parameters=[{"scenario_file": scenario_file}],
    )
    return LaunchDescription([scenario_arg, planner, harness])
