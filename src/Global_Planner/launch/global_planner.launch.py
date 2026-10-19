"""
Launch global_planner_node with PLANNING_PATH set so it can import the planning library.

Usage (from workspace root after colcon build):
  ros2 launch global_planner global_planner.launch.py

  # With task 2:
  ros2 launch global_planner global_planner.launch.py task_id:=2

  # Custom cmd_vel topic:
  ros2 launch global_planner global_planner.launch.py cmd_vel_topic:=/cmd_vel

  # With static map bounds (min_x, min_y, max_x, max_y in meters):
  ros2 launch global_planner global_planner.launch.py map_bounds:="[-50.0,-50.0,50.0,50.0]"

  # Task 4 with pump via pump_hold.py (full path to script):
  ros2 launch global_planner global_planner.launch.py task_id:=4 pump_hold_script:=/absolute/path/to/autonomy-ws-25-26/pump_hold.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("global_planner")
    # planning is typically in same workspace src/planning
    planning_path = os.path.abspath(os.path.join(share, "..", "..", "..", "src", "planning"))
    if not os.path.isdir(planning_path):
        planning_path = os.path.abspath(os.path.join(share, "..", "..", "..", "..", "planning"))
    if not os.path.isdir(planning_path):
        # Colcon workspace IS the planning dir (planning/install/... -> planning/)
        planning_path = os.path.abspath(os.path.join(share, "..", "..", "..", ".."))

    return LaunchDescription([
        DeclareLaunchArgument("task_id", default_value="1", description="Task ID: 0 (test), 1, 2, 3, or 4"),
        DeclareLaunchArgument("planning_hz", default_value="10.0", description="Planning loop frequency (Hz)"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/mavros/setpoint_velocity/cmd_vel_unstamped"),
        DeclareLaunchArgument("map_bounds", default_value="[]", description="Static map bounds [min_x, min_y, max_x, max_y] in meters; empty = dynamic from gates/entities"),
        DeclareLaunchArgument("pump_hold_script", default_value="", description="Full path to pump_hold.py for Task4 pump control; empty = disabled"),
        SetEnvironmentVariable(name="PLANNING_PATH", value=planning_path),
        Node(
            package="global_planner",
            executable="global_planner_node",
            name="global_planner_node",
            output="screen",
            parameters=[{
                "task_id": LaunchConfiguration("task_id"),
                "planning_hz": LaunchConfiguration("planning_hz"),
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                "map_bounds": LaunchConfiguration("map_bounds"),
                "pump_hold_script": LaunchConfiguration("pump_hold_script"),
            }],
        ),
    ])
