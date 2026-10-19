#!/usr/bin/env python3
"""
Launch file for map visualizer node.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for map visualizer."""
    
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8080',
        description='HTTP server port'
    )
    
    update_rate_arg = DeclareLaunchArgument(
        'update_rate_hz',
        default_value='10.0',
        description='WebSocket broadcast rate (Hz)'
    )
    
    boat_pose_topic_arg = DeclareLaunchArgument(
        'boat_pose_topic',
        default_value='/boat_pose',
        description='Boat pose topic'
    )
    
    global_detections_topic_arg = DeclareLaunchArgument(
        'global_detections_topic',
        default_value='/global_detections',
        description='Global detections topic'
    )
    
    # Map visualizer node
    map_visualizer_node = Node(
        package='web_server_map',
        executable='map_visualizer_node',
        name='map_visualizer_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'update_rate_hz': LaunchConfiguration('update_rate_hz'),
            'boat_pose_topic': LaunchConfiguration('boat_pose_topic'),
            'global_detections_topic': LaunchConfiguration('global_detections_topic'),
        }]
    )
    
    return LaunchDescription([
        port_arg,
        update_rate_arg,
        boat_pose_topic_arg,
        global_detections_topic_arg,
        map_visualizer_node,
    ])
