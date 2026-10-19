"""
Launch audio_capturer_node, sound_signal, and message_node in one process.
Run from workspace root: ros2 launch sound_pipeline_launch sound_pipeline.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='audio_common',
            executable='audio_capturer_node',
            name='audio_capturer_node',
            output='screen',
        ),
        Node(
            package='sound_signal',
            executable='sound_signal',
            name='sound_signal',
            output='screen',
            parameters=[
                {'sensitivity': 5},  # Lower = more sensitive (default was 10)
                {'frequency': 800},
            ],
        ),
        Node(
            package='message_node',
            executable='message_node',
            name='message_node',
            output='screen',
        ),
    ])
