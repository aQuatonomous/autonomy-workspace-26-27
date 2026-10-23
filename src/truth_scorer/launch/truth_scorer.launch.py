#!/usr/bin/env python3
"""Launch the scorer, and optionally the evaluator alongside it.

The two are separate nodes on purpose. truth_scorer_node never sees the answer key, which is what
keeps it deployable at competition where no answer key exists. truth_eval_node is the thing that
knows the truth, and it is pure instrumentation: disabling it changes no published topic.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('truth_scorer')
    default_config = os.path.join(share, 'config', 'truth_scorer.yaml')
    default_confusion = os.path.join(share, 'config', 'confusion_prior.yaml')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('scorer_config_file', default_value=default_config),
        DeclareLaunchArgument('confusion_prior_file', default_value=default_confusion),
        DeclareLaunchArgument('fused_buoys_topic', default_value='/fused_buoys'),
        DeclareLaunchArgument('boat_pose_topic', default_value='/boat_pose'),
        DeclareLaunchArgument('scored_objects_topic', default_value='/scored_objects'),
        DeclareLaunchArgument('enable_eval', default_value='true',
                              description='Run truth_eval_node. Off at competition.'),
        DeclareLaunchArgument('answer_key', default_value='',
                              description='Path to the course answer_key.json. Empty = autodetect '
                                          'from $SIM_COURSE_DIR / $SIM_WORLD.'),
        DeclareLaunchArgument('report_dir', default_value='',
                              description='Where to write the report. Empty = $SIM_RUNS_DIR.'),
    ]

    scorer = Node(
        package='truth_scorer',
        executable='truth_scorer_node',
        name='truth_scorer_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'scorer_config_file': LaunchConfiguration('scorer_config_file'),
            'confusion_prior_file': LaunchConfiguration('confusion_prior_file'),
            'fused_buoys_topic': LaunchConfiguration('fused_buoys_topic'),
            'boat_pose_topic': LaunchConfiguration('boat_pose_topic'),
            'scored_objects_topic': LaunchConfiguration('scored_objects_topic'),
        }],
    )

    evaluator = Node(
        package='truth_scorer',
        executable='truth_eval_node',
        name='truth_eval_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_eval')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'scored_objects_topic': LaunchConfiguration('scored_objects_topic'),
            'boat_pose_topic': LaunchConfiguration('boat_pose_topic'),
            'answer_key': LaunchConfiguration('answer_key'),
            'report_dir': LaunchConfiguration('report_dir'),
        }],
    )

    return LaunchDescription(args + [scorer, evaluator])
