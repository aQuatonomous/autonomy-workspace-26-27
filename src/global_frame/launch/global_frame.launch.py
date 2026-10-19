#!/usr/bin/env python3
"""Launch boat_state_node and detection_to_global_node."""

import launch
import launch.actions
import launch_ros.actions
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    boat_state = launch_ros.actions.Node(
        package="global_frame",
        executable="boat_state_node",
        name="boat_state_node",
        output="screen",
        parameters=[
            {"global_position_topic": "mavros/global_position/global"},
            {"heading_topic": "mavros/global_position/compass_hdg"},
            {"heading_topic_type": "compass_hdg"},
            {"use_first_fix_as_origin": True},
            {"publish_pose_stamped": True},
            {"publish_tf": True},
        ],
    )
    use_fused_arg = launch.actions.DeclareLaunchArgument(
        "use_fused_detections",
        default_value="true",
        description="If true, feed planning from /fused_buoys (LiDAR+CV fusion); if false, from /tracked_buoys_json (LiDAR only).",
    )
    publish_unknown_arg = launch.actions.DeclareLaunchArgument(
        "publish_unknown_detections",
        default_value="false",
        description="If true, include unknown (class_id 255) detections in /global_detections; if false, exclude them (default).",
    )
    detection_to_global = launch_ros.actions.Node(
        package="global_frame",
        executable="detection_to_global_node",
        name="detection_to_global_node",
        output="screen",
        parameters=[
            {"use_fused_detections": LaunchConfiguration("use_fused_detections", default="true")},
            {"publish_unknown_detections": LaunchConfiguration("publish_unknown_detections", default="false")},
        ],
    )
    return launch.LaunchDescription([use_fused_arg, publish_unknown_arg, boat_state, detection_to_global])
