"""
All-in-one LiDAR buoy pipeline: Unitree driver → range filter → detector → tracker → visualizer.
Optionally launches RViz. LiDAR is always /dev/ttyUSB0.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    lidar_port = '/dev/ttyUSB0'
    if not os.path.exists(lidar_port):
        raise RuntimeError(
            f"LiDAR port {lidar_port!r} not found. Is the LiDAR plugged in? Check: ls /dev/ttyUSB*"
        )
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='false',
        description='If true, launch RViz with buoy_pipeline config. Default false (no GUI).',
    )
    # Detector tuning: e.g. ros2 launch ... buoy_detector_log_level:=debug
    buoy_detector_log_level_arg = DeclareLaunchArgument(
        'buoy_detector_log_level', default_value='info', description='buoy_detector log level (e.g. debug)',
    )

    unitree_node = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        output='screen',
        parameters=[
            {'port': '/dev/ttyUSB0'},
            {'rotate_yaw_bias': 0.0},
            {'range_scale': 0.001},
            {'range_bias': 0.0},
            {'range_max': 50.0},
            {'range_min': 0.0},
            {'cloud_frame': 'unilidar_lidar'},
            {'cloud_topic': 'unilidar/cloud'},
            {'cloud_scan_num': 18},
            {'imu_frame': 'unilidar_imu'},
            {'imu_topic': 'unilidar/imu'},
        ],
    )

    # --- Range filter: manual rotations only (no TF). Output frame = base_link. ---
    range_filter_node = Node(
        package='pointcloud_filters',
        executable='lidar_range_filter',
        name='lidar_range_filter',
        output='screen',
        parameters=[
            {
                'input_topic': '/unilidar/cloud',
                'output_topic': '/points_filtered',
                'base_frame': 'base_link',
                'use_tf_transform': False,
                'rotate_cw_deg': 202.5,      # 202.5° clockwise around Z
                'rotate_cw_x_deg': -30.0,    # 30° counter-clockwise around positive X
                'rotate_ccw_z2_deg': 90.0,   # 90° counter-clockwise around Z (after X rotation)
                'rotate_ccw_y_deg': 0.0,
                'z_min': -0.37,
                'z_max': 5.0,                         # Reduce Z range too
                'range_max': 15.0,                    # Reduce range to get fewer points
                'fov_max_angle_from_x': 0.0,          # Disable FOV filter - see all around (360°)
                'enable_accumulation': True,          # Enable accumulation for denser cloud
                'accumulation_window': 0.6,
            }
        ],
    )

    # --- Buoy detector: edit these to tune sensitivity (higher = more detections, more false positives) ---
    buoy_detector_node = Node(
        package='pointcloud_filters',
        executable='buoy_detector',
        name='buoy_detector',
        output='screen',
        parameters=[
            {
                'input_topic': '/points_filtered',
                'output_topic': '/buoy_detections',
                'eps': 0.85,                          # DBSCAN neighbor distance (m); larger = merge more into one cluster
                'min_samples': 1,                     # min pts to form cluster (1 = very sensitive)
                'min_lateral_extent': 0.005,          # min blob size (m)
                'max_lateral_extent': 2.5,            # max blob size before reject as wall
                'min_points_final': 1,                # min pts to accept cluster (1 = accept tiny)
                'min_points_for_aspect_ratio': 999,   # skip aspect-ratio reject below this many pts (999 = effectively off)
                'small_cluster_max_points': 80,       # clusters <= this use isolation rule
                'small_cluster_isolation_radius': 0.6,
                'small_cluster_max_nearby': 25,       # small cluster ok if at most this many other pts in radius
                'confidence_scale': 8.0,              # pts needed for high confidence (lower = confident with fewer pts)
                'ransac_enabled': False,
                'ransac_iterations': 80,
                'ransac_distance_threshold': 0.15,
                'ransac_min_inlier_ratio': 0.3,
            }
        ],
    )

    # --- Buoy tracker: tuned for persistence (buoys stay visible longer, appear faster) ---
    buoy_tracker_node = Node(
        package='pointcloud_filters',
        executable='buoy_tracker',
        name='buoy_tracker',
        output='screen',
        parameters=[
            {
                'max_consecutive_misses': 50,         # frames before removing track (high = persistent)
                'min_observations_for_publish': 1,    # observations before track appears (low = fast)
                'min_observations_to_add': 2,         # observations before becoming a track (low = fast)
                'candidate_max_consecutive_misses': 8, # candidate persistence
                'position_alpha': 0.6,                # smoothing: 0.6 = 60% new pos, 40% old (smoother)
                'association_threshold': 0.9,         # distance (m) to match detection to existing track
            }
        ],
    )

    # LiDAR-only: shows raw detections as markers in RViz (topic /buoy_markers)
    buoy_visualizer_node = Node(
        package='pointcloud_filters',
        executable='buoy_visualizer',
        name='buoy_visualizer',
        output='screen',
    )
    # Fused (CV+LiDAR): shows tracked buoys with class from CV (topic /tracked_buoy_markers); only has data when fusion runs
    tracked_buoy_visualizer_node = Node(
        package='pointcloud_filters',
        executable='tracked_buoy_visualizer',
        name='tracked_buoy_visualizer',
        output='screen',
    )

    # RViz (optional)
    rviz_config_path = os.path.join(
        get_package_share_directory('pointcloud_filters'), 'rviz', 'buoy_pipeline.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='log',
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
    )

    return LaunchDescription([
        launch_rviz_arg,
        buoy_detector_log_level_arg,
        unitree_node,
        range_filter_node,
        buoy_detector_node,
        buoy_tracker_node,
        buoy_visualizer_node,
        tracked_buoy_visualizer_node,
        rviz_node,
    ])
