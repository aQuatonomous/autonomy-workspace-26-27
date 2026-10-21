"""The team's LiDAR buoy pipeline (pointcloud_filters/launch/buoy_pipeline.launch.py) on the
simulated Airy: same nodes and tuning, minus the Unitree driver and serial-port check. The
simulated cloud on /rslidar_points is already in boat orientation (x forward, z up), so the
Unitree mount rotations are 0. Keep the tuning below in step with buoy_pipeline.launch.py."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='pointcloud_filters', executable='lidar_range_filter', name='lidar_range_filter',
            output='screen',
            parameters=[{
                'input_topic': '/rslidar_points',
                'output_topic': '/points_filtered',
                'base_frame': 'base_link',
                'use_tf_transform': False,
                'rotate_cw_deg': 0.0,
                'rotate_cw_x_deg': 0.0,
                'rotate_ccw_z2_deg': 0.0,
                'rotate_ccw_y_deg': 0.0,
                'z_min': -0.37,
                'z_max': 5.0,
                'range_max': 15.0,
                'fov_max_angle_from_x': 0.0,
                'enable_accumulation': True,
                'accumulation_window': 0.6,
            }],
        ),
        Node(
            package='pointcloud_filters', executable='buoy_detector', name='buoy_detector',
            output='screen',
            parameters=[{
                'input_topic': '/points_filtered',
                'output_topic': '/buoy_detections',
                'eps': 0.85,
                'min_samples': 1,
                'min_lateral_extent': 0.005,
                'max_lateral_extent': 2.5,
                'min_points_final': 1,
                'min_points_for_aspect_ratio': 999,
                'small_cluster_max_points': 80,
                'small_cluster_isolation_radius': 0.6,
                'small_cluster_max_nearby': 25,
                'confidence_scale': 8.0,
                'ransac_enabled': False,
                'ransac_iterations': 80,
                'ransac_distance_threshold': 0.15,
                'ransac_min_inlier_ratio': 0.3,
            }],
        ),
        Node(
            package='pointcloud_filters', executable='buoy_tracker', name='buoy_tracker',
            output='screen',
            parameters=[{
                'max_consecutive_misses': 50,
                'min_observations_for_publish': 1,
                'min_observations_to_add': 2,
                'candidate_max_consecutive_misses': 8,
                'position_alpha': 0.6,
                'association_threshold': 0.9,
            }],
        ),
        Node(package='pointcloud_filters', executable='buoy_visualizer', name='buoy_visualizer', output='screen'),
        Node(package='pointcloud_filters', executable='tracked_buoy_visualizer', name='tracked_buoy_visualizer',
             output='screen'),
    ])
