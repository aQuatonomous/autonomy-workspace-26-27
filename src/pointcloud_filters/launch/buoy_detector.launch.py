from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='pointcloud_filters',
            executable='buoy_detector',
            name='buoy_detector',
            output='screen',
            parameters=[
                {
                    'input_topic': '/points_filtered',
                    'output_topic': '/buoy_detections',
                    
                    # DBSCAN clustering parameters
                    'eps': 0.8,              # Distance threshold (meters) - TUNE THIS!
                    'min_samples': 5,        # Minimum points per cluster
                    
                    # Validation thresholds
                    'min_lateral_extent': 0.2,   # Minimum buoy size (meters)
                    'max_lateral_extent': 3.0,   # Maximum buoy size (meters)
                    'min_points_final': 3,       # Final point count check
                    'confidence_scale': 15.0,    # Points for 100% confidence
                    
                    # RANSAC water plane removal (for tilted LiDAR)
                    'ransac_enabled': False,         # Water plane removal (default off)
                    'ransac_iterations': 80,         # Number of random samples (lower = less latency)
                    'ransac_distance_threshold': 0.15,  # Max distance to plane (meters)
                    'ransac_min_inlier_ratio': 0.3,   # Min fraction of points in plane
                }
            ],
        ),
    ])
