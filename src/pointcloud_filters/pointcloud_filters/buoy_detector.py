#!/usr/bin/env python3
"""
Buoy Detector Node - Uses DBSCAN clustering to detect buoys from filtered LiDAR point clouds.

Subscribes to: /points_filtered (sensor_msgs/PointCloud2)
Publishes to:  /buoy_detections (pointcloud_filters/BuoyDetectionArray)

Algorithm:
1. Cluster points using DBSCAN in Cartesian (x, y) coordinates
2. Extract cluster centroids and convert to polar (range, bearing)
3. Validate cluster size and publish detections
"""

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

from sklearn.cluster import DBSCAN

# Custom messages (will be generated)
from pointcloud_filters.msg import BuoyDetection, BuoyDetectionArray


class BuoyDetectorNode(Node):
    def __init__(self):
        super().__init__('buoy_detector')

        # Parameters
        self.declare_parameter('input_topic', '/points_filtered')
        self.declare_parameter('output_topic', '/buoy_detections')
        self.declare_parameter('eps', 0.8)  # DBSCAN distance threshold (m); larger helps small/sparse buoys form one cluster
        self.declare_parameter('min_samples', 2)  # DBSCAN minimum points per cluster
        self.declare_parameter('min_lateral_extent', 0.01)  # Minimum buoy size (m); allow very tight 2-point clusters
        self.declare_parameter('max_lateral_extent', 1.0)  # Maximum buoy size (meters); reject walls/extended surfaces
        self.declare_parameter('min_points_final', 2)  # Minimum points after validation
        self.declare_parameter('min_isolation_margin', 0.0)  # 0 = off. Non-zero rejects clusters near other points (can drop buoys near water).
        self.declare_parameter('max_aspect_ratio', 10.0)  # Max bbox aspect ratio; small clusters (2–3 pts along beam) are elongated, so keep high
        self.declare_parameter('min_points_for_aspect_ratio', 20)  # Only apply aspect-ratio filter when cluster has >= this many points; skip for small buoys (10–20 pts)
        self.declare_parameter('max_external_points_nearby', -1)  # -1 = off. Non-zero can reject buoys near water/ground points.
        self.declare_parameter('external_density_radius', 0.8)  # Radius (m) for counting nearby external points; use with max_external_points_nearby
        # Isolated small clusters: accept small blobs only when no/few other points nearby (solid cluster, nothing connected)
        self.declare_parameter('small_cluster_max_points', 30)  # Clusters with <= this many points are "small"
        self.declare_parameter('small_cluster_isolation_radius', 0.5)  # Radius (m) around cluster to check for other points
        self.declare_parameter('small_cluster_max_nearby', 4)  # Small cluster accepted only if at most this many other points in radius (else reject as not isolated)
        self.declare_parameter('confidence_scale', 15.0)  # Points for 100% confidence
        
        # RANSAC water plane removal parameters
        self.declare_parameter('ransac_enabled', False)  # RANSAC plane removal (default off)
        self.declare_parameter('ransac_iterations', 80)  # Number of RANSAC iterations (lower = less latency)
        self.declare_parameter('ransac_distance_threshold', 0.15)  # Max distance to plane (meters)
        self.declare_parameter('ransac_min_inlier_ratio', 0.3)  # Minimum fraction of points in plane

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.eps = self.get_parameter('eps').get_parameter_value().double_value
        self.min_samples = int(self.get_parameter('min_samples').get_parameter_value().integer_value)
        self.min_lateral_extent = self.get_parameter('min_lateral_extent').get_parameter_value().double_value
        self.max_lateral_extent = self.get_parameter('max_lateral_extent').get_parameter_value().double_value
        self.min_points_final = int(self.get_parameter('min_points_final').get_parameter_value().integer_value)
        self.min_isolation_margin = float(self.get_parameter('min_isolation_margin').get_parameter_value().double_value)
        self.max_aspect_ratio = float(self.get_parameter('max_aspect_ratio').get_parameter_value().double_value)
        self.min_points_for_aspect_ratio = int(self.get_parameter('min_points_for_aspect_ratio').get_parameter_value().integer_value)
        self.max_external_points_nearby = int(self.get_parameter('max_external_points_nearby').get_parameter_value().integer_value)
        self.external_density_radius = float(self.get_parameter('external_density_radius').get_parameter_value().double_value)
        self.small_cluster_max_points = int(self.get_parameter('small_cluster_max_points').get_parameter_value().integer_value)
        self.small_cluster_isolation_radius = float(self.get_parameter('small_cluster_isolation_radius').get_parameter_value().double_value)
        self.small_cluster_max_nearby = int(self.get_parameter('small_cluster_max_nearby').get_parameter_value().integer_value)
        self.confidence_scale = self.get_parameter('confidence_scale').get_parameter_value().double_value
        
        _re = self.get_parameter('ransac_enabled').get_parameter_value()
        try:
            self.ransac_enabled = _re.bool_value
        except Exception:
            self.ransac_enabled = str(getattr(_re, 'string_value', 'false')).strip().lower() in ('1', 'true', 'yes')
        self.ransac_iterations = int(self.get_parameter('ransac_iterations').get_parameter_value().integer_value)
        self.ransac_distance_threshold = self.get_parameter('ransac_distance_threshold').get_parameter_value().double_value
        self.ransac_min_inlier_ratio = self.get_parameter('ransac_min_inlier_ratio').get_parameter_value().double_value

        # QoS profile matching LiDAR driver (best effort)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscriber to filtered point cloud
        self.sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cloud_callback,
            qos
        )

        # Publisher for buoy detections (RELIABLE so tracker and visualizers can subscribe)
        qos_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(BuoyDetectionArray, self.output_topic, qos_pub)
        self._last_zero_buoys_log_ns = 0  # throttle "0 buoys" hint

        self.get_logger().info(
            f'Buoy detector started: input={self.input_topic}, output={self.output_topic}, '
            f'eps={self.eps}m, min_samples={self.min_samples}'
        )

    def cloud_callback(self, msg: PointCloud2) -> None:
        """Process incoming point cloud and detect buoys."""
        # Extract points from PointCloud2
        points = self.extract_points(msg)

        if len(points) == 0:
            self.get_logger().debug('Received empty point cloud')
            self.publish_detections([], msg.header)
            return

        # Remove water plane using RANSAC (if enabled)
        if self.ransac_enabled:
            points_filtered = self.remove_water_plane_ransac(points)
            if len(points_filtered) == 0:
                self.get_logger().debug('All points removed by RANSAC plane filtering')
                self.publish_detections([], msg.header)
                return
            points = points_filtered

        # Cluster points using DBSCAN
        labels = self.detect_buoys_dbscan(points)

        # Extract buoy detections from clusters
        buoys = self.extract_buoy_detections(points, labels)

        # Always publish (including empty) so tracker and visualizers update promptly
        self.publish_detections(buoys, msg.header)
        if len(buoys) > 0:
            self.get_logger().info(f'Detected {len(buoys)} buoy(s)')
        elif len(points) > 0:
            num_clusters = len(set(labels)) - (1 if -1 in labels else 0)
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_zero_buoys_log_ns > 5_000_000_000:  # throttle ~5 s
                self._last_zero_buoys_log_ns = now_ns
                if num_clusters == 0:
                    self.get_logger().warn(
                        f'0 buoys: DBSCAN found no clusters from {len(points)} pts. '
                        'Try larger eps or smaller min_samples.'
                    )
                else:
                    self.get_logger().warn(
                        f'0 buoys from {len(points)} pts ({num_clusters} clusters rejected). '
                        'Run with buoy_detector_log_level:=debug to see why.'
                    )

    def extract_points(self, msg: PointCloud2) -> np.ndarray:
        """
        Extract XYZ points from PointCloud2 message.
        
        Returns:
            Nx3 numpy array of (x, y, z) coordinates
        """
        # Determine available fields
        field_map = {f.name: f for f in msg.fields}
        has_xyz = all(f in field_map for f in ['x', 'y', 'z'])
        
        if not has_xyz:
            self.get_logger().error('PointCloud2 missing x, y, or z fields')
            return np.array([])

        # Read points (handle mixed datatypes robustly)
        try:
            pts_iter = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        except Exception as e:
            self.get_logger().error(f'Failed to read points: {e}')
            return np.array([])

        # Build list of (x, y, z) tuples
        values = []
        try:
            for p in pts_iter:
                # Handle both structured numpy records and plain sequences
                if hasattr(p, 'dtype') and p.dtype.names:
                    # Structured record (np.void)
                    values.append((float(p['x']), float(p['y']), float(p['z'])))
                else:
                    # Plain sequence
                    values.append((float(p[0]), float(p[1]), float(p[2])))
        except Exception as e:
            self.get_logger().error(f'Error parsing points: {e}')
            return np.array([])

        if len(values) == 0:
            return np.array([])

        # Convert to numpy array
        points = np.array(values, dtype=np.float64)
        return points

    def remove_water_plane_ransac(self, points: np.ndarray) -> np.ndarray:
        """
        Remove water plane using RANSAC plane fitting.
        
        Algorithm:
        1. Randomly sample 3 points to define a plane
        2. Count how many points are within distance_threshold of this plane (inliers)
        3. Repeat for ransac_iterations and keep the best plane (most inliers)
        4. Remove all inliers (water surface) and return outliers (buoys)
        
        A plane is defined by: n · p + d = 0
        where n = [a, b, c] is the unit normal vector and d is distance from origin.
        
        Args:
            points: Nx3 array (x, y, z)
        
        Returns:
            Filtered Nx3 array with water plane points removed
        """
        if len(points) < 3:
            return points
        
        best_inliers = None
        best_num_inliers = 0
        
        # RANSAC iterations
        for _ in range(self.ransac_iterations):
            # Step 1: Randomly sample 3 points
            indices = np.random.choice(len(points), 3, replace=False)
            sample_points = points[indices]
            
            # Step 2: Fit a plane through these 3 points
            # Plane is defined by two vectors in the plane
            v1 = sample_points[1] - sample_points[0]
            v2 = sample_points[2] - sample_points[0]
            
            # Normal vector = cross product of v1 and v2
            normal = np.cross(v1, v2)
            normal_length = np.linalg.norm(normal)
            
            # Skip if points are collinear (degenerate plane)
            if normal_length < 1e-6:
                continue
            
            # Normalize the normal vector
            n = normal / normal_length  # Unit normal [a, b, c]
            
            # Compute d: n · p0 + d = 0  =>  d = -n · p0
            d = -np.dot(n, sample_points[0])
            
            # Step 3: Count inliers (points close to this plane)
            # Distance from point p to plane: |n · p + d|
            distances = np.abs(np.dot(points, n) + d)
            inliers_mask = distances < self.ransac_distance_threshold
            num_inliers = np.sum(inliers_mask)
            
            # Update best plane if this one has more inliers
            if num_inliers > best_num_inliers:
                best_num_inliers = num_inliers
                best_inliers = inliers_mask
        
        # Check if we found a valid plane
        if best_inliers is None:
            self.get_logger().warn('RANSAC failed to find a plane')
            return points
        
        inlier_ratio = best_num_inliers / len(points)
        
        # Only remove plane if it has enough inliers (dominant surface = water)
        if inlier_ratio < self.ransac_min_inlier_ratio:
            self.get_logger().debug(
                f'RANSAC plane has {inlier_ratio:.1%} inliers (< {self.ransac_min_inlier_ratio:.1%} threshold), '
                'not removing - no dominant plane found'
            )
            return points
        
        # Remove inliers (water plane), keep outliers (buoys)
        filtered_points = points[~best_inliers]
        
        self.get_logger().info(
            f'RANSAC removed {best_num_inliers}/{len(points)} points '
            f'({inlier_ratio:.1%}) as water plane, {len(filtered_points)} remain'
        )
        
        return filtered_points

    def detect_buoys_dbscan(self, points: np.ndarray) -> np.ndarray:
        """
        Cluster LiDAR points into buoys using DBSCAN in Cartesian coordinates.
        
        Args:
            points: Nx3 array (x, y, z) in base_link frame
        
        Returns:
            labels: Cluster IDs for each point (-1 = noise)
        """
        if len(points) == 0:
            return np.array([])

        # Create DBSCAN clustering object
        # - Clusters in Cartesian (x, y) space
        # - Uses Euclidean distance metric
        # - Fast kd_tree algorithm for spatial indexing
        clustering = DBSCAN(
            eps=self.eps,              # Maximum distance for neighbors (meters)
            min_samples=self.min_samples,  # Minimum points for dense region
            metric='euclidean',        # Straight-line distance
            algorithm='kd_tree'        # Fast spatial search
        )

        # Cluster only in horizontal plane (x, y), ignore z
        # This prevents splitting a buoy vertically
        labels = clustering.fit_predict(points[:, :2])

        return labels

    def extract_buoy_detections(self, points: np.ndarray, labels: np.ndarray) -> list:
        """
        Convert DBSCAN clusters to buoy detections with polar coordinates.
        
        Args:
            points: Nx3 array (x, y, z) in Cartesian coordinates
            labels: Cluster IDs from DBSCAN
        
        Returns:
            List of buoy detection dicts with range, bearing, etc.
        """
        buoys = []
        unique_labels = set(labels)
        num_clusters = len(unique_labels) - (1 if -1 in unique_labels else 0)
        self.get_logger().debug(f'DBSCAN: {num_clusters} clusters (excluding noise), {np.sum(labels == -1)} noise pts')

        for label in unique_labels:
            # Skip noise points
            if label == -1:
                continue

            # Extract points belonging to this cluster
            cluster_mask = (labels == label)
            cluster_points = points[cluster_mask]

            # Compute centroid in Cartesian coordinates
            centroid_x = np.mean(cluster_points[:, 0])
            centroid_y = np.mean(cluster_points[:, 1])
            centroid_z = np.mean(cluster_points[:, 2])

            # Convert centroid to polar coordinates (range, bearing)
            # This is the hybrid approach: cluster in Cartesian, output in polar
            range_m = np.sqrt(centroid_x**2 + centroid_y**2)
            bearing_rad = np.arctan2(centroid_y, centroid_x)

            # Calculate cluster extents for validation
            # Radial extent (depth toward/away from boat)
            ranges = np.sqrt(cluster_points[:, 0]**2 + cluster_points[:, 1]**2)
            radial_extent = ranges.max() - ranges.min()

            # Lateral extent (bounding box diagonal)
            x_span = cluster_points[:, 0].max() - cluster_points[:, 0].min()
            y_span = cluster_points[:, 1].max() - cluster_points[:, 1].min()
            lateral_extent = np.sqrt(x_span**2 + y_span**2)

            # Validation filters (debug: log why a cluster is rejected)
            if lateral_extent > self.max_lateral_extent:
                self.get_logger().debug(f'Reject cluster at {range_m:.1f}m: lateral_extent {lateral_extent:.2f} > max {self.max_lateral_extent}')
                continue
            
            if lateral_extent < self.min_lateral_extent:
                self.get_logger().debug(f'Reject cluster at {range_m:.1f}m: lateral_extent {lateral_extent:.2f} < min {self.min_lateral_extent}')
                continue
            
            if len(cluster_points) < self.min_points_final:
                self.get_logger().debug(f'Reject cluster at {range_m:.1f}m: {len(cluster_points)} pts < min_points_final {self.min_points_final}')
                continue

            # Aspect ratio: reject elongated clusters (walls); only apply to larger clusters so small buoys (10–20 pts) are not rejected
            span_min = min(x_span, y_span)
            span_max = max(x_span, y_span)
            if span_min > 1e-6 and len(cluster_points) >= self.min_points_for_aspect_ratio:
                aspect_ratio = span_max / span_min
                if aspect_ratio > self.max_aspect_ratio:
                    self.get_logger().debug(f'Reject cluster at {range_m:.1f}m: aspect_ratio {aspect_ratio:.1f} > max {self.max_aspect_ratio}')
                    continue

            # Isolation and density: reject clusters that have other points very close or too many neighbors (wall)
            other_mask = ~cluster_mask
            other_points_xy = points[other_mask, :2]
            if len(other_points_xy) > 0:
                centroid_xy = np.array([[centroid_x, centroid_y]])
                dists = np.sqrt(((other_points_xy - centroid_xy) ** 2).sum(axis=1))
                min_dist_to_other = float(np.min(dists))
                if self.min_isolation_margin > 0.0 and min_dist_to_other < self.min_isolation_margin:
                    self.get_logger().debug(f'Reject cluster at {range_m:.1f}m: min_dist_to_other {min_dist_to_other:.2f} < isolation_margin {self.min_isolation_margin}')
                    continue
                # Isolated small clusters: accept small blobs only when no/few other points in radius (solid cluster, nothing connected)
                if len(cluster_points) <= self.small_cluster_max_points and self.small_cluster_isolation_radius > 0.0:
                    num_nearby = int(np.sum(dists < self.small_cluster_isolation_radius))
                    if num_nearby > self.small_cluster_max_nearby:
                        self.get_logger().debug(
                            f'Reject cluster at {range_m:.1f}m: small cluster ({len(cluster_points)} pts) not isolated '
                            f'({num_nearby} other pts within {self.small_cluster_isolation_radius}m > {self.small_cluster_max_nearby})'
                        )
                        continue
                if self.max_external_points_nearby >= 0 and self.external_density_radius > 0.0:
                    num_nearby = int(np.sum(dists < self.external_density_radius))
                    if num_nearby > self.max_external_points_nearby:
                        self.get_logger().debug(f'Reject cluster at {range_m:.1f}m: {num_nearby} nearby pts > max_external {self.max_external_points_nearby}')
                        continue

            # Compute confidence score (0.0 to 1.0)
            # More points = higher confidence, capped at 1.0
            confidence = min(1.0, len(cluster_points) / self.confidence_scale)

            # Package detection
            buoys.append({
                'range': range_m,
                'bearing': bearing_rad,
                'z_mean': centroid_z,
                'points': len(cluster_points),
                'lateral_extent_m': lateral_extent,
                'radial_extent_m': radial_extent,
                'confidence': confidence
            })

        return buoys

    def publish_detections(self, buoys: list, header) -> None:
        """Publish buoy detections as BuoyDetectionArray message."""
        msg = BuoyDetectionArray()
        msg.header = header
        msg.header.frame_id = header.frame_id  # Should be base_link or unilidar_lidar

        for buoy in buoys:
            detection = BuoyDetection()
            detection.range = float(buoy['range'])
            detection.bearing = float(buoy['bearing'])
            detection.z_mean = float(buoy['z_mean'])
            detection.num_points = int(buoy['points'])
            detection.lateral_extent = float(buoy['lateral_extent_m'])
            detection.radial_extent = float(buoy['radial_extent_m'])
            detection.confidence = float(buoy['confidence'])
            
            msg.detections.append(detection)

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BuoyDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
