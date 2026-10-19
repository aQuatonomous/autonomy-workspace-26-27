#!/usr/bin/env python3

import math
from typing import Optional
from collections import deque
from threading import Lock

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.duration import Duration

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
import tf2_ros

from sensor_msgs_py import point_cloud2 as pc2

from pointcloud_filters.srv import GetCloudWindow


def quaternion_to_rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    # Normalize to be safe
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm == 0.0:
        return np.eye(3)
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    # Quaternion to rotation matrix
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    R = np.array([
        [1 - 2 * (yy + zz),     2 * (xy - wz),         2 * (xz + wy)],
        [2 * (xy + wz),         1 - 2 * (xx + zz),     2 * (yz - wx)],
        [2 * (xz - wy),         2 * (yz + wx),         1 - 2 * (xx + yy)],
    ], dtype=np.float64)
    return R


class LidarRangeFilterNode(Node):
    def __init__(self) -> None:
        super().__init__('lidar_range_filter')

        # Parameters
        # Default input topic from unilidar_sdk
        self.declare_parameter('input_topic', '/unilidar/cloud')
        self.declare_parameter('output_topic', '/points_filtered')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('use_tf_transform', False)
        self.declare_parameter('z_min', -0.37)
        self.declare_parameter('z_max', 0.5)
        self.declare_parameter('range_max', 0.5)
        self.declare_parameter('max_buffer_duration_sec', 5.0)
        self.declare_parameter('keep_publisher', True)
        self.declare_parameter('enable_accumulation', False)
        self.declare_parameter('accumulation_window', 0.2)
        # Clockwise rotation in degrees around Z (0 = no rotation). Applied to all points after reading, before other filters.
        self.declare_parameter('rotate_cw_deg', 202.5)
        # Counter-clockwise rotation in degrees around Y (0 = no rotation). Applied after Z and X.
        self.declare_parameter('rotate_ccw_y_deg', 0.0)
        # Clockwise rotation in degrees around positive X (0 = no rotation). Applied after Z, before Y.
        self.declare_parameter('rotate_cw_x_deg', 30.0)
        # Counter-clockwise rotation in degrees around Z (0 = no rotation). Applied after X rotation.
        self.declare_parameter('rotate_ccw_z2_deg', 90.0)
        # Field of view limit: max angle from +X axis to keep (degrees). 105° = 210° total coverage centered on +X. 0 = no FOV filter.
        self.declare_parameter('fov_max_angle_from_x', 105.0)
        # Z ceiling: remove every point with z above this (m). E.g. 1.5 = cut off everything above 1.5 m. Set very large (e.g. 1000) to disable.
        self.declare_parameter('z_max_cutoff', 1.5)

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.use_tf_transform = self.get_parameter('use_tf_transform').get_parameter_value().bool_value
        self.z_min = float(self.get_parameter('z_min').get_parameter_value().double_value)
        self.z_max = float(self.get_parameter('z_max').get_parameter_value().double_value)
        self.range_max = float(self.get_parameter('range_max').get_parameter_value().double_value)
        self.max_buffer_duration_sec = float(self.get_parameter('max_buffer_duration_sec').get_parameter_value().double_value)
        self.keep_publisher = self.get_parameter('keep_publisher').get_parameter_value().bool_value
        self.enable_accumulation = self.get_parameter('enable_accumulation').get_parameter_value().bool_value
        self.accumulation_window = float(self.get_parameter('accumulation_window').get_parameter_value().double_value)
        self.rotate_cw_deg = float(self.get_parameter('rotate_cw_deg').get_parameter_value().double_value)
        self.rotate_ccw_y_deg = float(self.get_parameter('rotate_ccw_y_deg').get_parameter_value().double_value)
        self.rotate_cw_x_deg = float(self.get_parameter('rotate_cw_x_deg').get_parameter_value().double_value)
        self.rotate_ccw_z2_deg = float(self.get_parameter('rotate_ccw_z2_deg').get_parameter_value().double_value)
        self.fov_max_angle_from_x = float(self.get_parameter('fov_max_angle_from_x').get_parameter_value().double_value)
        self.z_max_cutoff = float(self.get_parameter('z_max_cutoff').get_parameter_value().double_value)
        # Precompute rotation around Z (clockwise = negative angle around Z)
        self._rot_z_rad = math.radians(-self.rotate_cw_deg)
        self._cos_z = math.cos(self._rot_z_rad)
        self._sin_z = math.sin(self._rot_z_rad)
        # Precompute rotation around X (clockwise = negative angle): y'=cos*y-sin*z, z'=sin*y+cos*z
        self._rot_x_rad = math.radians(-self.rotate_cw_x_deg)
        self._cos_x = math.cos(self._rot_x_rad)
        self._sin_x = math.sin(self._rot_x_rad)
        # Precompute second rotation around Z (counter-clockwise = positive angle around Z)
        self._rot_z2_rad = math.radians(self.rotate_ccw_z2_deg)
        self._cos_z2 = math.cos(self._rot_z2_rad)
        self._sin_z2 = math.sin(self._rot_z2_rad)
        # Precompute rotation around Y (counter-clockwise = positive angle around Y)
        self._rot_y_rad = math.radians(self.rotate_ccw_y_deg)
        self._cos_y = math.cos(self._rot_y_rad)
        self._sin_y = math.sin(self._rot_y_rad)

        # Time-ordered buffer for filtered clouds (stores up to max_buffer_duration_sec)
        self.cloud_buffer = deque()
        self.buffer_lock = Lock()

        # TF buffer/listener for transforms to base_frame
        self.tf_buffer: Optional[tf2_ros.Buffer] = None
        self.tf_listener: Optional[tf2_ros.TransformListener] = None
        if self.use_tf_transform:
            self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriber: BEST_EFFORT to match Unitree driver (sensor data)
        qos_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub = self.create_subscription(PointCloud2, self.input_topic, self.cloud_cb, qos_sub)

        # Publisher: RELIABLE for downstream nodes
        qos_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        if self.keep_publisher:
            self.pub = self.create_publisher(PointCloud2, self.output_topic, qos_pub)
        else:
            self.pub = None

        # Service for retrieving windowed point cloud data
        self.srv = self.create_service(GetCloudWindow, '~/get_cloud_window', self.handle_get_cloud_window)

        self.get_logger().info(
            f'lidar_range_filter started: input={self.input_topic}, output={self.output_topic if self.keep_publisher else "(disabled)"}, '
            f'z=[{self.z_min},{self.z_max}] m, range_max={self.range_max} m, base_frame={self.base_frame}, '
            f'use_tf_transform={self.use_tf_transform}, rotate_cw_deg={self.rotate_cw_deg}, rotate_cw_x_deg={self.rotate_cw_x_deg}, rotate_ccw_z2_deg={self.rotate_ccw_z2_deg}, rotate_ccw_y_deg={self.rotate_ccw_y_deg}, '
            f'fov_max_angle_from_x={self.fov_max_angle_from_x}°, z_max_cutoff={self.z_max_cutoff} m, '
            f'max_buffer_duration_sec={self.max_buffer_duration_sec}, '
            f'accumulation: {"enabled (" + str(self.accumulation_window) + "s)" if self.enable_accumulation else "disabled"}'
        )

    def lookup_transform(self, target_frame: str, source_frame: str, stamp) -> Optional[TransformStamped]:
        if not self.use_tf_transform or self.tf_buffer is None:
            return None
        try:
            # Use time from message; allow small timeout
            return self.tf_buffer.lookup_transform(target_frame, source_frame, stamp, timeout=Duration(seconds=0.2))
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed {source_frame} -> {target_frame} at time {stamp.sec}.{stamp.nanosec}: {e}')
            return None

    def cloud_cb(self, msg: PointCloud2) -> None:
        # Determine fields and read points (keep intensity if available)
        field_map = {f.name: f for f in msg.fields}
        keep_intensity = 'intensity' in field_map
        read_fields = ('x', 'y', 'z', 'intensity') if keep_intensity else ('x', 'y', 'z')

        # Always use read_points() to support mixed datatypes and padding in PointCloud2
        try:
            pts_iter = pc2.read_points(msg, field_names=read_fields, skip_nans=True)
        except Exception as e:
            self.get_logger().error(f'Failed to read points from PointCloud2: {e}')
            return

        # Build a list of numeric tuples robustly (supports structured records and sequences)
        values = []
        try:
            for p in pts_iter:
                # Structured numpy scalar (np.void) with named fields
                if isinstance(p, np.void) and getattr(p, 'dtype', None) is not None and p.dtype.fields is not None:
                    if keep_intensity:
                        values.append((float(p['x']), float(p['y']), float(p['z']), float(p['intensity'])))
                    else:
                        values.append((float(p['x']), float(p['y']), float(p['z'])))
                else:
                    # Assume indexable sequence/tuple
                    if keep_intensity:
                        values.append((float(p[0]), float(p[1]), float(p[2]), float(p[3])))
                    else:
                        values.append((float(p[0]), float(p[1]), float(p[2])))
        except Exception as e:
            self.get_logger().error(f'Error parsing PointCloud2 fields: {e}')
            return

        if len(values) == 0:
            return

        # Convert to ndarray (float64 for math stability)
        pts = np.array(values, dtype=np.float64)

        if pts.size == 0:
            return

        x = pts[:, 0].astype(np.float64, copy=False)
        y = pts[:, 1].astype(np.float64, copy=False)
        z = pts[:, 2].astype(np.float64, copy=False)
        if keep_intensity:
            intens = pts[:, 3].astype(np.float64, copy=False)

        # Apply rotations: first Z (clockwise), then Y (counter-clockwise)
        # Rotation 1: Clockwise around Z by rotate_cw_deg (e.g. 202.5°)
        if self.rotate_cw_deg != 0.0:
            x_new = self._cos_z * x - self._sin_z * y
            y_new = self._sin_z * x + self._cos_z * y
            x = x_new
            y = y_new

        # Rotation 2: Clockwise around positive X by rotate_cw_x_deg (e.g. 30°)
        # Rx(θ): x'=x, y'=cos(θ)*y - sin(θ)*z, z'=sin(θ)*y + cos(θ)*z (θ negative for CW)
        if self.rotate_cw_x_deg != 0.0:
            y_new = self._cos_x * y - self._sin_x * z
            z_new = self._sin_x * y + self._cos_x * z
            y = y_new
            z = z_new

        # Rotation 3: Counter-clockwise around Z by rotate_ccw_z2_deg (e.g. 90°)
        # Rz(θ): x'=cos(θ)*x - sin(θ)*y, y'=sin(θ)*x + cos(θ)*y, z'=z (θ positive for CCW)
        if self.rotate_ccw_z2_deg != 0.0:
            x_new = self._cos_z2 * x - self._sin_z2 * y
            y_new = self._sin_z2 * x + self._cos_z2 * y
            x = x_new
            y = y_new

        # Rotation 4: Counter-clockwise around Y by rotate_ccw_y_deg
        # Rotation around Y: [x', y', z'] = [cos(θ)*x + sin(θ)*z, y, -sin(θ)*x + cos(θ)*z]
        if self.rotate_ccw_y_deg != 0.0:
            x_new = self._cos_y * x + self._sin_y * z
            z_new = -self._sin_y * x + self._cos_y * z
            x = x_new
            z = z_new

        # Field of view filter: keep only points within fov_max_angle_from_x degrees from +X axis
        # This creates a cone of 2*fov_max_angle_from_x degrees centered on +X (e.g., 115° = 230° total)
        if self.fov_max_angle_from_x > 0.0 and self.fov_max_angle_from_x < 180.0:
            # Calculate angle from +X axis in 3D: angle = atan2(sqrt(y² + z²), x)
            perpendicular_dist = np.sqrt(y * y + z * z)
            angle_from_x_rad = np.arctan2(perpendicular_dist, x)
            angle_from_x_deg = np.degrees(angle_from_x_rad)
            
            fov_mask = angle_from_x_deg <= self.fov_max_angle_from_x
            # Apply FOV mask (keep only points within cone in front of lidar)
            x = x[fov_mask]
            y = y[fov_mask]
            z = z[fov_mask]
            if keep_intensity:
                intens = intens[fov_mask]
            
            # If no points remain after FOV filter, publish empty cloud
            if len(x) == 0:
                header = msg.header
                header.frame_id = self.base_frame
                out_msg = pc2.create_cloud_xyz32(header, [])
                if self.pub:
                    self.pub.publish(out_msg)
                with self.buffer_lock:
                    self._append_to_buffer(out_msg)
                return

        # Z ceiling: remove every point with z above z_max_cutoff (e.g. 2 m)
        if self.z_max_cutoff < 1e6:
            z_ceiling_mask = z <= self.z_max_cutoff
            x = x[z_ceiling_mask]
            y = y[z_ceiling_mask]
            z = z[z_ceiling_mask]
            if keep_intensity:
                intens = intens[z_ceiling_mask]
            if len(x) == 0:
                header = msg.header
                header.frame_id = self.base_frame
                out_msg = pc2.create_cloud_xyz32(header, [])
                if self.pub:
                    self.pub.publish(out_msg)
                with self.buffer_lock:
                    self._append_to_buffer(out_msg)
                return

        # Transform to base_frame if requested and frames differ
        src_frame = msg.header.frame_id
        transformed = False
        if self.use_tf_transform and src_frame and src_frame != self.base_frame:
            tf = self.lookup_transform(self.base_frame, src_frame, msg.header.stamp)
            if tf is not None:
                t = tf.transform.translation
                q = tf.transform.rotation
                R = quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)
                T = np.array([t.x, t.y, t.z], dtype=np.float64)
                pts_xyz = np.vstack((x, y, z))  # 3xN (do not overwrite pts; keep_intensity needs pts)
                pts_tf = (R @ pts_xyz) + T.reshape(3, 1)
                x, y, z = pts_tf[0, :], pts_tf[1, :], pts_tf[2, :]
                transformed = True
            else:
                # Proceed without transform; assume already in base_frame
                pass

        # Compute range in the horizontal plane and apply filters
        rng = np.sqrt(x * x + y * y)
        mask = (rng <= self.range_max) & (z >= self.z_min) & (z <= self.z_max)
        header = msg.header
        header.frame_id = self.base_frame

        if not np.any(mask):
            # Publish empty XYZ cloud
            out_msg = pc2.create_cloud_xyz32(header, [])
            if self.pub:
                self.pub.publish(out_msg)
            # Store empty cloud in buffer
            with self.buffer_lock:
                self._append_to_buffer(out_msg)
            return

        if keep_intensity:
            # Keep intensity; use the already-sliced intens variable (not pts[:,3] which is original unfiltered array)
            sel = mask
            points_list = list(map(lambda i: (float(x[i]), float(y[i]), float(z[i]), float(intens[i])), np.flatnonzero(sel)))
            from sensor_msgs.msg import PointField
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            out_msg = pc2.create_cloud(header, fields, points_list)
        else:
            # XYZ only
            sel_pts = np.vstack((x[mask], y[mask], z[mask])).T.astype(np.float32)
            out_msg = pc2.create_cloud_xyz32(header, sel_pts.tolist())

        # Store filtered cloud in buffer first
        with self.buffer_lock:
            self._append_to_buffer(out_msg)
            
            # Publish accumulated or single frame based on settings
            if self.pub:
                if self.enable_accumulation:
                    # Get clouds within accumulation window and merge them
                    accumulated_cloud = self._get_accumulated_cloud()
                    self.pub.publish(accumulated_cloud)
                else:
                    # Publish just the current frame
                    self.pub.publish(out_msg)

    def _append_to_buffer(self, cloud: PointCloud2) -> None:
        """Append cloud to buffer and evict old entries beyond max_buffer_duration_sec."""
        self.cloud_buffer.append(cloud)
        # Evict old clouds
        if len(self.cloud_buffer) > 1:
            newest_stamp = self.cloud_buffer[-1].header.stamp
            newest_time_sec = newest_stamp.sec + newest_stamp.nanosec * 1e-9
            while len(self.cloud_buffer) > 1:
                oldest_stamp = self.cloud_buffer[0].header.stamp
                oldest_time_sec = oldest_stamp.sec + oldest_stamp.nanosec * 1e-9
                if (newest_time_sec - oldest_time_sec) > self.max_buffer_duration_sec:
                    self.cloud_buffer.popleft()
                else:
                    break
    
    def _get_accumulated_cloud(self) -> PointCloud2:
        """Get merged cloud from points within accumulation_window. Must be called with buffer_lock held."""
        if len(self.cloud_buffer) == 0:
            return PointCloud2()
        
        if len(self.cloud_buffer) == 1:
            return self.cloud_buffer[0]
        
        # Get newest timestamp and calculate cutoff
        newest_stamp = self.cloud_buffer[-1].header.stamp
        newest_time_sec = newest_stamp.sec + newest_stamp.nanosec * 1e-9
        cutoff_time_sec = newest_time_sec - self.accumulation_window
        
        # Select clouds within accumulation window
        selected = []
        for cloud in self.cloud_buffer:
            stamp = cloud.header.stamp
            time_sec = stamp.sec + stamp.nanosec * 1e-9
            if time_sec >= cutoff_time_sec:
                selected.append(cloud)
        
        if len(selected) == 0:
            return PointCloud2()
        
        # Merge and return
        return self._merge_clouds(selected)

    def handle_get_cloud_window(self, request, response):
        """Service handler: return windowed point clouds."""
        window_sec = min(request.window_sec, self.max_buffer_duration_sec)
        
        with self.buffer_lock:
            if len(self.cloud_buffer) == 0:
                # Empty buffer
                response.clouds = []
                response.merged_cloud = PointCloud2()
                return response

            # Get newest timestamp
            newest_stamp = self.cloud_buffer[-1].header.stamp
            newest_time_sec = newest_stamp.sec + newest_stamp.nanosec * 1e-9
            cutoff_time_sec = newest_time_sec - window_sec

            # Select clouds within window
            selected = []
            for cloud in self.cloud_buffer:
                stamp = cloud.header.stamp
                time_sec = stamp.sec + stamp.nanosec * 1e-9
                if time_sec >= cutoff_time_sec:
                    selected.append(cloud)

            if len(selected) == 0:
                response.clouds = []
                response.merged_cloud = PointCloud2()
                return response

            if request.merged:
                # Merge all selected clouds into one
                response.merged_cloud = self._merge_clouds(selected)
                response.clouds = []
            else:
                # Return array of individual clouds
                response.clouds = selected
                response.merged_cloud = PointCloud2()

        return response

    def _merge_clouds(self, clouds):
        """Concatenate multiple PointCloud2 messages into a single cloud."""
        if len(clouds) == 0:
            return PointCloud2()
        if len(clouds) == 1:
            return clouds[0]

        # Use the header from the newest cloud
        merged_header = clouds[-1].header

        # Determine if we have intensity field by checking first cloud
        first_cloud_fields = [f.name for f in clouds[0].fields]
        has_intensity = 'intensity' in first_cloud_fields
        
        # Read all points from all clouds, ensuring consistent field structure
        all_points = []
        for cloud in clouds:
            cloud_fields = [f.name for f in cloud.fields]
            cloud_has_intensity = 'intensity' in cloud_fields
            
            # Read with appropriate fields
            if has_intensity and cloud_has_intensity:
                field_names = ('x', 'y', 'z', 'intensity')
            else:
                field_names = ('x', 'y', 'z')
            
            try:
                pts_iter = pc2.read_points(cloud, field_names=field_names, skip_nans=True)
                for p in pts_iter:
                    if isinstance(p, np.void) and getattr(p, 'dtype', None) is not None and p.dtype.fields is not None:
                        if has_intensity and len(field_names) == 4:
                            all_points.append((float(p['x']), float(p['y']), float(p['z']), float(p['intensity'])))
                        else:
                            all_points.append((float(p['x']), float(p['y']), float(p['z'])))
                    else:
                        # Tuple/sequence format
                        if has_intensity and len(p) >= 4:
                            all_points.append((float(p[0]), float(p[1]), float(p[2]), float(p[3])))
                        else:
                            all_points.append((float(p[0]), float(p[1]), float(p[2])))
            except Exception as e:
                self.get_logger().warn(f'Failed to read points during merge: {e}')
                continue

        if len(all_points) == 0:
            return PointCloud2()

        # Reconstruct PointCloud2 with merged points using consistent field structure
        if has_intensity:
            from sensor_msgs.msg import PointField
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            merged_cloud = pc2.create_cloud(merged_header, fields, all_points)
        else:
            # XYZ only
            merged_cloud = pc2.create_cloud_xyz32(merged_header, all_points)

        return merged_cloud


def main(args=None):
    rclpy.init(args=args)
    node = LidarRangeFilterNode()
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
