"""
Multi-Camera Detection Combiner ROS2 Node - Per-Camera Bearing Approach

Subscribes to: /camera0/detection_info, /camera1/detection_info, /camera2/detection_info;
  /camera0/task4_detections, /camera1/task4_detections, /camera2/task4_detections
Publishes to: /combined/detection_info (String with all detections; bbox in local camera frame)

PER-CAMERA BEARING APPROACH:
- Each camera detection stays in its local frame (bbox is per-camera coordinates)
- Bearing is computed using:
  1. Camera mounting angle: [-70.0°, 0°, +70.0°] for [left, center, right]
  2. Camera intrinsics derived from 85° horizontal FOV and 1920×1200 resolution
  3. Formula: bearing_deg = mounting_angle + atan2((u - cx), fx)
- Output: boat-relative bearing for each detection (0° = forward, + = right, - = left)

WHAT THIS NODE DOES:
This node combines detections from all 3 side-by-side cameras into a single unified list.
Each detection includes:
- source camera_id (0=left, 1=center, 2=right)
- bbox in local camera frame
- bearing_deg/bearing_rad: angle relative to boat forward direction
- elevation_deg/elevation_rad: angle relative to horizon
- class_name with aspect ratio-based buoy classification (red_buoy vs red_pole_buoy)

HOW IT WORKS:
1. Subscribes to detection_info from all 3 cameras
2. Collects ALL detections from each active camera
3. Computes per-camera bearing using geometry (camera mounting + intrinsics)
4. Adds boat-relative bearing to each detection
5. Applies aspect ratio logic to distinguish red_buoy/red_pole_buoy and green_buoy/green_pole_buoy
6. Publishes combined list at ~30 Hz

OPERATION MODE:
- Latest Value Approach (default): Combines the most recent detection from each camera
- Staleness Filtering: Excludes cameras that haven't updated recently

Usage:
    # Default: Combines all 3 cameras, 1.0s staleness threshold
    python3 vision_combiner.py
    
    # Custom staleness threshold
    python3 vision_combiner.py --staleness_threshold 0.5
    
    # Timestamp-based sync (optional)
    python3 vision_combiner.py --use_timestamp_sync --sync_window 0.05
"""

import math
import os
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
import json
import yaml
from typing import Dict, List, Optional, Tuple
import time

from cv_ros_nodes.workspace_paths import default_class_mapping_path

# Camera specifications (Arducam B0495 2.3MP AR0234)
HORIZONTAL_FOV_PER_CAMERA_DEG = 85.0
VERTICAL_FOV_PER_CAMERA_DEG = 69.0
CAMERA_RESOLUTION_WIDTH = 1920
CAMERA_RESOLUTION_HEIGHT = 1200

# Camera mounting angles (boat frame: 0° = forward, + = right, - = left)
# Camera 0 = left, Camera 1 = center, Camera 2 = right
CAMERA_MOUNTING_ANGLES_DEG = [-70.0, 0.0, 70.0]

# Overlap-based duplicate detection: 20% of image width at each adjacent camera seam
OVERLAP_FRACTION = 0.2
OVERLAP_BBOX_AREA_MIN_RATIO = 0.4
OVERLAP_BBOX_AREA_MAX_RATIO = 2.5
ADJACENT_CAMERA_PAIRS: List[Tuple[int, int]] = [(0, 1), (1, 2)]


def _parse_camera_ids(camera_ids_str: str) -> List[int]:
    """Parse comma-separated camera IDs string to list of ints (e.g. '1' -> [1], '0,1,2' -> [0,1,2])."""
    if not camera_ids_str or not camera_ids_str.strip():
        return [0, 1, 2]
    return [int(x.strip()) for x in camera_ids_str.split(',') if x.strip().isdigit()]


class DetectionCombiner(Node):
    
    def __init__(self, sync_window: float = 0.05, use_timestamp_sync: bool = False,
                 staleness_threshold: float = 1.0, camera_ids: str = '0,1,2'):
        super().__init__('detection_combiner')
        
        self.camera_ids = _parse_camera_ids(camera_ids)
        if not self.camera_ids:
            self.camera_ids = [0, 1, 2]
        self.use_timestamp_sync = use_timestamp_sync
        self.sync_window = sync_window  # Time window for timestamp matching (seconds)
        self.staleness_threshold = staleness_threshold  # Max age for detections (seconds)
        
        # Per-camera frame dimensions: learned from detection_info (inference publishes preprocessed
        # image size, e.g. 1920×1200). Fallback until first message.
        self.frame_width = CAMERA_RESOLUTION_WIDTH
        self.frame_height = CAMERA_RESOLUTION_HEIGHT
        
        # Compute camera intrinsics from FOV and resolution
        self._compute_camera_intrinsics()

        # Store latest detections from each camera (inference)
        self.detections = {cid: None for cid in self.camera_ids}
        self.last_update_times = {cid: 0.0 for cid in self.camera_ids}
        # Task 4 supply-drop detections per camera (from /cameraN/task4_detections)
        self.task4_detections = {cid: None for cid in self.camera_ids}
        # Indicator buoy detections per camera (from /cameraN/indicator_detections)
        self.indicator_detections = {cid: None for cid in self.camera_ids}

        # Load class_mapping for class_name resolution
        self._class_id_to_name = self._load_class_mapping()

        # For timestamp-based synchronization: store detection history
        # Format: {timestamp: {camera_id: detection_data}}
        self.detection_history = []
        self.max_history_size = 100  # Keep last 100 detections per camera
        
        # Subscriptions for each camera's detection info (only for active camera_ids)
        self._detection_subs = []
        for camera_id in self.camera_ids:
            sub = self.create_subscription(
                String,
                f'/camera{camera_id}/detection_info',
                lambda msg, cid=camera_id: self.detection_callback(msg, cid),
                10
            )
            self._detection_subs.append(sub)
            self.get_logger().info(f'Subscribed to: /camera{camera_id}/detection_info')

        for camera_id in self.camera_ids:
            sub = self.create_subscription(
                String,
                f'/camera{camera_id}/task4_detections',
                lambda msg, cid=camera_id: self._task4_callback(msg, cid),
                10
            )
            self.get_logger().info(f'Subscribed to: /camera{camera_id}/task4_detections')

        for camera_id in self.camera_ids:
            sub = self.create_subscription(
                String,
                f'/camera{camera_id}/indicator_detections',
                lambda msg, cid=camera_id: self._indicator_callback(msg, cid),
                10
            )
            self.get_logger().info(f'Subscribed to: /camera{camera_id}/indicator_detections')

        # Publisher for combined detections
        self.combined_pub = self.create_publisher(
            String,
            '/combined/detection_info',
            10
        )
        
        self.get_logger().info('Detection combiner node initialized')
        self.get_logger().info('Publishing to: /combined/detection_info')
        self.get_logger().info(f'Camera intrinsics: fx={self.fx:.1f}, fy={self.fy:.1f}, cx={self.cx:.1f}, cy={self.cy:.1f}')
        self.get_logger().info(f'Camera mounting angles: {CAMERA_MOUNTING_ANGLES_DEG}')
        
        # Timer to periodically publish combined detections (10 Hz; matches ~5 fps camera pipeline to avoid CPU spin)
        self.timer = self.create_timer(0.1, self.publish_combined)

    def _load_class_mapping(self) -> Dict[int, str]:
        """Load class_id -> class_name from class_mapping.yaml. Returns dict; missing IDs get 'class_N'."""
        paths = []
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory('cv_ros_nodes')
            paths.append(os.path.join(pkg_share, 'class_mapping.yaml'))
        except Exception:
            pass
        paths.append(default_class_mapping_path())
        for path in paths:
            if os.path.isfile(path):
                try:
                    with open(path, 'r') as f:
                        data = yaml.safe_load(f)
                    classes = data.get('classes') or {}
                    return {int(k): str(v) for k, v in classes.items()}
                except Exception as e:
                    self.get_logger().warn(f'Failed to load class_mapping from {path}: {e}')
        self.get_logger().warn('Class mapping not found; detections will have class_name "class_N"')
        return {}

    def _trim_detection_for_output(self, det: Dict) -> Dict:
        """Return a copy of the detection with redundant/radian fields removed for combined output."""
        keys_to_drop = {'x1', 'y1', 'x2', 'y2', 'width', 'height', 'bearing_rad', 'elevation_rad'}
        return {k: v for k, v in det.items() if k not in keys_to_drop}

    def _resolve_class_name(self, det: Dict) -> str:
        """
        Resolve class_name for a detection with aspect ratio-based buoy classification.
        
        Task4 uses type; others use class_id. For red_buoy/green_buoy detections,
        applies aspect ratio logic to distinguish between regular buoys (square-ish)
        and pole buoys (vertical/tall):
        - Square-ish (width ≈ height): red_buoy, green_buoy (1 ft above waterline)
        - Tall/vertical (height >> width): red_pole_buoy, green_pole_buoy (39 in above waterline)
        
        This improves distance estimation accuracy by using correct reference dimensions.
        """
        if det.get('source') == 'task4' and det.get('type'):
            return det['type']  # yellow_supply_drop, black_supply_drop
            
        cid = det.get('class_id')
        if cid is None:
            return 'unknown'
            
        # Get initial class name from mapping
        initial_class_name = self._class_id_to_name.get(int(cid), f'class_{cid}')
        
        # Apply aspect ratio logic for red/green buoy classification (bidirectional)
        # Check both regular buoys and pole buoys to correct misclassification
        if initial_class_name in ['red_buoy', 'green_buoy', 'red_pole_buoy', 'green_pole_buoy']:
            bbox = det.get('bbox') or [det.get('x1', 0), det.get('y1', 0), det.get('x2', 0), det.get('y2', 0)]
            
            if len(bbox) >= 4:
                width = abs(bbox[2] - bbox[0])
                height = abs(bbox[3] - bbox[1])
                
                if width > 0 and height > 0:
                    aspect_ratio = width / height
                    
                    # Classification thresholds:
                    # - aspect_ratio < 0.7: tall/vertical (height >> width) → pole buoy (39 in)
                    # - aspect_ratio >= 0.7: square-ish or wider (width ≈ height or width > height) → regular buoy (1 ft)
                    
                    if aspect_ratio < 0.7:
                        # Tall/vertical → pole buoy
                        if initial_class_name == 'red_buoy':
                            return 'red_pole_buoy'
                        elif initial_class_name == 'green_buoy':
                            return 'green_pole_buoy'
                        # If already pole buoy, keep it
                    else:
                        # Square-ish or wider → regular buoy
                        if initial_class_name == 'red_pole_buoy':
                            return 'red_buoy'
                        elif initial_class_name == 'green_pole_buoy':
                            return 'green_buoy'
                        # If already regular buoy, keep it
        
        return initial_class_name

    def _compute_camera_intrinsics(self):
        """
        Compute camera intrinsics from FOV and resolution.
        Uses manufacturer specs: 85° horizontal FOV, 69° vertical FOV, 1920×1200 resolution.
        Formula: fx = width / (2 * tan(fov_h / 2))
        """
        # Compute focal lengths from FOV
        self.fx = self.frame_width / (2.0 * math.tan(math.radians(HORIZONTAL_FOV_PER_CAMERA_DEG / 2.0)))
        self.fy = self.frame_height / (2.0 * math.tan(math.radians(VERTICAL_FOV_PER_CAMERA_DEG / 2.0)))
        
        # Principal point at image center (standard assumption)
        self.cx = self.frame_width / 2.0
        self.cy = self.frame_height / 2.0
    
    def _compute_bearing_and_elevation(self, camera_id: int, bbox: List[float]) -> Dict[str, float]:
        """
        Compute boat-relative bearing and elevation from camera detection.
        
        Args:
            camera_id: 0=left, 1=center, 2=right
            bbox: [x1, y1, x2, y2] in camera frame
        
        Returns:
            Dict with bearing_deg, elevation_deg (and optional camera_angle_deg, mounting_angle_deg for debug).
            - bearing: 0° = boat forward, + = right, - = left
            - elevation: 0° = horizon, + = up, - = down
        """
        if len(bbox) != 4:
            return {
                'bearing_deg': 0.0,
                'elevation_deg': 0.0
            }
        
        # Bbox center in camera frame
        u_center = (bbox[0] + bbox[2]) / 2.0
        v_center = (bbox[1] + bbox[3]) / 2.0
        
        # Angle relative to camera optical axis (horizontal)
        camera_angle_rad = math.atan2(u_center - self.cx, self.fx)
        camera_angle_deg = math.degrees(camera_angle_rad)
        
        # Boat-relative bearing: add camera mounting angle
        mounting_angle_deg = CAMERA_MOUNTING_ANGLES_DEG[camera_id]
        bearing_deg = mounting_angle_deg + camera_angle_deg
        
        # Elevation angle (vertical): 0° = horizon, positive = up, negative = down
        elevation_rad = math.atan2(self.cy - v_center, self.fy)
        elevation_deg = math.degrees(elevation_rad)
        
        return {
            'bearing_deg': bearing_deg,
            'elevation_deg': elevation_deg,
            'camera_angle_deg': camera_angle_deg,
            'mounting_angle_deg': mounting_angle_deg
        }

    @staticmethod
    def _bbox_center_x(bbox: List[float]) -> float:
        """Return horizontal center of bbox [x1, y1, x2, y2]. Returns 0 if invalid."""
        if not bbox or len(bbox) < 4:
            return 0.0
        return (float(bbox[0]) + float(bbox[2])) / 2.0

    @staticmethod
    def _in_overlap_strip_toward(camera_id: int, bbox: List[float], neighbor_id: int, frame_width: int) -> bool:
        """
        True iff bbox center x lies in this camera's overlap strip toward neighbor_id.
        Camera 0: right 20% toward 1 (x >= 0.8*W). Camera 1: left 20% toward 0 (x <= 0.2*W), right 20% toward 2 (x >= 0.8*W). Camera 2: left 20% toward 1 (x <= 0.2*W).
        """
        if frame_width <= 0:
            return False
        cx = DetectionCombiner._bbox_center_x(bbox)
        if camera_id == 0 and neighbor_id == 1:
            return cx >= (1.0 - OVERLAP_FRACTION) * frame_width
        if camera_id == 1 and neighbor_id == 0:
            return cx <= OVERLAP_FRACTION * frame_width
        if camera_id == 1 and neighbor_id == 2:
            return cx >= (1.0 - OVERLAP_FRACTION) * frame_width
        if camera_id == 2 and neighbor_id == 1:
            return cx <= OVERLAP_FRACTION * frame_width
        return False

    @staticmethod
    def _bbox_area(bbox: List[float]) -> float:
        """Return area of bbox [x1, y1, x2, y2]. Returns 0 if invalid."""
        if not bbox or len(bbox) < 4:
            return 0.0
        w = abs(float(bbox[2]) - float(bbox[0]))
        h = abs(float(bbox[3]) - float(bbox[1]))
        return w * h

    @staticmethod
    def _bbox_sizes_similar(
        bbox_a: List[float],
        bbox_b: List[float],
        min_ratio: float = OVERLAP_BBOX_AREA_MIN_RATIO,
        max_ratio: float = OVERLAP_BBOX_AREA_MAX_RATIO,
    ) -> bool:
        """True if area ratio (smaller/larger) is in [min_ratio, max_ratio]. Handles zero area (returns False)."""
        area_a = DetectionCombiner._bbox_area(bbox_a)
        area_b = DetectionCombiner._bbox_area(bbox_b)
        if area_a <= 0 or area_b <= 0:
            return False
        ratio = min(area_a, area_b) / max(area_a, area_b)
        return min_ratio <= ratio <= max_ratio

    def _find_overlap_pairs(self, all_detections: List[Dict]) -> List[Tuple[int, int]]:
        """
        Find pairs of indices (i, j) with i < j that are overlap duplicates: adjacent cameras,
        both in overlap strip toward each other, same class_id, similar bbox size.
        Each detection appears in at most one pair (greedy by smallest bearing difference).
        """
        pairs: List[Tuple[int, int]] = []
        used = set()
        n = len(all_detections)
        fw = self.frame_width

        for (cam_a, cam_b) in ADJACENT_CAMERA_PAIRS:
            candidates_a = []
            candidates_b = []
            for idx in range(n):
                if idx in used:
                    continue
                d = all_detections[idx]
                if d.get('class_id') is None:
                    continue
                cid = d.get('camera_id')
                bbox = d.get('bbox')
                if not bbox or len(bbox) < 4:
                    continue
                if cid == cam_a and self._in_overlap_strip_toward(cam_a, bbox, cam_b, fw):
                    candidates_a.append(idx)
                elif cid == cam_b and self._in_overlap_strip_toward(cam_b, bbox, cam_a, fw):
                    candidates_b.append(idx)

            # Greedy: sort by bearing_deg, then for each candidate in A find best in B (same class_id, similar size, min bearing diff)
            def bearing_deg_key(idx: int) -> float:
                return float(all_detections[idx].get('bearing_deg', 0.0))

            candidates_a.sort(key=bearing_deg_key)
            candidates_b.sort(key=bearing_deg_key)

            for i in candidates_a:
                if i in used:
                    continue
                di = all_detections[i]
                class_i = di.get('class_id')
                bbox_i = di.get('bbox')
                bear_i = float(di.get('bearing_deg', 0.0))
                best_j: Optional[int] = None
                best_diff: float = 1e9
                for j in candidates_b:
                    if j in used:
                        continue
                    dj = all_detections[j]
                    if dj.get('class_id') != class_i:
                        continue
                    bbox_j = dj.get('bbox')
                    if not self._bbox_sizes_similar(bbox_i, bbox_j):
                        continue
                    diff = abs(bear_i - float(dj.get('bearing_deg', 0.0)))
                    if diff < best_diff:
                        best_diff = diff
                        best_j = j
                if best_j is not None:
                    pairs.append((min(i, best_j), max(i, best_j)))
                    used.add(i)
                    used.add(best_j)

        return pairs

    def _apply_overlap_merge(self, all_detections: List[Dict]) -> List[Dict]:
        """
        Merge overlap duplicate pairs into single detections with duplicate=true,
        both bboxes, and averaged bearing/elevation. Returns new list (unpaired + merged).
        """
        pairs = self._find_overlap_pairs(all_detections)
        used = set()
        for (i, j) in pairs:
            used.add(i)
            used.add(j)

        out: List[Dict] = []
        for idx, det in enumerate(all_detections):
            if idx in used:
                continue
            out.append(det.copy())

        for (i, j) in pairs:
            di = all_detections[i]
            dj = all_detections[j]
            cam_i = di.get('camera_id', 0)
            cam_j = dj.get('camera_id', 0)
            primary_idx, other_idx = (i, j) if cam_i <= cam_j else (j, i)
            d_primary = all_detections[primary_idx]
            d_other = all_detections[other_idx]
            merged = dict(d_primary)
            merged['duplicate'] = True
            merged['bbox'] = list(d_primary.get('bbox') or [])
            merged['bbox_other'] = list(d_other.get('bbox') or [])
            merged['camera_id_other'] = d_other.get('camera_id')
            b1 = float(d_primary.get('bearing_deg', 0.0))
            b2 = float(d_other.get('bearing_deg', 0.0))
            e1 = float(d_primary.get('elevation_deg', 0.0))
            e2 = float(d_other.get('elevation_deg', 0.0))
            merged['bearing_deg'] = (b1 + b2) / 2.0
            merged['elevation_deg'] = (e1 + e2) / 2.0
            out.append(merged)

        return out

    def detection_callback(self, msg: String, camera_id: int):
        """Callback for individual camera detection info"""
        try:
            detection_data = json.loads(msg.data)
            detection_timestamp = detection_data.get('timestamp', time.time())
            
            # Store latest detection (for non-sync mode)
            self.detections[camera_id] = detection_data
            self.last_update_times[camera_id] = time.time()
            
            # Learn frame dimensions from detection_info (bbox already in preprocessed frame)
            if 'frame_width' in detection_data or 'frame_height' in detection_data:
                new_width = int(detection_data.get('frame_width', self.frame_width))
                new_height = int(detection_data.get('frame_height', self.frame_height))
                
                # Recompute intrinsics if frame size changed
                if new_width != self.frame_width or new_height != self.frame_height:
                    self.frame_width = new_width
                    self.frame_height = new_height
                    self._compute_camera_intrinsics()
                    self.get_logger().info(
                        f'Updated frame size to {self.frame_width}×{self.frame_height}, '
                        f'recomputed intrinsics: fx={self.fx:.1f}, fy={self.fy:.1f}'
                    )
            
            # If using timestamp sync, try to match with other cameras
            if self.use_timestamp_sync:
                self._try_synchronize_and_publish(detection_timestamp, camera_id, detection_data)
            else:
                # Immediately publish when new data arrives
                self.publish_combined()
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse detection info from camera{camera_id}: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing detection from camera{camera_id}: {e}')

    def _task4_callback(self, msg: String, camera_id: int):
        """Store latest Task 4 supply-drop detections for a camera."""
        try:
            self.task4_detections[camera_id] = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse task4_detections from camera{camera_id}: {e}')

    def _indicator_callback(self, msg: String, camera_id: int):
        """Store latest indicator buoy detections for a camera."""
        try:
            self.indicator_detections[camera_id] = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse indicator_detections from camera{camera_id}: {e}')

    def _indicator_detections_to_combined(self, camera_id: int, indicator_data: dict) -> List[Dict]:
        """
        Convert indicator buoy payload to same format as other detections.
        Uses shape_bbox for bbox (whole buoy); class_id 9=red, 10=green.
        """
        out = []
        for det in indicator_data.get('detections', []):
            bbox = det.get('shape_bbox')
            if not bbox or len(bbox) != 4:
                continue
            det_with_camera = {
                'camera_id': camera_id,
                'class_id': det.get('class_id'),
                'score': det.get('score', 0.0),
                'bbox': list(bbox),
                'indicator_color': det.get('indicator_color'),
                'source': 'indicator_buoy',
            }
            out.append(det_with_camera)
        return out

    def _task4_detections_to_combined(self, camera_id: int, task4_data: dict) -> List[Dict]:
        """
        Convert Task4 payload detections to the same format as inference detections
        so they can be merged into all_detections. shape_bbox is already in preprocessed frame.
        """
        out = []
        for det in task4_data.get('detections', []):
            bbox = det.get('shape_bbox')
            if not bbox or len(bbox) != 4:
                continue
            det_with_camera = {
                'camera_id': camera_id,
                'class_id': det.get('class_id'),
                'score': det.get('score', 0.0),
                'bbox': list(bbox),
                'type': det.get('type'),
                'source': 'task4',
            }
            if det.get('vessel_bbox') is not None:
                det_with_camera['vessel_bbox'] = det['vessel_bbox']
            out.append(det_with_camera)
        return out

    def _try_synchronize_and_publish(self, timestamp: float, source_camera_id: int, detection_data: dict):
        """
        Attempt to synchronize detections from all cameras based on timestamp.
        If detections from all cameras are within sync_window, publish combined output.
        """
        # Check if we have recent detections from other cameras within sync window
        matched_detections = {source_camera_id: detection_data}
        matched_timestamps = {source_camera_id: timestamp}
        
        current_time = time.time()
        
        for camera_id in self.camera_ids:
            if camera_id == source_camera_id:
                continue
            
            detection = self.detections[camera_id]
            if detection is None:
                continue
            
            # Check if detection is stale
            time_since_update = current_time - self.last_update_times[camera_id]
            if time_since_update > self.staleness_threshold:
                continue
            
            other_timestamp = detection.get('timestamp', 0.0)
            time_diff = abs(timestamp - other_timestamp)
            
            # If timestamps are close enough, include in synchronized output
            if time_diff <= self.sync_window:
                matched_detections[camera_id] = detection
                matched_timestamps[camera_id] = other_timestamp
        
        # If we have at least 2 cameras synchronized, or if we have all 3, publish
        if len(matched_detections) >= 2:
            self._publish_synchronized(matched_detections, matched_timestamps)
        # Otherwise, if this is the first camera or others are too far off, 
        # we'll wait for timer-based publishing
    
    def _publish_synchronized(self, matched_detections: dict, matched_timestamps: dict):
        """Publish synchronized detections from matched cameras"""
        all_detections = []
        camera_stats = {}
        current_time = time.time()
        
        # Process matched detections (bbox already in preprocessed frame from inference)
        for camera_id, detection_data in matched_detections.items():
            detections = detection_data.get('detections', [])
            for det in detections:
                det_with_camera = det.copy()
                det_with_camera['camera_id'] = camera_id
                # Bbox is already in preprocessed frame (frame_width x frame_height) from inference
                bbox = det.get('bbox') or [det.get('x1', 0), det.get('y1', 0), det.get('x2', 0), det.get('y2', 0)]
                det_with_camera['bbox'] = list(bbox)
                all_detections.append(det_with_camera)
            
            # Merge Task4 detections for this camera when present
            task4_data = self.task4_detections.get(camera_id)
            if task4_data is not None:
                for det in self._task4_detections_to_combined(camera_id, task4_data):
                    all_detections.append(det)
            # Merge indicator buoy detections for this camera when present
            indicator_data = self.indicator_detections.get(camera_id)
            if indicator_data is not None:
                for det in self._indicator_detections_to_combined(camera_id, indicator_data):
                    all_detections.append(det)

            # Add stats for matched cameras (use output_fps = actual publish rate when available)
            camera_stats[camera_id] = {
                'status': 'active',
                'num_detections': len(detections),
                'fps': detection_data.get('output_fps', detection_data.get('fps', 0.0)),
                'timestamp': matched_timestamps[camera_id]
            }
        
        # Compute bearing and elevation for each detection
        for det in all_detections:
            bbox = det.get('bbox') or [det.get('x1', 0), det.get('y1', 0), det.get('x2', 0), det.get('y2', 0)]
            camera_id = det.get('camera_id', 0)
            bearing_info = self._compute_bearing_and_elevation(camera_id, bbox)
            det.update(bearing_info)
        # Add class_name for every detection
        for det in all_detections:
            det['class_name'] = self._resolve_class_name(det)
        # Overlap merge: merge duplicate detections in adjacent camera overlap strips
        all_detections = self._apply_overlap_merge(all_detections)
        # Trim output: drop x1,y1,x2,y2,width,height and radian fields
        all_detections = [self._trim_detection_for_output(d) for d in all_detections]

        # Add stats for unmatched cameras
        for camera_id in self.camera_ids:
            if camera_id not in matched_detections:
                time_since_update = current_time - self.last_update_times[camera_id]
                if self.detections[camera_id] is None:
                    camera_stats[camera_id] = {'status': 'no_data', 'num_detections': 0}
                elif time_since_update > self.staleness_threshold:
                    camera_stats[camera_id] = {
                        'status': 'stale',
                        'num_detections': 0,
                        'time_since_update': time_since_update
                    }
                else:
                    # Camera has data but timestamp doesn't match
                    camera_stats[camera_id] = {
                        'status': 'unsynchronized',
                        'num_detections': 0,
                        'time_since_update': time_since_update
                    }
        
        # Calculate average timestamp of matched detections
        avg_timestamp = sum(matched_timestamps.values()) / len(matched_timestamps) if matched_timestamps else current_time
        
        # Create combined detection info
        combined_info = {
            'timestamp': avg_timestamp,
            'num_cameras': len(self.camera_ids),
            'num_synchronized_cameras': len(matched_detections),
            'total_detections': len(all_detections),
            'camera_stats': camera_stats,
            'detections': all_detections,
            'synchronized': True,
            'camera_info': {
                'frame_width': self.frame_width,
                'frame_height': self.frame_height,
                'horizontal_fov_deg': HORIZONTAL_FOV_PER_CAMERA_DEG,
                'vertical_fov_deg': VERTICAL_FOV_PER_CAMERA_DEG,
                'mounting_angles_deg': CAMERA_MOUNTING_ANGLES_DEG,
                'fx': self.fx,
                'fy': self.fy,
                'cx': self.cx,
                'cy': self.cy
            }
        }
        
        # Publish combined detection info
        msg = String()
        msg.data = json.dumps(combined_info)
        self.combined_pub.publish(msg)
    
    def publish_combined(self):
        """
        Combine detections from all 3 cameras and publish.
        Uses latest value approach with staleness filtering.
        Only includes detections from cameras that have updated recently.
        
        This is the main function that combines all camera views:
        - Loops through all 3 cameras (0, 1, 2)
        - Collects ALL detections from each active camera
        - Adds camera_id to each detection
        - Publishes unified list
        """
        current_time = time.time()
        all_detections = []
        camera_stats = {}
        active_cameras = 0
        stale_cameras = 0
        no_data_cameras = 0
        
        # Collect detections from all 3 cameras (side-by-side configuration)
        for camera_id in self.camera_ids:
            detection_data = self.detections[camera_id]
            
            # Check if camera has no data at all
            if detection_data is None:
                no_data_cameras += 1
                camera_stats[camera_id] = {
                    'status': 'no_data',
                    'num_detections': 0,
                    'time_since_update': None
                }
                continue
            
            # Check if detection is stale (too old)
            time_since_update = current_time - self.last_update_times[camera_id]
            is_stale = time_since_update > self.staleness_threshold
            
            if is_stale:
                stale_cameras += 1
                camera_stats[camera_id] = {
                    'status': 'stale',
                    'num_detections': 0,
                    'time_since_update': round(time_since_update, 3),
                    'staleness_threshold': self.staleness_threshold,
                    'fps': detection_data.get('output_fps', detection_data.get('fps', 0.0)),
                    'last_timestamp': detection_data.get('timestamp', 0.0)
                }
                # Log stale warning at most once per second per camera (avoid flooding terminal)
                last_warn_key = f'_last_stale_warn_{camera_id}'
                last_warn_time = getattr(self, last_warn_key, 0.0)
                if current_time - last_warn_time >= 1.0:
                    self.get_logger().warn(
                        f'Camera{camera_id} is stale: {time_since_update:.3f}s since last update '
                        f'(threshold: {self.staleness_threshold}s) - excluding from output'
                    )
                    setattr(self, last_warn_key, current_time)
                continue
            
            # Camera is active and fresh - include ALL its detections (bbox already in preprocessed frame from inference)
            active_cameras += 1
            detections = detection_data.get('detections', [])
            for det in detections:
                det_with_camera = det.copy()
                det_with_camera['camera_id'] = camera_id  # Tag each detection with source camera
                # Bbox is already in preprocessed frame (frame_width x frame_height) from inference
                bbox = det.get('bbox') or [det.get('x1', 0), det.get('y1', 0), det.get('x2', 0), det.get('y2', 0)]
                det_with_camera['bbox'] = list(bbox)
                all_detections.append(det_with_camera)  # Add to combined list
            
            # Add stats for active camera (use output_fps = actual publish rate when available)
            camera_stats[camera_id] = {
                'status': 'active',
                'num_detections': len(detections),
                'fps': detection_data.get('output_fps', detection_data.get('fps', 0.0)),
                'timestamp': detection_data.get('timestamp', 0.0),
                'time_since_update': round(time_since_update, 3)
            }
        
        # Merge Task4 supply-drop detections when present
        for camera_id in self.camera_ids:
            task4_data = self.task4_detections[camera_id]
            if task4_data is not None:
                for det in self._task4_detections_to_combined(camera_id, task4_data):
                    all_detections.append(det)
        # Merge indicator buoy detections when present
        for camera_id in self.camera_ids:
            indicator_data = self.indicator_detections[camera_id]
            if indicator_data is not None:
                for det in self._indicator_detections_to_combined(camera_id, indicator_data):
                    all_detections.append(det)

        # Compute bearing and elevation for each detection
        for det in all_detections:
            bbox = det.get('bbox') or [det.get('x1', 0), det.get('y1', 0), det.get('x2', 0), det.get('y2', 0)]
            camera_id = det.get('camera_id', 0)
            bearing_info = self._compute_bearing_and_elevation(camera_id, bbox)
            det.update(bearing_info)
        # Add class_name for every detection
        for det in all_detections:
            det['class_name'] = self._resolve_class_name(det)
        # Overlap merge: merge duplicate detections in adjacent camera overlap strips
        all_detections = self._apply_overlap_merge(all_detections)
        # Trim output: drop x1,y1,x2,y2,width,height and radian fields
        all_detections = [self._trim_detection_for_output(d) for d in all_detections]

        # Create combined detection info
        combined_info = {
            'timestamp': current_time,
            'num_cameras': len(self.camera_ids),
            'num_active_cameras': active_cameras,
            'num_stale_cameras': stale_cameras,
            'num_no_data_cameras': no_data_cameras,
            'staleness_threshold': self.staleness_threshold,
            'total_detections': len(all_detections),
            'camera_stats': camera_stats,
            'detections': all_detections,
            'synchronized': self.use_timestamp_sync if self.use_timestamp_sync else None,
            'camera_info': {
                'frame_width': self.frame_width,
                'frame_height': self.frame_height,
                'horizontal_fov_deg': HORIZONTAL_FOV_PER_CAMERA_DEG,
                'vertical_fov_deg': VERTICAL_FOV_PER_CAMERA_DEG,
                'mounting_angles_deg': CAMERA_MOUNTING_ANGLES_DEG,
                'fx': self.fx,
                'fy': self.fy,
                'cx': self.cx,
                'cy': self.cy
            }
        }
        
        # Publish combined detection info
        msg = String()
        msg.data = json.dumps(combined_info)
        self.combined_pub.publish(msg)
        
        # Log health status periodically
        if hasattr(self, '_log_counter'):
            self._log_counter += 1
        else:
            self._log_counter = 0
        
        if self._log_counter % 30 == 0:  # Log every ~1 s
            if stale_cameras > 0 or no_data_cameras > 0:
                self.get_logger().warn(
                    f'Camera health: {active_cameras} active, {stale_cameras} stale, '
                    f'{no_data_cameras} no data | Total detections: {len(all_detections)}'
                )
            else:
                parts = [f'Camera{cid}: {camera_stats.get(cid, {}).get("num_detections", 0)}' for cid in self.camera_ids]
                self.get_logger().info(
                    f'Combined detections: {len(all_detections)} total | '
                    f'Active cameras: {active_cameras}/{len(self.camera_ids)} | '
                    + ', '.join(parts)
                )


def main(args=None):
    rclpy.init(args=args)
    
    import argparse
    parser = argparse.ArgumentParser(description='ROS2 Multi-Camera Detection Combiner')
    parser.add_argument('--staleness_threshold', type=float, default=1.0,
                       help='Maximum age (seconds) for detections before excluding camera (default: 1.0s)')
    parser.add_argument('--sync_window', type=float, default=0.05,
                       help='Time window (seconds) for timestamp-based synchronization (default: 0.05 = 50ms)')
    parser.add_argument('--use_timestamp_sync', action='store_true',
                       help='Enable timestamp-based synchronization (default: latest value approach)')
    parser.add_argument('--camera_ids', type=str, default='0,1,2',
                       help='Comma-separated camera IDs to combine (e.g. 1 for single-camera, 0,1,2 for three)')
    args_parsed, _ = parser.parse_known_args(args=args)
    
    node = DetectionCombiner(
        sync_window=args_parsed.sync_window,
        use_timestamp_sync=args_parsed.use_timestamp_sync,
        staleness_threshold=args_parsed.staleness_threshold,
        camera_ids=args_parsed.camera_ids
    )
    
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
