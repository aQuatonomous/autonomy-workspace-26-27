#!/usr/bin/env python3
"""
Vision-LiDAR fusion node.

Fuses **final CV output** (/combined/detection_info_with_distance) with LiDAR
(/tracked_buoys). For each CV detection, finds the **closest LiDAR track in that
direction** (by bearing only). Assigns class_id and class_name from CV to that track.
Distance from camera (distance_m) is present in the topic for future use but is NOT
used for matching.

Subscribes: /combined/detection_info_with_distance, /tracked_buoys
Publishes:  /fused_buoys (FusedBuoyArray)
"""

import json
import math
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from pointcloud_filters.msg import TrackedBuoyArray, TrackedBuoy, FusedBuoyArray, FusedBuoy


# class_id for unmatched LiDAR tracks (no CV detection in that direction)
CLASS_ID_UNKNOWN = 255


def _track_to_fused(track: TrackedBuoy, class_id: int, class_name: str) -> FusedBuoy:
    """Copy TrackedBuoy to FusedBuoy and set class_id, class_name."""
    fused = FusedBuoy()
    fused.id = track.id
    fused.range = track.range
    fused.bearing = track.bearing
    fused.x = track.x
    fused.y = track.y
    fused.z_mean = track.z_mean
    fused.class_id = class_id
    fused.class_name = class_name
    fused.confidence = track.confidence
    fused.observation_count = track.observation_count
    fused.first_seen = track.first_seen
    fused.last_seen = track.last_seen
    return fused


class VisionLidarFusionNode(Node):
    """
    CV-driven fusion: for each CV detection, find closest LiDAR track in that direction
    (by bearing only). Assign class_id and class_name from CV. Publish all LiDAR tracks
    as FusedBuoyArray. Distance from camera is not used for matching (available for later).
    """

    def __init__(self):
        super().__init__('vision_lidar_fusion')

        self.declare_parameter('bearing_threshold_rad', 0.15)
        self._bearing_threshold = float(
            self.get_parameter('bearing_threshold_rad').get_parameter_value().double_value
        )

        self._latest_cv_detections: List[Dict] = []
        self._latest_lidar: Optional[TrackedBuoyArray] = None

        self._cv_sub = self.create_subscription(
            String,
            '/combined/detection_info_with_distance',
            self._cv_callback,
            10,
        )
        self._lidar_sub = self.create_subscription(
            TrackedBuoyArray,
            '/tracked_buoys',
            self._lidar_callback,
            10,
        )
        self._fused_pub = self.create_publisher(
            FusedBuoyArray,
            '/fused_buoys',
            10,
        )

        self.get_logger().info(
            f'Vision-LiDAR fusion: bearing_threshold={self._bearing_threshold:.2f} rad. '
            'CV-driven: for each CV detection, closest LiDAR track in that direction gets class_id/class_name. '
            'Distance from camera not used for matching (available in topic for later).'
        )

    def _cv_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'CV: invalid JSON: {e}')
            return
        self._latest_cv_detections = data.get('detections', [])
        self._maybe_publish_fused()

    def _lidar_callback(self, msg: TrackedBuoyArray) -> None:
        self._latest_lidar = msg
        self._maybe_publish_fused()

    def _maybe_publish_fused(self) -> None:
        if self._latest_lidar is None:
            return
        self._run_fusion(self._latest_lidar)

    def _run_fusion(self, msg: TrackedBuoyArray) -> None:
        """
        CV-driven: for each CV detection, find the closest LiDAR track in that direction
        by bearing only. Assign that track the CV class_id and class_name.
        Unmatched tracks get class_id=CLASS_ID_UNKNOWN, class_name="unknown".
        """
        out = FusedBuoyArray()
        out.header = msg.header
        out.header.stamp = self.get_clock().now().to_msg()

        if len(msg.buoys) == 0:
            self._fused_pub.publish(out)
            return

        # Build CV candidates: (cv_bearing_rad, class_id, class_name, score)
        # CV bearing_deg: 0=forward, -left, +right. LiDAR: 0=+X, +left. So cv_bearing_rad = -rad(deg).
        cv_candidates: List[Tuple[float, int, str, float]] = []
        for d in self._latest_cv_detections:
            cid = d.get('class_id')
            if cid is None:
                continue
            try:
                cid = int(cid)
            except (TypeError, ValueError):
                continue
            class_name = d.get('class_name')
            if not class_name:
                class_name = f'class_{cid}'
            bearing_deg = d.get('bearing_deg')
            if bearing_deg is None:
                self.get_logger().warn(
                    f'CV detection skipped: missing bearing_deg (class_id={cid}, class_name={class_name})',
                    throttle_duration_sec=5.0
                )
                continue
            cv_bearing = -math.radians(float(bearing_deg))
            score = float(d.get('score', 0.0))
            cv_candidates.append((cv_bearing, cid, class_name, score))

        # For each CV detection, find closest LiDAR track in that direction (bearing only)
        # track_id -> (class_id, class_name, best_bearing_diff)
        assigned: Dict[int, Tuple[int, str, float]] = {}

        for cv_bearing, class_id, class_name, score in cv_candidates:
            best_track_id = None
            best_bearing_diff = self._bearing_threshold + 1.0

            for track in msg.buoys:
                bearing_diff = abs(cv_bearing - track.bearing)
                if bearing_diff > self._bearing_threshold:
                    continue
                if bearing_diff < best_bearing_diff:
                    best_bearing_diff = bearing_diff
                    best_track_id = track.id

            if best_track_id is not None:
                if best_track_id not in assigned or best_bearing_diff < assigned[best_track_id][2]:
                    assigned[best_track_id] = (class_id, class_name, best_bearing_diff)

        # Build output: every LiDAR track with assigned class or unknown
        for track in msg.buoys:
            if track.id in assigned:
                cid, cname, _ = assigned[track.id]
                out.buoys.append(_track_to_fused(track, cid, cname))
            else:
                out.buoys.append(_track_to_fused(track, CLASS_ID_UNKNOWN, 'unknown'))

        self._fused_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = VisionLidarFusionNode()
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
