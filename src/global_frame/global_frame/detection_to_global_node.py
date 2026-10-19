#!/usr/bin/env python3
"""
Detection-to-global node: boat pose + detections -> global (east, north) in map frame.

Subscribes to (one of two, based on use_fused_detections):
  - /boat_pose (global_frame/BoatPose)
  - /fused_buoys (FusedBuoyArray)  [when use_fused_detections=true] - LiDAR tracks with CV class
  - /tracked_buoys_json (std_msgs/String)  [when use_fused_detections=false] - raw LiDAR only

Publishes:
  - /global_detections (global_frame/GlobalDetectionArray)

Source at pipeline end:
  - use_fused_detections=true (default): feed planning from /fused_buoys (LiDAR+CV fusion), source="fused"
  - use_fused_detections=false: feed planning from /tracked_buoys_json (LiDAR only), source="lidar"

CV-LiDAR fusion runs independently; this toggle only selects which stream feeds /global_detections.
Heading convention: 0 = East, positive = CCW. Boat-relative bearing: 0 = boat forward (+X).
"""

import json
import math
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from builtin_interfaces.msg import Time

from global_frame.msg import BoatPose, GlobalDetection, GlobalDetectionArray
from pointcloud_filters.msg import FusedBuoyArray


def _normalize_angle(angle_rad: float) -> float:
    """Normalize angle to [-π, π]."""
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


class DetectionToGlobalNode(Node):
    def __init__(self):
        super().__init__("detection_to_global_node")

        self.declare_parameter("boat_pose_topic", "/boat_pose")
        self.declare_parameter("fused_buoys_topic", "/fused_buoys")
        self.declare_parameter("tracked_buoys_topic", "/tracked_buoys_json")
        self.declare_parameter("global_detections_topic", "/global_detections")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("use_fused_detections", True)  # default: fused (LiDAR+CV) feeds planning
        self.declare_parameter("publish_unknown_detections", False)  # if False, do not include class_id 255 / unknown in /global_detections

        self._boat_pose_topic = self.get_parameter("boat_pose_topic").value
        self._fused_buoys_topic = self.get_parameter("fused_buoys_topic").value
        self._tracked_buoys_topic = self.get_parameter("tracked_buoys_topic").value
        self._global_detections_topic = self.get_parameter("global_detections_topic").value
        self._map_frame = self.get_parameter("map_frame").value
        _use_fused = self.get_parameter("use_fused_detections").value
        self._use_fused_detections = bool(_use_fused) if isinstance(_use_fused, bool) else (str(_use_fused).lower() == "true")
        _pub_unknown = self.get_parameter("publish_unknown_detections").value
        self._publish_unknown_detections = bool(_pub_unknown) if isinstance(_pub_unknown, bool) else (str(_pub_unknown).lower() == "true")

        self._boat_east: Optional[float] = None
        self._boat_north: Optional[float] = None
        self._boat_heading_rad: Optional[float] = None
        self._boat_stamp: Optional[Time] = None

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._boat_pose_sub = self.create_subscription(
            BoatPose,
            self._boat_pose_topic,
            self._boat_pose_callback,
            qos,
        )
        if self._use_fused_detections:
            self._fused_sub = self.create_subscription(
                FusedBuoyArray,
                self._fused_buoys_topic,
                self._fused_callback,
                qos,
            )
            self._lidar_sub = None
        else:
            self._fused_sub = None
            self._lidar_sub = self.create_subscription(
                String,
                self._tracked_buoys_topic,
                self._lidar_callback,
                qos,
            )
        self._global_pub = self.create_publisher(
            GlobalDetectionArray,
            self._global_detections_topic,
            qos,
        )

        mode = "fused (/fused_buoys)" if self._use_fused_detections else "lidar (/tracked_buoys_json)"
        self.get_logger().info(
            f"detection_to_global: pose={self._boat_pose_topic}, source={mode} -> {self._global_detections_topic}"
        )

    def _boat_pose_callback(self, msg: BoatPose) -> None:
        self._boat_east = msg.east
        self._boat_north = msg.north
        self._boat_heading_rad = msg.heading_rad
        self._boat_stamp = msg.header.stamp

    def _to_global(self, range_m: float, bearing_rad: float) -> tuple[float, float]:
        """Convert boat-relative (range, bearing) to global (east, north)."""
        if self._boat_east is None or self._boat_north is None or self._boat_heading_rad is None:
            return (0.0, 0.0)
        alpha = _normalize_angle(self._boat_heading_rad + bearing_rad)
        east = self._boat_east + range_m * math.cos(alpha)
        north = self._boat_north + range_m * math.sin(alpha)
        return (east, north)

    def _fused_callback(self, msg: FusedBuoyArray) -> None:
        """Convert /fused_buoys to GlobalDetectionArray, source=fused."""
        if self._boat_east is None:
            return
        stamp = self._boat_stamp or self.get_clock().now().to_msg()
        detections = []
        for b in msg.buoys:
            range_m = float(b.range)
            bearing_rad = float(b.bearing)
            east, north = self._to_global(range_m, bearing_rad)
            alpha = _normalize_angle((self._boat_heading_rad or 0.0) + bearing_rad)
            g = GlobalDetection()
            g.header.stamp = stamp
            g.header.frame_id = self._map_frame
            g.east = east
            g.north = north
            g.range_m = range_m
            g.bearing_global_rad = alpha
            g.source = "fused"
            g.class_name = b.class_name or "unknown"
            cid = int(b.class_id)
            g.class_id = cid if 0 <= cid <= 22 else 255
            g.id = b.id
            if not self._publish_unknown_detections and (g.class_id == 255 or (g.class_name or "").strip().lower() == "unknown"):
                continue
            detections.append(g)
        if detections:
            out = GlobalDetectionArray()
            out.header.stamp = stamp
            out.header.frame_id = self._map_frame
            out.detections = detections
            self._global_pub.publish(out)

    def _lidar_callback(self, msg: String) -> None:
        """Convert /tracked_buoys_json to GlobalDetectionArray, source=lidar."""
        if self._boat_east is None:
            return
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        buoys = data.get("detections", [])
        stamp = self._boat_stamp or self.get_clock().now().to_msg()
        detections = []
        for d in buoys:
            range_m = float(d.get("range", 0.0))
            bearing_rad = float(d.get("bearing", 0.0))
            east, north = self._to_global(range_m, bearing_rad)
            alpha = _normalize_angle((self._boat_heading_rad or 0.0) + bearing_rad)
            g = GlobalDetection()
            g.header.stamp = stamp
            g.header.frame_id = self._map_frame
            g.east = east
            g.north = north
            g.range_m = range_m
            g.bearing_global_rad = alpha
            g.source = "lidar"
            g.class_name = "unknown"
            g.class_id = 255
            g.id = int(d.get("id", 0))
            if not self._publish_unknown_detections:
                continue  # lidar-only detections are all unknown; skip when param is off
            detections.append(g)
        if detections:
            out = GlobalDetectionArray()
            out.header.stamp = stamp
            out.header.frame_id = self._map_frame
            out.detections = detections
            self._global_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionToGlobalNode()
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


if __name__ == "__main__":
    main()
