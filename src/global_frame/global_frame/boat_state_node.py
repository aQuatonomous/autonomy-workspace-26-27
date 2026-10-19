#!/usr/bin/env python3
"""
Boat state node: NavSatFix + heading -> (east, north, heading_rad) in map frame.

Subscribes to:
  - mavros/global_position/global (sensor_msgs/NavSatFix)
  - Configurable heading topic (std_msgs/Float64 degrees from North, or geometry_msgs/PoseStamped)

Publishes:
  - /boat_pose (global_frame/BoatPose): east, north, heading_rad
  - geometry_msgs/PoseStamped in frame "map" (optional)
  - TF map -> base_link (optional)

Origin: (lat0, lon0) from params or use first GPS fix.
Heading convention: Published heading_rad is angle from East (0 = East, positive = CCW).
"""

import math
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from builtin_interfaces.msg import Time
import tf2_ros

from global_frame.msg import BoatPose

# Inline lat/lon -> ENU (avoids import path issues when run as installed executable)
_R_EARTH_EQUATORIAL = 6378137.0
_R_EARTH_POLAR = 6356752.314245


def _lat_lon_to_enu(lat_deg: float, lon_deg: float, lat0_deg: float, lon0_deg: float) -> tuple[float, float]:
    lat_rad = math.radians(lat_deg)
    lon_rad = math.radians(lon_deg)
    lat0_rad = math.radians(lat0_deg)
    lon0_rad = math.radians(lon0_deg)
    e2 = 1.0 - (_R_EARTH_POLAR / _R_EARTH_EQUATORIAL) ** 2
    sin_lat = math.sin(lat0_rad)
    r_merid = _R_EARTH_EQUATORIAL * (1.0 - e2) / ((1.0 - e2 * sin_lat * sin_lat) ** 1.5)
    east = (lon_rad - lon0_rad) * math.cos(lat0_rad) * _R_EARTH_EQUATORIAL
    north = (lat_rad - lat0_rad) * r_merid
    return (east, north)


def quaternion_from_yaw(yaw_rad: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw_rad / 2.0)
    q.w = math.cos(yaw_rad / 2.0)
    return q


def yaw_from_quaternion(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class BoatStateNode(Node):
    def __init__(self):
        super().__init__("boat_state_node")

        self.declare_parameter("global_position_topic", "mavros/global_position/global")
        self.declare_parameter("heading_topic", "mavros/global_position/compass_hdg")
        self.declare_parameter("heading_topic_type", "compass_hdg")  # "compass_hdg" | "pose"
        self.declare_parameter("use_first_fix_as_origin", True)
        self.declare_parameter("origin_lat_deg", 0.0)
        self.declare_parameter("origin_lon_deg", 0.0)
        self.declare_parameter("publish_pose_stamped", True)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_link_frame", "base_link")

        self._global_position_topic = self.get_parameter("global_position_topic").value
        self._heading_topic = self.get_parameter("heading_topic").value
        self._heading_topic_type = self.get_parameter("heading_topic_type").value
        self._use_first_fix_as_origin = self.get_parameter("use_first_fix_as_origin").value
        self._origin_lat = self.get_parameter("origin_lat_deg").value
        self._origin_lon = self.get_parameter("origin_lon_deg").value
        self._publish_pose_stamped = self.get_parameter("publish_pose_stamped").value
        self._publish_tf = self.get_parameter("publish_tf").value
        self._map_frame = self.get_parameter("map_frame").value
        self._base_link_frame = self.get_parameter("base_link_frame").value

        self._origin_set = not self._use_first_fix_as_origin
        self._last_east: Optional[float] = None
        self._last_north: Optional[float] = None
        self._last_heading_rad: Optional[float] = None

        # RELIABLE QoS for our own publishers and any internal subscribers
        qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        # BEST_EFFORT QoS to match MAVROS sensor publishers (NavSatFix, Float64 compass)
        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._boat_pose_pub = self.create_publisher(BoatPose, "/boat_pose", qos_reliable)
        if self._publish_pose_stamped:
            self._pose_pub = self.create_publisher(
                PoseStamped, "/boat_pose_stamped", qos_reliable
            )
        if self._publish_tf:
            self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self._gps_sub = self.create_subscription(
            NavSatFix,
            self._global_position_topic,
            self._gps_callback,
            qos_sensor,
        )

        if self._heading_topic_type == "pose":
            from geometry_msgs.msg import PoseStamped as PoseStampedMsg
            self._heading_sub = self.create_subscription(
                PoseStampedMsg,
                self._heading_topic,
                self._heading_pose_callback,
                qos_sensor,
            )
        else:
            self._heading_sub = self.create_subscription(
                Float64,
                self._heading_topic,
                self._heading_compass_callback,
                qos_sensor,
            )

        self.get_logger().info(
            f"Boat state node: GPS={self._global_position_topic}, heading={self._heading_topic} ({self._heading_topic_type}), "
            f"origin={'first fix' if self._use_first_fix_as_origin else f'({self._origin_lat}, {self._origin_lon})'}"
        )

    def _heading_compass_callback(self, msg: Float64) -> None:
        # compass_hdg: 0 = North, 90 = East (degrees). Convert to ENU heading (0 = East, 90 = North).
        hdg_deg = float(msg.data)
        self._last_heading_rad = math.radians(90.0 - hdg_deg)
        # Publish with whatever position we have (0,0 if no GPS yet) so heading is always live
        east = self._last_east if self._last_east is not None else 0.0
        north = self._last_north if self._last_north is not None else 0.0
        self._publish_pose(east, north, self._last_heading_rad, self.get_clock().now().to_msg())

    def _heading_pose_callback(self, msg: PoseStamped) -> None:
        yaw = yaw_from_quaternion(msg.pose.orientation)
        self._last_heading_rad = yaw
        east = self._last_east if self._last_east is not None else 0.0
        north = self._last_north if self._last_north is not None else 0.0
        self._publish_pose(east, north, self._last_heading_rad, msg.header.stamp)

    def _gps_callback(self, msg: NavSatFix) -> None:
        if msg.status.status < 0:
            return
        lat = msg.latitude
        lon = msg.longitude

        if self._use_first_fix_as_origin and not self._origin_set:
            self._origin_lat = lat
            self._origin_lon = lon
            self._origin_set = True
            self.get_logger().info(f"Origin set from first fix: ({lat:.6f}, {lon:.6f})")

        east, north = _lat_lon_to_enu(lat, lon, self._origin_lat, self._origin_lon)
        self._last_east = east
        self._last_north = north

        if self._last_heading_rad is None:
            self._last_heading_rad = 0.0

        stamp = msg.header.stamp
        self._publish_pose(east, north, self._last_heading_rad, stamp)

    def _publish_pose(
        self, east: float, north: float, heading_rad: float, stamp: Time
    ) -> None:
        # BoatPose
        boat_msg = BoatPose()
        boat_msg.header.stamp = stamp
        boat_msg.header.frame_id = self._map_frame
        boat_msg.east = east
        boat_msg.north = north
        boat_msg.heading_rad = heading_rad
        self._boat_pose_pub.publish(boat_msg)

        if self._publish_pose_stamped:
            pose_msg = PoseStamped()
            pose_msg.header = boat_msg.header
            pose_msg.pose.position.x = east
            pose_msg.pose.position.y = north
            pose_msg.pose.position.z = 0.0
            pose_msg.pose.orientation = quaternion_from_yaw(heading_rad)
            self._pose_pub.publish(pose_msg)

        if self._publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self._map_frame
            t.child_frame_id = self._base_link_frame
            t.transform.translation.x = east
            t.transform.translation.y = north
            t.transform.translation.z = 0.0
            t.transform.rotation = quaternion_from_yaw(heading_rad)
            self._tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = BoatStateNode()
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
