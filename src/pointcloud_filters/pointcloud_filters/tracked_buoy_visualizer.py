#!/usr/bin/env python3
"""
Tracked Buoy Visualizer Node

Visualizes fused buoys (LiDAR tracks with class_id/class_name from CV) in RViz.
Run the fusion node to get /fused_buoys.

Subscribes to:
    /fused_buoys (pointcloud_filters/FusedBuoyArray): LiDAR tracks with class_id, class_name from CV fusion

Publishes:
    /tracked_buoy_markers (visualization_msgs/MarkerArray): Visualization markers
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from pointcloud_filters.msg import FusedBuoyArray
from visualization_msgs.msg import Marker, MarkerArray
import math


# Map class_name to (r, g, b) for cylinder display only (visualization)
CLASS_NAME_TO_RGB = {
    'black_buoy': (0.2, 0.2, 0.2),
    'green_buoy': (0.0, 1.0, 0.0),
    'green_pole_buoy': (0.0, 0.85, 0.0),
    'red_buoy': (1.0, 0.0, 0.0),
    'red_pole_buoy': (1.0, 0.2, 0.2),
    'yellow_buoy': (1.0, 1.0, 0.0),
    'cross': (0.6, 0.6, 0.6),
    'dock': (0.4, 0.4, 0.6),
    'triangle': (0.6, 0.6, 0.4),
    'red_indicator_buoy': (1.0, 0.0, 0.0),
    'green_indicator_buoy': (0.0, 1.0, 0.0),
    'yellow_supply_drop': (1.0, 1.0, 0.0),
    'black_supply_drop': (0.2, 0.2, 0.2),
    'digit_1': (0.8, 0.8, 0.8),
    'digit_2': (0.8, 0.8, 0.8),
    'digit_3': (0.8, 0.8, 0.8),
    'unknown': (0.7, 0.7, 0.7),  # Grey when no CV match
}


class TrackedBuoyVisualizer(Node):
    """
    Visualizes fused buoys with stable IDs; class_id and class_name from CV.
    Cylinder color is derived from class_name for display; text label shows class_name.
    """

    def __init__(self):
        super().__init__('tracked_buoy_visualizer')

        self.declare_parameter('marker_height', 0.3)
        self.declare_parameter('marker_radius', 0.1)
        self.declare_parameter('marker_lifetime', 2.0)
        self.declare_parameter('show_text_labels', True)

        self.marker_height = self.get_parameter('marker_height').value
        self.marker_radius = self.get_parameter('marker_radius').value
        self.marker_lifetime = self.get_parameter('marker_lifetime').value
        self.show_text = self.get_parameter('show_text_labels').value

        self.active_ids = set()

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub = self.create_subscription(
            FusedBuoyArray,
            '/fused_buoys',
            self.tracked_callback,
            qos,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/tracked_buoy_markers',
            qos,
        )

        self.get_logger().info('Tracked buoy visualizer started (class_id/class_name from fusion)')
        self.get_logger().info(
            f'Marker: height={self.marker_height}m, radius={self.marker_radius}m, '
            f'lifetime={self.marker_lifetime}s, text_labels={self.show_text}'
        )

    def _rgb_for_class_name(self, class_name: str) -> tuple:
        if not class_name:
            return CLASS_NAME_TO_RGB['unknown']
        key = class_name.strip().lower()
        return CLASS_NAME_TO_RGB.get(key, CLASS_NAME_TO_RGB['unknown'])

    def tracked_callback(self, msg: FusedBuoyArray):
        marker_array = MarkerArray()
        current_ids = set()

        for buoy in msg.buoys:
            current_ids.add(buoy.id)

            marker = Marker()
            marker.header = msg.header
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = buoy.id
            marker.ns = "tracked_buoys"
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            marker.pose.position.x = buoy.x
            marker.pose.position.y = buoy.y
            marker.pose.position.z = buoy.z_mean
            marker.pose.orientation.w = 1.0

            marker.scale.x = self.marker_radius * 2.0
            marker.scale.y = self.marker_radius * 2.0
            marker.scale.z = self.marker_height

            rgb = self._rgb_for_class_name(buoy.class_name)
            confidence_intensity = min(1.0, max(0.3, buoy.confidence))
            marker.color.r = rgb[0] * confidence_intensity
            marker.color.g = rgb[1] * confidence_intensity
            marker.color.b = rgb[2] * confidence_intensity
            marker.color.a = 0.9

            marker.lifetime.sec = int(self.marker_lifetime)
            marker.lifetime.nanosec = int((self.marker_lifetime % 1.0) * 1e9)
            marker_array.markers.append(marker)

            if self.show_text:
                text_marker = Marker()
                text_marker.header = marker.header
                text_marker.id = buoy.id + 10000
                text_marker.ns = "buoy_labels"
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.pose.position.x = buoy.x
                text_marker.pose.position.y = buoy.y
                text_marker.pose.position.z = buoy.z_mean + self.marker_height + 0.2
                text_marker.pose.orientation.w = 1.0

                angle_deg = -math.degrees(buoy.bearing)
                # Show class_name, ID, range, angle
                text_marker.text = (
                    f"{buoy.class_name}\n"
                    f"ID{buoy.id}  {buoy.range:.1f}m  {angle_deg:.0f}\u00b0"
                )
                text_marker.scale.z = 0.15
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
                text_marker.color.a = 1.0
                text_marker.lifetime = marker.lifetime
                marker_array.markers.append(text_marker)

        deleted_ids = self.active_ids - current_ids
        for deleted_id in deleted_ids:
            delete_marker = Marker()
            delete_marker.header = msg.header
            delete_marker.id = deleted_id
            delete_marker.ns = "tracked_buoys"
            delete_marker.action = Marker.DELETE
            marker_array.markers.append(delete_marker)
            if self.show_text:
                delete_text = Marker()
                delete_text.header = msg.header
                delete_text.id = deleted_id + 10000
                delete_text.ns = "buoy_labels"
                delete_text.action = Marker.DELETE
                marker_array.markers.append(delete_text)

        self.active_ids = current_ids
        if len(marker_array.markers) > 0:
            self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = TrackedBuoyVisualizer()
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
