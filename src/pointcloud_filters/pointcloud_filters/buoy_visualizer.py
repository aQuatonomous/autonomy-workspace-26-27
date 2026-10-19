#!/usr/bin/env python3
"""
Buoy Detection Visualizer Node

This node subscribes to buoy detections and publishes visualization markers for RViz.
Each detected buoy is shown as a cylinder at its detected position.

Subscribes to:
    /buoy_detections (pointcloud_filters/BuoyDetectionArray): Detected buoys

Publishes:
    /buoy_markers (visualization_msgs/MarkerArray): Visualization markers for RViz
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from pointcloud_filters.msg import BuoyDetectionArray
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import math


class BuoyVisualizer(Node):
    """
    Converts buoy detections (range, bearing) into 3D markers for visualization in RViz.
    
    The markers are cylinders positioned at the detected buoy locations. Color intensity
    represents detection confidence - brighter colors = higher confidence.
    """
    
    def __init__(self):
        super().__init__('buoy_visualizer')
        
        # Parameters
        self.declare_parameter('marker_height', 0.3)  # Height of cylinder markers (meters)
        self.declare_parameter('marker_radius', 0.1)  # Radius of cylinder markers (meters)
        self.declare_parameter('marker_lifetime', 0.5)  # How long markers persist (seconds)
        
        self.marker_height = self.get_parameter('marker_height').value
        self.marker_radius = self.get_parameter('marker_radius').value
        self.marker_lifetime = self.get_parameter('marker_lifetime').value
        
        # QoS profile: depth=1 for low-latency marker updates
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        
        # Subscriber to buoy detections
        self.sub = self.create_subscription(
            BuoyDetectionArray,
            '/buoy_detections',
            self.detections_callback,
            qos
        )
        
        # Publisher for visualization markers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/buoy_markers',
            qos
        )
        
        self.get_logger().info('Buoy visualizer node started')
        self.get_logger().info(f'Marker parameters: height={self.marker_height}m, '
                              f'radius={self.marker_radius}m, '
                              f'lifetime={self.marker_lifetime}s')
    
    def detections_callback(self, msg: BuoyDetectionArray):
        """
        Convert each buoy detection into a visualization marker.
        
        For each detected buoy:
        1. Convert polar coordinates (range, bearing) to Cartesian (x, y)
        2. Create a cylinder marker at that position
        3. Color the marker based on confidence (green with varying intensity)
        """
        marker_array = MarkerArray()
        
        # Create a marker for each detected buoy
        for i, detection in enumerate(msg.detections):
            marker = Marker()
            
            # Header - use same frame and timestamp as detection
            marker.header = msg.header
            marker.header.stamp = self.get_clock().now().to_msg()
            
            # Unique ID for this marker
            marker.id = i
            marker.ns = "buoys"
            
            # Marker type: cylinder represents a buoy
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            # Convert polar (range, bearing) to Cartesian (x, y)
            # bearing is in radians, range is in meters
            x = detection.range * math.cos(detection.bearing)
            y = detection.range * math.sin(detection.bearing)
            
            # Position the marker
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = detection.z_mean  # Use actual height from detection
            
            # Orientation (upright cylinder)
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            
            # Scale: diameter and height of the cylinder
            marker.scale.x = self.marker_radius * 2.0  # diameter in x
            marker.scale.y = self.marker_radius * 2.0  # diameter in y
            marker.scale.z = self.marker_height  # height
            
            # Color: Green with intensity based on confidence
            # Higher confidence = brighter/more opaque green
            marker.color = ColorRGBA()
            marker.color.r = 0.0
            marker.color.g = 1.0  # Green
            marker.color.b = 0.0
            marker.color.a = min(1.0, detection.confidence)  # Alpha (transparency) from confidence
            
            # Lifetime: how long this marker stays visible
            marker.lifetime.sec = int(self.marker_lifetime)
            marker.lifetime.nanosec = int((self.marker_lifetime % 1.0) * 1e9)
            
            marker_array.markers.append(marker)
            
            # Log detection info
            self.get_logger().info(
                f'Buoy {i}: range={detection.range:.2f}m, '
                f'bearing={math.degrees(detection.bearing):.1f}°, '
                f'position=({x:.2f}, {y:.2f}), '
                f'confidence={detection.confidence:.2f}'
            )
        
        # If we had previous detections but now have none, publish a delete-all marker
        if len(marker_array.markers) == 0:
            delete_marker = Marker()
            delete_marker.header = msg.header
            delete_marker.header.stamp = self.get_clock().now().to_msg()
            delete_marker.ns = "buoys"
            delete_marker.id = 0
            delete_marker.action = Marker.DELETEALL
            marker_array.markers.append(delete_marker)
        
        # Publish the marker array
        self.marker_pub.publish(marker_array)
        self.get_logger().info(f'Published {len(msg.detections)} buoy markers')


def main(args=None):
    rclpy.init(args=args)
    node = BuoyVisualizer()
    
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
