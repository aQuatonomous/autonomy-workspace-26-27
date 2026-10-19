#!/usr/bin/env python3
"""
Map Visualizer ROS 2 Node.

Subscribes to /boat_pose and /global_detections, maintains state,
and serves a web-based 2D map visualization via HTTP/WebSocket.
"""

import asyncio
import threading
import time
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from global_frame.msg import BoatPose, GlobalDetectionArray

from .web_server import MapWebServer


class MapVisualizerNode(Node):
    """
    ROS 2 node that subscribes to boat pose and global detections,
    and serves a web-based map visualization.
    """
    
    def __init__(self):
        super().__init__('map_visualizer_node')
        
        # Declare parameters
        self.declare_parameter('port', 8080)
        self.declare_parameter('update_rate_hz', 10.0)
        self.declare_parameter('boat_pose_topic', '/boat_pose')
        self.declare_parameter('global_detections_topic', '/global_detections')
        
        # Get parameters
        self.port = self.get_parameter('port').value
        self.update_rate_hz = self.get_parameter('update_rate_hz').value
        self.boat_pose_topic = self.get_parameter('boat_pose_topic').value
        self.global_detections_topic = self.get_parameter('global_detections_topic').value
        
        # Create web server
        self.web_server = MapWebServer(
            port=self.port,
            update_rate_hz=self.update_rate_hz
        )
        
        # QoS profile
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create subscriptions
        self.boat_pose_sub = self.create_subscription(
            BoatPose,
            self.boat_pose_topic,
            self.boat_pose_callback,
            qos
        )
        
        self.global_detections_sub = self.create_subscription(
            GlobalDetectionArray,
            self.global_detections_topic,
            self.global_detections_callback,
            qos
        )
        
        # Start web server in separate thread
        self.web_server_thread = threading.Thread(target=self._run_web_server, daemon=True)
        self.web_server_thread.start()
        
        self.get_logger().info(f'Map Visualizer Node started')
        self.get_logger().info(f'Web server: http://0.0.0.0:{self.port}')
        self.get_logger().info(f'Web directory: {self.web_server.web_dir}')
        self.get_logger().info(f'Subscribing to: {self.boat_pose_topic}, {self.global_detections_topic}')
        self.get_logger().info(f'Update rate: {self.update_rate_hz} Hz')
        self.get_logger().info(f'Access via SSH: ssh -L {self.port}:localhost:{self.port} user@jetson-ip')
    
    def _run_web_server(self):
        """Run the web server in an asyncio event loop."""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        try:
            loop.run_until_complete(self.web_server.start())
            loop.run_forever()
        except Exception as e:
            self.get_logger().error(f'Web server error: {e}')
        finally:
            loop.run_until_complete(self.web_server.stop())
            loop.close()
    
    def boat_pose_callback(self, msg: BoatPose):
        """
        Callback for boat pose updates.
        
        Args:
            msg: BoatPose message with east, north, heading_rad
        """
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        self.web_server.update_boat_state(
            east=msg.east,
            north=msg.north,
            heading_rad=msg.heading_rad,
            timestamp=timestamp
        )
        
        self.get_logger().debug(
            f'Boat pose: east={msg.east:.2f}, north={msg.north:.2f}, '
            f'heading={msg.heading_rad:.2f} rad',
            throttle_duration_sec=2.0
        )
    
    def global_detections_callback(self, msg: GlobalDetectionArray):
        """
        Callback for global detections updates.
        
        Args:
            msg: GlobalDetectionArray message with detection list
        """
        detections = []
        for det in msg.detections:
            detections.append({
                'east': det.east,
                'north': det.north,
                'class_name': det.class_name,
                'source': det.source,
                'id': det.id,
                'range_m': det.range_m,
                'bearing_global_rad': det.bearing_global_rad
            })
        
        self.web_server.update_detections(detections)
        
        self.get_logger().debug(
            f'Global detections: {len(detections)} detections',
            throttle_duration_sec=2.0
        )
    
    def destroy_node(self):
        """Clean up resources."""
        self.get_logger().info('Shutting down map visualizer node...')
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = MapVisualizerNode()
    executor = MultiThreadedExecutor()
    
    try:
        rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
