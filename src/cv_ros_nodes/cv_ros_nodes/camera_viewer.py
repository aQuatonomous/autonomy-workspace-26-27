import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import os
from pathlib import Path


class CameraViewer(Node):
    def __init__(self, camera_id: int = 0, output_dir: str = "./camera_images"):
        super().__init__(f'camera_viewer_camera{camera_id}')
        self.camera_id = camera_id
        self.bridge = CvBridge()
        
        # Create output directory if it doesn't exist
        self.OUT_DIR = Path(output_dir)
        self.OUT_DIR.mkdir(parents=True, exist_ok=True)
        
        # Subscribe to camera image topic
        self.subscription = self.create_subscription(
            Image,
            f'/camera{camera_id}/image_raw',
            self.image_callback,
            10
        )
        
        self.get_logger().info(f'Camera viewer for camera{camera_id} initialized')
        self.get_logger().info(f'Subscribed to: /camera{camera_id}/image_raw')
        self.get_logger().info(f'Output directory: {self.OUT_DIR}')
    
    def image_callback(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            tmp_path = self.OUT_DIR / f"cam{self.camera_id}.tmp.jpg"
            final_path = self.OUT_DIR / f"cam{self.camera_id}.jpg"
            
            cv2.imwrite(str(tmp_path), cv_img)
            os.replace(tmp_path, final_path)  # atomic rename
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')


def main(args=None):
    import argparse
    
    rclpy.init(args=args)
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='ROS2 Camera Viewer - Saves camera images to disk')
    parser.add_argument('--camera_id', type=int, default=0,
                       help='Camera ID (0, 1, or 2)')
    parser.add_argument('--output_dir', type=str, default='./camera_images',
                       help='Output directory for saved images')
    args_parsed = parser.parse_args(args)
    
    node = CameraViewer(
        camera_id=args_parsed.camera_id,
        output_dir=args_parsed.output_dir
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