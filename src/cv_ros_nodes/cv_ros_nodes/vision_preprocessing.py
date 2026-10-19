import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import argparse


class PreprocessCamera(Node):
    
    def __init__(self, camera_id: int, image_encoding: str = 'bgr8', max_fps: float = 0.0):
        super().__init__(f'preprocess_camera{camera_id}')
        self.camera_id = camera_id
        self.image_encoding = image_encoding if image_encoding in ('bgr8', 'rgb8') else 'bgr8'
        self.bridge = CvBridge()
        self.frame_count = 0
        # Throttle: only publish at max_fps (0 = no throttle, use every frame from driver)
        self.max_fps = max(0.0, float(max_fps))
        self.min_interval_ns = int(1e9 / self.max_fps) if self.max_fps > 0 else 0
        self.last_publish_ns = 0

        # v4l2_camera publishes with RELIABLE QoS.
        # Subscriber MUST match: use RELIABLE to receive messages.
        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.subscription = self.create_subscription(
            Image,
            f'/camera{camera_id}/image_raw',
            self.image_callback,
            qos_sensor
        )

        # Publisher for processed images
        self.publisher = self.create_publisher(
            Image,
            f'/camera{camera_id}/image_preprocessed',
            10
        )
        
        self.get_logger().info(
            f'Preprocessing node for camera{camera_id} initialized (encoding={self.image_encoding}, max_fps={self.max_fps or "unlimited"})'
        )
        self.get_logger().info(f'Subscribed to: /camera{camera_id}/image_raw')
        self.get_logger().info(f'Publishing to: /camera{camera_id}/image_preprocessed')
        
    def image_callback(self, msg: Image):
        # Throttle: only process and publish at max_fps (driver still runs at full rate)
        if self.min_interval_ns > 0:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_publish_ns < self.min_interval_ns:
                return
            self.last_publish_ns = now_ns

        # 1) ROS2 Image -> OpenCV
        # Accept YUV422 directly from v4l2_camera to avoid CPU-intensive conversion
        # Only convert to BGR here (preprocessing runs at lower rate than camera)
        if msg.encoding == 'yuv422_yuy2' or msg.encoding == 'yuv422':
            import cv2
            # Log encoding info on first message
            if self.frame_count == 0:
                self.get_logger().info(f'Preprocessing input encoding: {msg.encoding} -> output encoding: {self.image_encoding}')
            # Get YUV data directly without conversion
            cv_image_yuv = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # Convert YUV to BGR (happens at preprocessing rate, not camera rate)
            cv_image = cv2.cvtColor(cv_image_yuv, cv2.COLOR_YUV2BGR_YUY2)
        else:
            # For simulation or already-converted images
            if self.frame_count == 0:
                self.get_logger().info(f'Preprocessing input encoding: {msg.encoding} -> output encoding: {self.image_encoding}')
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.image_encoding)

        # 2) Pass-through at camera resolution (no resize; blob and other CV run on full res)
        processed = cv_image

        # 3) OpenCV -> ROS2 Image
        out_msg = self.bridge.cv2_to_imgmsg(processed, encoding=self.image_encoding)

        # Preserve original header (timestamp + frame_id)
        out_msg.header = msg.header

        # 4) Publish the processed image
        self.publisher.publish(out_msg)

        self.frame_count += 1
        log_interval = max(1, int(self.max_fps) if self.max_fps > 0 else 15)
        if self.frame_count % log_interval == 0:
            self.get_logger().info(
                f'Published {log_interval} processed frames (preprocess fps={self.max_fps or "full"}, last size={processed.shape})'
            )


def main(args=None):
    rclpy.init(args=args)
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='ROS2 Camera Preprocessing Node')
    parser.add_argument('--camera_id', type=int, default=0,
                       help='Camera ID (0, 1, or 2)')
    parser.add_argument('--image_encoding', type=str, default='bgr8', choices=('bgr8', 'rgb8'),
                       help='Image encoding: bgr8 (real cameras) or rgb8 (Gazebo sim often publishes RGB).')
    parser.add_argument('--max_fps', type=float, default=0.0,
                       help='Max publish rate for preprocessed images (0 = no limit, use every frame from driver). e.g. 5 = 5 FPS only in preprocessing.')
    args_parsed, _ = parser.parse_known_args(args=args)
    
    if args_parsed.camera_id not in [0, 1, 2]:
        print("Error: camera_id must be 0, 1, or 2")
        return
    
    node = PreprocessCamera(
        camera_id=args_parsed.camera_id,
        image_encoding=args_parsed.image_encoding,
        max_fps=args_parsed.max_fps,
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

