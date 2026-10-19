#!/usr/bin/env python3
"""
Patrol boat detection for RoboBoat dynamic challenge (Finals) – ROS2 node.

Detects a yellow, flattened-rectangle patrol boat in preprocessed camera frames.
This node subscribes to the preprocessed stream (`/camera{N}/image_preprocessed`)
coming from the `cv_ros_nodes` pipeline and publishes a simple Boolean flag when
the patrol boat is seen.

Topics:
  - Subscribes: `/camera{camera_id}/image_preprocessed` (sensor_msgs/Image, BGR)
  - Publishes:  `/patrol_boat/detected` (std_msgs/Bool)

Downstream, the task sequence coordinator listens to `/patrol_boat/detected` and
handles LOITER/STOPPING/RESUMING behaviour.
"""

from typing import List, Tuple

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge


# Default config: yellow patrol boat, flattened, at least 100 px high
class PatrolBoatConfig:
    """Configuration for patrol boat detection."""

    # Yellow in HSV (same ballpark as task4 pipeline)
    YELLOW_HSV_LOWER = np.array([18, 80, 100])
    YELLOW_HSV_UPPER = np.array([35, 255, 255])

    # Minimum size: boat must be "big enough" in frame
    MIN_BLOB_AREA = 20000  # area in pixels (tune for your camera)
    MIN_BLOB_WIDTH = 150
    MIN_BLOB_HEIGHT = 80  # patrol boat must be at least ~80 px high

    # Shape constraints:
    # - Flattened rectangle: height/width must be below this (clearly not square)
    # - But also not an infinitely long stripe: width/height must be < this ratio
    MAX_ASPECT_RATIO = 0.6         # h/w < 0.6  (flattened)
    MAX_WIDTH_HEIGHT_RATIO = 15.0  # w/h < 15   (not crazy long)

    # Spatial constraint: only consider blobs in lower 60% of frame
    LOWER_FRAME_THRESHOLD = 0.4  # center_y must be > 40% of image height

    MORPH_KERNEL_SIZE = 5


def detect_patrol_boat(
    image: np.ndarray,
    config: PatrolBoatConfig | None = None,
) -> Tuple[bool, List[Tuple[int, int, int, int]]]:
    """
    Detect yellow patrol boat(s) in a BGR image.

    Args:
        image: BGR image (e.g. from cv2.imread or ROS Image message).
        config: Optional config; uses PatrolBoatConfig() if None.

    Returns:
        (detected, bboxes): detected is True if at least one valid yellow flattened
        blob with height > 100 px is found; bboxes is list of (x1, y1, x2, y2).
    """
    if config is None:
        config = PatrolBoatConfig()

    if image is None or image.size == 0:
        return False, []

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, config.YELLOW_HSV_LOWER, config.YELLOW_HSV_UPPER)

    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (config.MORPH_KERNEL_SIZE, config.MORPH_KERNEL_SIZE),
    )
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    bboxes: List[Tuple[int, int, int, int]] = []
    img_h = image.shape[0]
    lower_y_thresh = int(img_h * config.LOWER_FRAME_THRESHOLD)

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < config.MIN_BLOB_AREA:
            continue

        x, y, w, h = cv2.boundingRect(contour)
        if w < config.MIN_BLOB_WIDTH or h < config.MIN_BLOB_HEIGHT:
            continue

        # Spatial: center of blob must be in lower 60% of frame
        center_y = y + h // 2
        if center_y <= lower_y_thresh:
            continue

        aspect_ratio = h / w if w > 0 else float("inf")
        if aspect_ratio > config.MAX_ASPECT_RATIO:
            continue

        # Also require width/height < MAX_WIDTH_HEIGHT_RATIO (avoid ultra-long stripes)
        width_height_ratio = w / h if h > 0 else float("inf")
        if width_height_ratio > config.MAX_WIDTH_HEIGHT_RATIO:
            continue

        bboxes.append((x, y, x + w, y + h))

    return len(bboxes) > 0, bboxes


class PatrolBoatDetectorNode(Node):
    """ROS2 node that runs patrol-boat detection on preprocessed camera frames."""

    def __init__(self, camera_id: int = 0):
        super().__init__("patrol_boat_detector")
        self._bridge = CvBridge()
        self._config = PatrolBoatConfig()
        self._camera_id = camera_id

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        topic = f"/camera{camera_id}/image_preprocessed"
        self._sub = self.create_subscription(
            Image, topic, self._image_callback, qos
        )
        self._pub = self.create_publisher(Bool, "/patrol_boat/detected", 10)

        self.get_logger().info(
            f"PatrolBoatDetectorNode listening on {topic}, publishing /patrol_boat/detected"
        )

    def _image_callback(self, msg: Image) -> None:
        try:
            # Preprocessed node already gives us BGR
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        try:
            detected, _ = detect_patrol_boat(frame, self._config)
        except Exception as e:
            self.get_logger().error(f"patrol detection error: {e}")
            return

        if detected:
            m = Bool()
            m.data = True
            self._pub.publish(m)


def main(args=None):
    import argparse

    rclpy.init(args=args)
    p = argparse.ArgumentParser(description="Patrol boat detector node")
    p.add_argument(
        "--camera_id",
        type=int,
        default=0,
        help="Camera ID (subscribes to /camera{camera_id}/image_preprocessed)",
    )
    parsed, _ = p.parse_known_args(args=args)
    node = PatrolBoatDetectorNode(camera_id=parsed.camera_id)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
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
