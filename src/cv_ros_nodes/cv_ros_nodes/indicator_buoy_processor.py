"""
Indicator Buoy Processor ROS2 Node with Multi-Diamond Grouping.

Self-contained implementation with all detection logic embedded.
No external dependencies on task_specific modules.

Subscribes to: /camera{N}/image_preprocessed (N=0,1,2)
Publishes to: /camera{N}/indicator_detections (String JSON)

Features:
- Multi-diamond grouping: Handles angled buoy views by grouping nearby diamonds
- Collective centering: Uses average position of grouped diamonds for indicator ROI
- Glare handling: Adjusted thresholds for outdoor lighting conditions
- Width-based distance: Optimized for 20-inch buoy width reference
"""

import json
import sys
import time
from pathlib import Path
from typing import Tuple, Optional

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

# Class IDs for combined output (class_mapping.yaml)
CLASS_RED_INDICATOR_BUOY = 9
CLASS_GREEN_INDICATOR_BUOY = 10

# ============================================================
# DETECTION PARAMETERS
# ============================================================

# Edge detection
BLUR_KSIZE = (5, 5)
CANNY_LOW = 50
CANNY_HIGH = 150

# Diamond detection
MIN_CONTOUR_AREA = 200
EPS_RATIO = 0.03
MAX_BLACK_BRIGHTNESS = 230
MIN_SURROUND_BRIGHTNESS = 85
DEFAULT_CONF_THRESHOLD = 0.6
DEFAULT_BUOY_CONF_THRESHOLD = 0.3
MIN_WHITE_BRIGHTNESS = 100
MIN_WHITE_BLOB_SCORE = 0.15

# Multi-diamond grouping
MAX_DISTANCE_FACTOR = 2.0

# ============================================================
# CORE DETECTION FUNCTIONS (COPIED FROM colour_indicator_buoy_detector.py)
# ============================================================

def run_edge_detection(img: np.ndarray) -> np.ndarray:
    """Grayscale → Gaussian blur → Canny."""
    if img is None or img.size == 0:
        return np.array([])
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, BLUR_KSIZE, 0)
    edges = cv2.Canny(blurred, CANNY_LOW, CANNY_HIGH)
    return edges


def _is_diamond(pts: np.ndarray) -> bool:
    """STRICT diamond detection with geometry validation."""
    if pts is None or len(pts) != 4:
        return False
    pts = pts.reshape(4, 2).astype(np.float64)
    
    # Check convexity
    cross_products = []
    for i in range(4):
        p1 = pts[i]
        p2 = pts[(i + 1) % 4]
        p3 = pts[(i + 2) % 4]
        v1 = p2 - p1
        v2 = p3 - p2
        cross = v1[0] * v2[1] - v1[1] * v2[0]
        cross_products.append(cross)
    
    if not (all(c > 0 for c in cross_products) or all(c < 0 for c in cross_products)):
        return False
    
    # Side lengths uniformity
    sides = []
    for i in range(4):
        a = pts[i]
        b = pts[(i + 1) % 4]
        sides.append(np.linalg.norm(b - a))
    sides = np.array(sides)
    
    mean_side = np.mean(sides)
    if mean_side < 1e-6:
        return False
    
    ratios = sides / mean_side
    if np.any(ratios < 0.5) or np.any(ratios > 1.5):
        return False
    
    # Aspect ratio
    x_coords = pts[:, 0]
    y_coords = pts[:, 1]
    width = np.max(x_coords) - np.min(x_coords)
    height = np.max(y_coords) - np.min(y_coords)
    
    if width < 1e-6 or height < 1e-6:
        return False
    
    aspect_ratio = max(width, height) / min(width, height)
    if aspect_ratio > 2.0:
        return False
    
    # Interior angles
    angles = []
    for i in range(4):
        p1 = pts[(i - 1) % 4]
        p2 = pts[i]
        p3 = pts[(i + 1) % 4]
        
        v1 = p1 - p2
        v2 = p3 - p2
        
        v1_norm = np.linalg.norm(v1)
        v2_norm = np.linalg.norm(v2)
        
        if v1_norm < 1e-6 or v2_norm < 1e-6:
            return False
        
        v1 = v1 / v1_norm
        v2 = v2 / v2_norm
        
        cos_angle = np.clip(np.dot(v1, v2), -1.0, 1.0)
        angle_deg = np.degrees(np.arccos(cos_angle))
        angles.append(angle_deg)
    
    angles = np.array(angles)
    
    if abs(np.sum(angles) - 360.0) > 20.0:
        return False
    
    if np.any(angles < 30) or np.any(angles > 150):
        return False
    
    return True


def _mean_brightness(gray: np.ndarray, contour: np.ndarray) -> float:
    """Mean grayscale intensity inside a contour."""
    if gray.size == 0 or contour is None:
        return 255.0
    mask = np.zeros_like(gray, dtype=np.uint8)
    cv2.drawContours(mask, [contour], -1, 255, -1)
    return float(cv2.mean(gray, mask=mask)[0])


def detect_diamonds(
    img: np.ndarray,
    max_black_brightness: float = MAX_BLACK_BRIGHTNESS,
    conf_threshold: float = DEFAULT_CONF_THRESHOLD,
) -> list:
    """Detect black diamond markers on white buoy bodies."""
    if img is None or img.size == 0:
        return []

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges_raw = run_edge_detection(img)
    if edges_raw.size == 0:
        return []

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    edges = cv2.morphologyEx(edges_raw, cv2.MORPH_CLOSE, kernel, iterations=1)

    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    results = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_CONTOUR_AREA:
            continue
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, EPS_RATIO * peri, True)
        n = len(approx)
        if n != 4:
            continue

        if not _is_diamond(approx):
            continue

        x, y, w, h = cv2.boundingRect(c)
        M = cv2.moments(c)
        cx = int(M["m10"] / M["m00"]) if M["m00"] else x + w // 2
        cy = int(M["m01"] / M["m00"]) if M["m00"] else y + h // 2

        mean_brightness = _mean_brightness(gray, c)
        
        bbox_roi = gray[y:y+h, x:x+w]
        bbox_mean_brightness = float(np.mean(bbox_roi)) if bbox_roi.size > 0 else 255.0
        
        # Black filter
        if mean_brightness > max_black_brightness or bbox_mean_brightness > max_black_brightness:
            continue
        
        # Surrounding region check
        expand_factor = 0.5
        expand_w = int(w * expand_factor)
        expand_h = int(h * expand_factor)
        surround_x1 = max(0, x - expand_w)
        surround_y1 = max(0, y - expand_h)
        surround_x2 = min(gray.shape[1], x + w + expand_w)
        surround_y2 = min(gray.shape[0], y + h + expand_h)
        
        surround_roi = gray[surround_y1:surround_y2, surround_x1:surround_x2]
        if surround_roi.size > 0:
            surround_mean = float(np.mean(surround_roi))
            if surround_mean < MIN_SURROUND_BRIGHTNESS:
                continue

        # Confidence scoring
        pts = approx.reshape(-1, 2).astype(np.float64)
        side_lengths = []
        for i in range(len(pts)):
            a = pts[i]
            b = pts[(i + 1) % len(pts)]
            side_lengths.append(np.linalg.norm(b - a))
        side_lengths = np.array(side_lengths, dtype=np.float64)
        if side_lengths.size >= 2:
            mean_side = side_lengths.mean()
            std_side = side_lengths.std()
            if mean_side > 1e-6:
                shape_score = max(0.0, 1.0 - (std_side / (mean_side * 2.0)))
            else:
                shape_score = 0.0
        else:
            shape_score = 0.0

        if max_black_brightness > 0:
            black_score = float(np.clip((max_black_brightness - mean_brightness) / max_black_brightness, 0.0, 1.0))
        else:
            black_score = 1.0

        gray_roi = gray[y:y + h, x:x + w]
        mask_full = np.zeros_like(gray, dtype=np.uint8)
        cv2.drawContours(mask_full, [c], -1, 255, -1)
        mask_roi = mask_full[y:y + h, x:x + w]
        bg_mask = cv2.bitwise_not(mask_roi)
        if np.count_nonzero(bg_mask) > 0:
            bg_mean = cv2.mean(gray_roi, mask=bg_mask)[0]
        else:
            bg_mean = 255.0

        min_white = 150.0
        max_white = 255.0
        white_score = float(np.clip((bg_mean - min_white) / max(1.0, max_white - min_white), 0.0, 1.0))

        confidence = float((shape_score + black_score + white_score) / 3.0)

        if confidence < conf_threshold:
            continue

        results.append({
            "bbox": (x, y, w, h),
            "center": (cx, cy),
            "area": area,
            "contour": approx,
            "confidence": confidence,
            "mean_brightness": mean_brightness,
        })

    return results


def detect_white_blob_around_diamond(
    gray: np.ndarray,
    diamond_bbox: Tuple[int, int, int, int],
    diamond_contour: np.ndarray,
    expansion_factor: float = 2.0,
    min_white_brightness: float = MIN_WHITE_BRIGHTNESS,
) -> Tuple[float, Tuple[int, int, int, int]]:
    """Check for white/grey blob around a diamond."""
    img_h, img_w = gray.shape[:2]
    x, y, w, h = diamond_bbox
    
    expand_w = int(w * (expansion_factor - 1.0) / 2.0)
    expand_h = int(h * (expansion_factor - 1.0) / 2.0)
    
    search_x1 = max(0, x - expand_w)
    search_y1 = max(0, y - expand_h)
    search_x2 = min(img_w, x + w + expand_w)
    search_y2 = min(img_h, y + h + expand_h)
    
    search_w = search_x2 - search_x1
    search_h = search_y2 - search_y1
    
    if search_w <= 0 or search_h <= 0:
        return 0.0, (search_x1, search_y1, search_w, search_h)
    
    search_roi = gray[search_y1:search_y2, search_x1:search_x2]
    
    mask_full = np.zeros_like(gray, dtype=np.uint8)
    cv2.drawContours(mask_full, [diamond_contour], -1, 255, -1)
    diamond_mask = mask_full[search_y1:search_y2, search_x1:search_x2]
    
    surround_mask = cv2.bitwise_not(diamond_mask)
    
    white_pixels = np.sum((search_roi >= min_white_brightness) & (surround_mask > 0))
    total_surround_pixels = np.sum(surround_mask > 0)
    
    if total_surround_pixels == 0:
        return 0.0, (search_x1, search_y1, search_w, search_h)
    
    white_ratio = white_pixels / total_surround_pixels
    
    surround_mean = cv2.mean(search_roi, mask=surround_mask)[0]
    brightness_score = np.clip((surround_mean - min_white_brightness) / (255.0 - min_white_brightness), 0.0, 1.0)
    
    white_blob_score = float((white_ratio + brightness_score) / 2.0)
    
    return white_blob_score, (search_x1, search_y1, search_w, search_h)


def _indicator_red_or_green(roi_bgr: np.ndarray) -> Tuple[str, float]:
    """Fallback: decide red vs green from ROI when indicator detector returns 'none'."""
    if roi_bgr is None or roi_bgr.size == 0:
        return "red", 0.5
    B, G, R = cv2.split(roi_bgr)
    mean_r = float(np.mean(R))
    mean_g = float(np.mean(G))
    total = mean_r + mean_g
    if total < 1e-6:
        return "red", 0.5
    conf_red = mean_r / total
    conf_green = mean_g / total
    if conf_red >= conf_green:
        return "red", conf_red
    return "green", conf_green


def detect_indicator_simple(roi_bgr: np.ndarray) -> Tuple[str, float, Optional[Tuple[int, int, int, int]]]:
    """
    Simplified indicator detection using color masks.
    Returns (state, confidence, bbox) where state is "red", "green", or "none".
    """
    if roi_bgr is None or roi_bgr.size == 0 or roi_bgr.shape[0] < 5 or roi_bgr.shape[1] < 5:
        return "none", 0.0, None
    
    B, G, R = cv2.split(roi_bgr)
    
    # Primary color masks (stricter)
    red_mask = (R > 120) & (R > G + 25) & (R > B + 25) & (G < 150) & (B < 150)
    green_mask = (G > 80) & (G > R + 5) & (G > B + 5) & (R < 200) & (B < 200)
    
    red_count = np.sum(red_mask)
    green_count = np.sum(green_mask)
    total_pixels = roi_bgr.shape[0] * roi_bgr.shape[1]
    
    red_ratio = red_count / total_pixels
    green_ratio = green_count / total_pixels
    
    # Determine state
    if green_ratio > 0.05 and green_ratio > red_ratio * 1.5:
        state = "green"
        conf = float(min(1.0, green_ratio * 10))
    elif red_ratio > 0.05 and red_ratio > green_ratio * 1.5:
        state = "red"
        conf = float(min(1.0, red_ratio * 10))
    else:
        state = "none"
        conf = 0.0
    
    # Simple bbox: entire ROI
    bbox = (0, 0, roi_bgr.shape[1], roi_bgr.shape[0]) if state != "none" else None
    
    return state, conf, bbox


# ============================================================
# MULTI-DIAMOND GROUPING AND CLASSIFICATION
# ============================================================

def classify_colour_indicator_buoy(
    img_bgr: np.ndarray,
    conf_threshold: float = DEFAULT_CONF_THRESHOLD,
    max_black_brightness: float = MAX_BLACK_BRIGHTNESS,
    buoy_conf_threshold: float = DEFAULT_BUOY_CONF_THRESHOLD,
    white_blob_expansion: float = 2.0,
    min_white_brightness: float = MIN_WHITE_BRIGHTNESS,
    min_white_blob_score: float = MIN_WHITE_BLOB_SCORE,
) -> dict:
    """
    Full pipeline with multi-diamond grouping.
    Returns dict with "buoys" list containing detection info.
    """
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    
    # 1) Detect all black diamonds
    all_diamonds = detect_diamonds(img_bgr, max_black_brightness, conf_threshold)
    
    if not all_diamonds:
        return {"buoys": []}
    
    # 2) Check white blob around each diamond
    buoy_candidates = []
    for diamond in all_diamonds:
        white_blob_score, white_blob_bbox = detect_white_blob_around_diamond(
            gray,
            diamond["bbox"],
            diamond["contour"],
            expansion_factor=white_blob_expansion,
            min_white_brightness=min_white_brightness,
        )
        
        diamond_conf = diamond.get("confidence", 0.0)
        buoy_confidence = (diamond_conf + white_blob_score) / 2.0
        
        buoy_candidates.append({
            "diamond": diamond,
            "white_blob_score": white_blob_score,
            "white_blob_bbox": white_blob_bbox,
            "buoy_confidence": buoy_confidence,
        })
    
    # 3) Filter by confidence
    high_conf_buoys = [
        b for b in buoy_candidates 
        if b["buoy_confidence"] >= buoy_conf_threshold 
        and b["white_blob_score"] >= min_white_blob_score
    ]
    
    if not high_conf_buoys:
        return {"buoys": []}
    
    # 4) Group nearby diamonds (MULTI-DIAMOND GROUPING)
    def _diamonds_are_nearby(d1_bbox, d2_bbox, max_distance_factor=MAX_DISTANCE_FACTOR):
        x1, y1, w1, h1 = d1_bbox
        x2, y2, w2, h2 = d2_bbox
        c1_x, c1_y = x1 + w1 // 2, y1 + h1 // 2
        c2_x, c2_y = x2 + w2 // 2, y2 + h2 // 2
        distance = ((c2_x - c1_x) ** 2 + (c2_y - c1_y) ** 2) ** 0.5
        avg_size = (w1 + h1 + w2 + h2) / 4.0
        max_distance = avg_size * max_distance_factor
        return distance <= max_distance
    
    sorted_buoys = sorted(high_conf_buoys, key=lambda b: -b["buoy_confidence"])
    buoy_groups = []
    
    for buoy in sorted_buoys:
        diamond_bbox = buoy["diamond"]["bbox"]
        assigned_to_group = False
        for group in buoy_groups:
            if any(_diamonds_are_nearby(diamond_bbox, existing["diamond"]["bbox"]) for existing in group):
                group.append(buoy)
                assigned_to_group = True
                break
        if not assigned_to_group:
            buoy_groups.append([buoy])
    
    # 5) Process each group
    all_final_buoys = []
    
    for group in buoy_groups:
        if len(group) == 1:
            # Single diamond
            buoy = group[0]
            diamond = buoy["diamond"]
            dx, dy, dw, dh = diamond["bbox"]
            white_blob_bbox = buoy["white_blob_bbox"]
            
            diamond_center_x = dx + dw // 2
            roi_w = int(dw * 3.0)
            roi_h = int(dh * 2.0)
            
            roi_x1 = max(0, diamond_center_x - roi_w // 2)
            roi_x2 = min(img_bgr.shape[1], roi_x1 + roi_w)
            roi_y2 = max(0, dy)
            roi_y1 = max(0, roi_y2 - roi_h)
            
            indicator_roi = (roi_x1, roi_y1, roi_x2 - roi_x1, roi_y2 - roi_y1)
            
            representative_diamond = diamond
            combined_white_blob_bbox = white_blob_bbox
            combined_buoy_confidence = buoy["buoy_confidence"]
            
        else:
            # Multiple diamonds - use collective center
            diamonds = [b["diamond"] for b in group]
            
            centers_x = [d["bbox"][0] + d["bbox"][2] // 2 for d in diamonds]
            centers_y = [d["bbox"][1] + d["bbox"][3] // 2 for d in diamonds]
            collective_center_x = sum(centers_x) // len(centers_x)
            collective_center_y = sum(centers_y) // len(centers_y)
            
            x1s = [d["bbox"][0] for d in diamonds]
            y1s = [d["bbox"][1] for d in diamonds]
            collective_y1 = min(y1s)
            
            best_buoy = max(group, key=lambda b: b["buoy_confidence"])
            representative_diamond = best_buoy["diamond"]
            
            white_blobs = [b["white_blob_bbox"] for b in group]
            wb_x1s = [wb[0] for wb in white_blobs]
            wb_y1s = [wb[1] for wb in white_blobs]
            wb_x2s = [wb[0] + wb[2] for wb in white_blobs]
            wb_y2s = [wb[1] + wb[3] for wb in white_blobs]
            
            combined_wb_x1 = min(wb_x1s)
            combined_wb_y1 = min(wb_y1s)
            combined_wb_x2 = max(wb_x2s)
            combined_wb_y2 = max(wb_y2s)
            combined_white_blob_bbox = (combined_wb_x1, combined_wb_y1, 
                                     combined_wb_x2 - combined_wb_x1, 
                                     combined_wb_y2 - combined_wb_y1)
            
            combined_buoy_confidence = sum(b["buoy_confidence"] for b in group) / len(group)
            
            avg_diamond_size = sum(d["bbox"][2] + d["bbox"][3] for d in diamonds) / (2 * len(diamonds))
            roi_w = int(avg_diamond_size * 3.0)
            roi_h = int(avg_diamond_size * 2.0)
            
            roi_x1 = max(0, collective_center_x - roi_w // 2)
            roi_x2 = min(img_bgr.shape[1], collective_center_x + roi_w // 2)
            roi_y2 = max(0, collective_y1)
            roi_y1 = max(0, roi_y2 - roi_h)
            
            indicator_roi = (roi_x1, roi_y1, roi_x2 - roi_x1, roi_y2 - roi_y1)
        
        # Detect indicator
        rx, ry, rw, rh = indicator_roi
        if rw > 0 and rh > 0:
            roi_img = img_bgr[ry:ry + rh, rx:rx + rw]
            state, state_conf, ind_bbox = detect_indicator_simple(roi_img)
            if state == "none":
                state, state_conf = _indicator_red_or_green(roi_img)
        else:
            state, state_conf, ind_bbox = "red", 0.5, None
        
        indicator_bbox_global = None
        if ind_bbox is not None:
            bx, by, bw, bh = ind_bbox
            indicator_bbox_global = (rx + bx, ry + by, bw, bh)
        
        # Full bbox
        components = [combined_white_blob_bbox]
        if indicator_bbox_global:
            ix, iy, iw, ih = indicator_bbox_global
            components.append((ix, iy, iw, ih))
        
        x1s = [x for (x, y, w, h) in components]
        y1s = [y for (x, y, w, h) in components]
        x2s = [x + w for (x, y, w, h) in components]
        y2s = [y + h for (x, y, w, h) in components]
        
        full_x1 = max(0, min(x1s))
        full_y1 = max(0, min(y1s))
        full_x2 = min(img_bgr.shape[1], max(x2s))
        full_y2 = min(img_bgr.shape[0], max(y2s))
        full_bbox = (full_x1, full_y1, full_x2 - full_x1, full_y2 - full_y1)
        
        all_final_buoys.append({
            "diamond": representative_diamond,
            "white_blob_score": combined_buoy_confidence,
            "white_blob_bbox": combined_white_blob_bbox,
            "buoy_confidence": combined_buoy_confidence,
            "indicator_state": state,
            "indicator_conf": float(state_conf),
            "indicator_bbox": indicator_bbox_global,
            "indicator_roi": indicator_roi,
            "full_bbox": full_bbox,
        })
    
    return {"buoys": all_final_buoys}


# ============================================================
# ROS NODE
# ============================================================

def _parse_camera_ids(camera_ids_str: str) -> list:
    """Parse comma-separated camera IDs (e.g. '1' -> [1], '0,1,2' -> [0,1,2])."""
    if not camera_ids_str or not camera_ids_str.strip():
        return [0, 1, 2]
    return [int(x.strip()) for x in camera_ids_str.split(',') if x.strip().isdigit()]


class IndicatorBuoyProcessor(Node):
    def __init__(
        self, 
        conf_threshold: float = 0.6,
        max_black_brightness: float = 230.0,
        buoy_conf_threshold: float = 0.3,
        white_blob_expansion: float = 2.0,
        min_white_brightness: float = 100.0,
        min_white_blob_score: float = 0.15,
        camera_ids: str = "0,1,2",
    ):
        super().__init__("indicator_buoy_processor")
        self.bridge = CvBridge()
        self.camera_ids = _parse_camera_ids(camera_ids) or [0, 1, 2]
        
        self.conf_threshold = conf_threshold
        self.max_black_brightness = max_black_brightness
        self.buoy_conf_threshold = buoy_conf_threshold
        self.white_blob_expansion = white_blob_expansion
        self.min_white_brightness = min_white_brightness
        self.min_white_blob_score = min_white_blob_score

        for cam_id in self.camera_ids:
            self.create_subscription(
                Image,
                f"/camera{cam_id}/image_preprocessed",
                lambda msg, cid=cam_id: self._image_callback(msg, cid),
                10,
            )
        
        self._pubs = {}
        for cam_id in self.camera_ids:
            self._pubs[cam_id] = self.create_publisher(
                String, f"/camera{cam_id}/indicator_detections", 10
            )

        self.get_logger().info("Indicator buoy processor initialized (self-contained)")
        self.get_logger().info(f"Parameters: conf={conf_threshold}, max_black={max_black_brightness}")

    def _image_callback(self, msg: Image, camera_id: int) -> None:
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"image_callback camera{camera_id}: {e}")
            return

        try:
            info = classify_colour_indicator_buoy(
                bgr,
                conf_threshold=self.conf_threshold,
                max_black_brightness=self.max_black_brightness,
                buoy_conf_threshold=self.buoy_conf_threshold,
                white_blob_expansion=self.white_blob_expansion,
                min_white_brightness=self.min_white_brightness,
                min_white_blob_score=self.min_white_blob_score,
            )
        except Exception as e:
            self.get_logger().error(f"classify_colour_indicator_buoy camera{camera_id}: {e}")
            self._publish(camera_id, time.time(), [])
            return

        detections = []
        for buoy in info.get("buoys", []):
            state = buoy.get("indicator_state", "none")
            indicator_conf = float(buoy.get("indicator_conf", 0.0))
            buoy_conf = float(buoy.get("buoy_confidence", 0.0))
            
            if state not in ("red", "green"):
                continue
            
            full_bbox = buoy.get("full_bbox")
            if full_bbox and len(full_bbox) == 4:
                x, y, w, h = full_bbox
                x1 = max(0, float(x))
                y1 = max(0, float(y))
                x2 = min(bgr.shape[1], float(x + w))
                y2 = min(bgr.shape[0], float(y + h))
                if x1 < x2 and y1 < y2:
                    class_id = CLASS_RED_INDICATOR_BUOY if state == "red" else CLASS_GREEN_INDICATOR_BUOY
                    detections.append({
                        "class_id": class_id,
                        "score": buoy_conf,
                        "shape_bbox": [x1, y1, x2, y2],
                        "indicator_color": state,
                        "indicator_confidence": indicator_conf,
                        "source": "indicator_buoy",
                    })

        stamp = msg.header.stamp
        timestamp = stamp.sec + stamp.nanosec * 1e-9
        self._publish(camera_id, timestamp, detections)

    def _publish(self, camera_id: int, timestamp: float, detections: list) -> None:
        payload = {
            "camera_id": camera_id,
            "timestamp": timestamp,
            "detections": detections,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._pubs[camera_id].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    import argparse
    p = argparse.ArgumentParser(description="Indicator buoy processor with multi-diamond grouping")
    p.add_argument("--conf_threshold", type=float, default=0.6, help="Diamond confidence threshold")
    p.add_argument("--max_black_brightness", type=float, default=230.0, help="Max brightness for black diamonds")
    p.add_argument("--buoy_conf_threshold", type=float, default=0.3, help="Combined buoy confidence threshold")
    p.add_argument("--white_blob_expansion", type=float, default=2.0, help="White blob search expansion factor")
    p.add_argument("--min_white_brightness", type=float, default=100.0, help="Minimum white brightness")
    p.add_argument("--min_white_blob_score", type=float, default=0.15, help="Minimum white blob score")
    p.add_argument("--camera_ids", type=str, default="0,1,2", help="Comma-separated camera IDs (e.g. 1 or 0,1,2)")
    parsed, _ = p.parse_known_args(args=args)
    node = IndicatorBuoyProcessor(
        conf_threshold=parsed.conf_threshold,
        max_black_brightness=parsed.max_black_brightness,
        buoy_conf_threshold=parsed.buoy_conf_threshold,
        white_blob_expansion=parsed.white_blob_expansion,
        min_white_brightness=parsed.min_white_brightness,
        min_white_blob_score=parsed.min_white_blob_score,
        camera_ids=parsed.camera_ids,
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


if __name__ == "__main__":
    main()
