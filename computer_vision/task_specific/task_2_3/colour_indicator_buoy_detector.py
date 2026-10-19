"""
Colour Indicator Buoy Detector

Pipeline:
  1) Detect individual black diamonds on the buoy body.
  2) For each diamond, check for a white/grey blob around it (2x diamond width).
  3) Compute combined confidence (diamond shape quality + white blob presence).
  4) For each high-confidence diamond+white blob, define an ROI directly above
     the diamond to detect the red/green indicator.
  5) Return multiple buoy detections, each with:
     - Diamond detection
     - White blob region
     - Indicator color/confidence
     - Full bounding box (diamond + white blob + indicator)
"""

import argparse
import sys
from pathlib import Path
from typing import Tuple, Optional

import cv2
import numpy as np

# ============================================================
# Core CV utilities: edge detection + diamond detection
# (inlined here so this file is the single source of truth for
# the entire diamond + indicator pipeline).
# ============================================================

# Edge detection parameters
BLUR_KSIZE = (5, 5)
CANNY_LOW = 50
CANNY_HIGH = 150


def run_edge_detection(
    img: np.ndarray,
    *,
    canny_low: int = CANNY_LOW,
    canny_high: int = CANNY_HIGH,
    blur_ksize: tuple = BLUR_KSIZE,
) -> np.ndarray:
    """Grayscale → Gaussian blur → Canny. Returns single-channel edge image (0–255)."""
    if img is None or img.size == 0:
        return np.array([])
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, blur_ksize, 0)
    edges = cv2.Canny(blurred, canny_low, canny_high)
    return edges


# Minimum contour area (avoid noise)
MIN_CONTOUR_AREA = 200
# Approx polygon: epsilon = eps_ratio * perimeter.
# STRICTER: Lower epsilon means tighter polygon approximation, requiring clearer corners
EPS_RATIO = 0.03
# Diamond filter: mean grayscale inside contour must be below this (0–255) - increased for glare/lighting
MAX_BLACK_BRIGHTNESS = 230
# Surrounding region (around diamond) must have mean brightness above this to be a white buoy
MIN_SURROUND_BRIGHTNESS = 85
# Default confidence threshold for accepting diamonds (0–1) - increased for stricter detection
DEFAULT_CONF_THRESHOLD = 0.6
# Default confidence threshold for accepting full buoys (diamond + white blob) (lowered for outdoor scenes)
DEFAULT_BUOY_CONF_THRESHOLD = 0.3
# White blob detection: minimum brightness to be considered "white/grey" (lowered for bright outdoor scenes)
MIN_WHITE_BRIGHTNESS = 100
# Minimum white blob score to consider it a valid buoy (lowered for outdoor lighting)
MIN_WHITE_BLOB_SCORE = 0.15
# When two full buoy bboxes overlap above this IoU, keep only the one with higher diamond confidence
FULL_BBOX_OVERLAP_IOU_THRESHOLD = 0.05
# Intersection-over-min-area: if smaller box is this fraction covered by the other, treat as overlap
FULL_BBOX_OVERLAP_IO_MIN_THRESHOLD = 0.35
# When two diamond bboxes overlap above this IoU, treat as same buoy (keep higher confidence)
DIAMOND_BBOX_OVERLAP_IOU_THRESHOLD = 0.3


def _is_diamond(pts: np.ndarray) -> bool:
    """
    STRICT diamond detection: 4-point contour must have clear corners and diamond-like geometry.
    - 4 vertices (already guaranteed by caller)
    - All 4 sides roughly equal length (like a rotated square)
    - Interior angles close to 90 degrees (for squares) or two acute/two obtuse (for diamonds)
    - Convex shape (no concave corners)
    - Aspect ratio not too elongated
    """
    if pts is None or len(pts) != 4:
        return False
    pts = pts.reshape(4, 2).astype(np.float64)
    
    # 1. Check if convex (cross products should all have same sign)
    cross_products = []
    for i in range(4):
        p1 = pts[i]
        p2 = pts[(i + 1) % 4]
        p3 = pts[(i + 2) % 4]
        v1 = p2 - p1
        v2 = p3 - p2
        cross = v1[0] * v2[1] - v1[1] * v2[0]
        cross_products.append(cross)
    
    # All cross products should have same sign (all positive or all negative)
    if not (all(c > 0 for c in cross_products) or all(c < 0 for c in cross_products)):
        return False
    
    # 2. Side lengths - must be relatively uniform
    sides = []
    for i in range(4):
        a = pts[i]
        b = pts[(i + 1) % 4]
        sides.append(np.linalg.norm(b - a))
    sides = np.array(sides)
    
    mean_side = np.mean(sides)
    if mean_side < 1e-6:
        return False
    
    # STRICTER: All sides must be within 50% of mean (was 80% tolerance before)
    ratios = sides / mean_side
    if np.any(ratios < 0.5) or np.any(ratios > 1.5):
        return False
    
    # 3. Check aspect ratio of bounding box (diamonds shouldn't be too elongated)
    x_coords = pts[:, 0]
    y_coords = pts[:, 1]
    width = np.max(x_coords) - np.min(x_coords)
    height = np.max(y_coords) - np.min(y_coords)
    
    if width < 1e-6 or height < 1e-6:
        return False
    
    aspect_ratio = max(width, height) / min(width, height)
    # Diamonds should be roughly square-ish, not extremely elongated
    if aspect_ratio > 2.0:
        return False
    
    # 4. Check interior angles - should be roughly 90° or two acute + two obtuse
    angles = []
    for i in range(4):
        p1 = pts[(i - 1) % 4]
        p2 = pts[i]
        p3 = pts[(i + 1) % 4]
        
        v1 = p1 - p2
        v2 = p3 - p2
        
        # Normalize vectors
        v1_norm = np.linalg.norm(v1)
        v2_norm = np.linalg.norm(v2)
        
        if v1_norm < 1e-6 or v2_norm < 1e-6:
            return False
        
        v1 = v1 / v1_norm
        v2 = v2 / v2_norm
        
        # Compute angle using dot product
        cos_angle = np.clip(np.dot(v1, v2), -1.0, 1.0)
        angle_deg = np.degrees(np.arccos(cos_angle))
        angles.append(angle_deg)
    
    angles = np.array(angles)
    
    # Angles should sum to 360 degrees (within tolerance)
    if abs(np.sum(angles) - 360.0) > 20.0:
        return False
    
    # For a diamond: angles should be reasonable (not too acute or too obtuse)
    # Reject if any angle is too extreme
    if np.any(angles < 30) or np.any(angles > 150):
        return False
    
    return True


def _mean_brightness(gray: np.ndarray, contour: np.ndarray) -> float:
    """Mean grayscale intensity inside a contour (0–255). Returns 255.0 on failure."""
    if gray.size == 0 or contour is None:
        return 255.0
    mask = np.zeros_like(gray, dtype=np.uint8)
    cv2.drawContours(mask, [contour], -1, 255, -1)
    return float(cv2.mean(gray, mask=mask)[0])


def detect_white_blob_around_diamond(
    gray: np.ndarray,
    diamond_bbox: Tuple[int, int, int, int],
    diamond_contour: np.ndarray,
    expansion_factor: float = 2.0,
    min_white_brightness: float = MIN_WHITE_BRIGHTNESS,
) -> Tuple[float, Tuple[int, int, int, int]]:
    """
    Check for white/grey blob around a diamond.
    
    Args:
        gray: Grayscale image
        diamond_bbox: (x, y, w, h) of the diamond
        diamond_contour: Diamond contour for masking out the diamond itself
        expansion_factor: How much to expand search region (2.0 = 2x diamond width/height)
        min_white_brightness: Minimum brightness to consider as "white"
    
    Returns:
        (white_blob_score, expanded_bbox):
            - white_blob_score: 0-1, how much white is around the diamond
            - expanded_bbox: (x, y, w, h) of the search region
    """
    img_h, img_w = gray.shape[:2]
    x, y, w, h = diamond_bbox
    
    # Expand search region by expansion_factor
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
    
    # Extract the search region
    search_roi = gray[search_y1:search_y2, search_x1:search_x2]
    
    # Create mask to exclude the diamond itself
    mask_full = np.zeros_like(gray, dtype=np.uint8)
    cv2.drawContours(mask_full, [diamond_contour], -1, 255, -1)
    diamond_mask = mask_full[search_y1:search_y2, search_x1:search_x2]
    
    # Invert mask: we want the region AROUND the diamond, not the diamond itself
    surround_mask = cv2.bitwise_not(diamond_mask)
    
    # Count white pixels in the surrounding region
    white_pixels = np.sum((search_roi >= min_white_brightness) & (surround_mask > 0))
    total_surround_pixels = np.sum(surround_mask > 0)
    
    if total_surround_pixels == 0:
        return 0.0, (search_x1, search_y1, search_w, search_h)
    
    # Score: ratio of white pixels in surrounding region
    white_ratio = white_pixels / total_surround_pixels
    
    # Also check mean brightness of surrounding region
    surround_mean = cv2.mean(search_roi, mask=surround_mask)[0]
    brightness_score = np.clip((surround_mean - min_white_brightness) / (255.0 - min_white_brightness), 0.0, 1.0)
    
    # Combined score: average of ratio and brightness
    white_blob_score = float((white_ratio + brightness_score) / 2.0)
    
    return white_blob_score, (search_x1, search_y1, search_w, search_h)


def _shape_name(pts: np.ndarray) -> str:
    """Classify contour by vertex count and return shape name."""
    n = len(pts)
    if n == 3:
        return "triangle"
    if n == 4:
        return "diamond" if _is_diamond(pts) else "quad"
    if n == 5:
        return "pentagon"
    if n == 6:
        return "hexagon"
    if n > 6:
        return "circle"
    return "unknown"


def detect_shapes(
    img: np.ndarray,
    *,
    canny_low: int = CANNY_LOW,
    canny_high: int = CANNY_HIGH,
    min_area: float = MIN_CONTOUR_AREA,
    eps_ratio: float = EPS_RATIO,
    filter_diamond_only: bool = False,
    filter_black_only: bool = False,
    max_black_brightness: float = MAX_BLACK_BRIGHTNESS,
    conf_threshold: float = 0.0,
) -> list[dict]:
    """
    Run edge-based shape detection on a BGR image.

    When filter_diamond_only and filter_black_only are True, only diamonds whose
    interior is dark (mean grayscale <= max_black_brightness) are returned.

    Returns list of dicts:
      - "shape": str (e.g. "diamond", "triangle", "quad")
      - "bbox": (x, y, w, h)
      - "center": (cx, cy)
      - "area": float
      - "contour": np.ndarray (optional, for drawing)
    """
    if img is None or img.size == 0:
        return []

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges_raw = run_edge_detection(img, canny_low=canny_low, canny_high=canny_high)
    if edges_raw.size == 0:
        return []

    # Make edges more "solid" so contours form closed loops, and retrieve all
    # contours (including inner ones like the printed diamond), not just the
    # outer hull.
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    edges = cv2.morphologyEx(edges_raw, cv2.MORPH_CLOSE, kernel, iterations=1)

    contours, _ = cv2.findContours(
        edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
    )

    results: list[dict] = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, eps_ratio * peri, True)
        n = len(approx)
        if n < 3:
            continue

        shape = _shape_name(approx)
        if filter_diamond_only and shape != "diamond":
            continue

        # Bounding box and center (needed for confidence and output)
        x, y, w, h = cv2.boundingRect(c)
        M = cv2.moments(c)
        cx = int(M["m10"] / M["m00"]) if M["m00"] else x + w // 2
        cy = int(M["m01"] / M["m00"]) if M["m00"] else y + h // 2

        # Brightness inside the full contour (used for black filter and confidence)
        mean_brightness = _mean_brightness(gray, c)
        
        # ADDITIONAL CHECK: Also verify the bounding box mean brightness
        # This catches cases where the contour edge is dark but interior is bright
        bbox_roi = gray[y:y+h, x:x+w]
        bbox_mean_brightness = float(np.mean(bbox_roi)) if bbox_roi.size > 0 else 255.0
        
        if filter_black_only:
            # BOTH the contour AND the bounding box must be dark
            if mean_brightness > max_black_brightness or bbox_mean_brightness > max_black_brightness:
                continue
            
            # ADDITIONAL CHECK: The surrounding region should be relatively bright (white buoy body)
            # Expand bbox to check surrounding brightness
            expand_factor = 0.5  # Expand by 50% on each side
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

        # Heuristic confidence score (0–1)
        confidence = 1.0
        if shape == "diamond":
            # Side-length uniformity (how "diamond-like" the polygon is)
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
                    # 0 when std is large, 1 when all sides equal
                    shape_score = max(0.0, 1.0 - (std_side / (mean_side * 2.0)))
                else:
                    shape_score = 0.0
            else:
                shape_score = 0.0

            # Darkness score (1 = perfectly black up to threshold, 0 = bright)
            if max_black_brightness > 0:
                black_score = float(
                    np.clip(
                        (max_black_brightness - mean_brightness) / max_black_brightness,
                        0.0,
                        1.0,
                    )
                )
            else:
                black_score = 1.0

            # Background whiteness score: measure brightness outside contour within bbox
            gray_roi = gray[y:y + h, x:x + w]
            mask_full = np.zeros_like(gray, dtype=np.uint8)
            cv2.drawContours(mask_full, [c], -1, 255, -1)
            mask_roi = mask_full[y:y + h, x:x + w]
            bg_mask = cv2.bitwise_not(mask_roi)
            # Avoid division by zero if bg_mask has no pixels
            if np.count_nonzero(bg_mask) > 0:
                bg_mean = cv2.mean(gray_roi, mask=bg_mask)[0]
            else:
                bg_mean = 255.0

            # Map background brightness to [0,1]; expect white buoy ~ 200-255
            min_white = 150.0
            max_white = 255.0
            white_score = float(
                np.clip((bg_mean - min_white) / max(1.0, max_white - min_white), 0.0, 1.0)
            )

            # Final confidence = mixture of shape, blackness, and white background
            confidence = float(
                (shape_score + black_score + white_score) / 3.0
            )

            # Apply confidence threshold for diamonds if requested
            if conf_threshold > 0.0 and confidence < conf_threshold:
                continue

        results.append({
            "shape": shape,
            "bbox": (x, y, w, h),
            "center": (cx, cy),
            "area": area,
            "contour": approx,
            "confidence": confidence,
            "mean_brightness": mean_brightness,
        })

    return results


def detect_diamonds(
    img: np.ndarray,
    *,
    black_only: bool = True,
    max_black_brightness: float = MAX_BLACK_BRIGHTNESS,
    conf_threshold: float = DEFAULT_CONF_THRESHOLD,
    **kwargs,
) -> list[dict]:
    """Return only diamond detections. By default only black/dark diamonds (Task 3 logo)."""
    return detect_shapes(
        img,
        filter_diamond_only=True,
        filter_black_only=black_only,
        max_black_brightness=max_black_brightness,
        conf_threshold=conf_threshold,
        **kwargs,
    )


def draw_detections(img: np.ndarray, detections: list[dict], labels: bool = True) -> np.ndarray:
    """Draw bounding boxes and shape labels on a copy of the image."""
    out = img.copy()
    for d in detections:
        x, y, w, h = d["bbox"]
        box_color = (0, 255, 0) if d["shape"] == "diamond" else (200, 200, 200)
        cv2.rectangle(out, (x, y), (x + w, y + h), box_color, 2)
        if d.get("contour") is not None:
            cv2.drawContours(out, [d["contour"]], -1, box_color, 2)
        if labels:
            conf = float(d.get("confidence", 0.0))
            label = f"{conf:.2f}" if d["shape"] == "diamond" else d["shape"]
            text_color = (0, 0, 255)
            cv2.putText(
                out,
                label,
                (x, max(10, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                text_color,
                1,
                cv2.LINE_AA,
            )
    return out


def _load_indicator_detector():
    """
    Import detect_indicator from the task_specific/indicator folder.
    Returns the function object.
    """
    script_dir = Path(__file__).resolve().parent
    indicator_dir = script_dir.parent / "indicator"
    if str(indicator_dir) not in sys.path:
        sys.path.append(str(indicator_dir))
    try:
        from indicator_detector import detect_indicator  # type: ignore
    except ImportError as e:
        raise RuntimeError(f"Failed to import indicator_detector.py from {indicator_dir}") from e
    return detect_indicator


def _indicator_red_or_green(roi_bgr: np.ndarray) -> Tuple[str, float]:
    """
    When indicator detector returns 'none', decide red vs green from ROI.
    Returns ('red', conf) or ('green', conf) based on which channel dominates.
    """
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


def _bbox_iou(b1: Tuple[int, int, int, int], b2: Tuple[int, int, int, int]) -> float:
    """Intersection-over-union of two (x, y, w, h) boxes."""
    x1, y1, w1, h1 = b1
    x2, y2, w2, h2 = b2
    xa1, ya1 = x1, y1
    xa2, ya2 = x1 + w1, y1 + h1
    xb1, yb1 = x2, y2
    xb2, yb2 = x2 + w2, y2 + h2

    inter_x1 = max(xa1, xb1)
    inter_y1 = max(ya1, yb1)
    inter_x2 = min(xa2, xb2)
    inter_y2 = min(ya2, yb2)

    inter_w = max(0, inter_x2 - inter_x1)
    inter_h = max(0, inter_y2 - inter_y1)
    inter_area = inter_w * inter_h
    if inter_area == 0:
        return 0.0

    area_a = w1 * h1
    area_b = w2 * h2
    union = area_a + area_b - inter_area
    if union <= 0:
        return 0.0
    return inter_area / union


def _full_bboxes_overlap(b1: Tuple[int, int, int, int], b2: Tuple[int, int, int, int]) -> bool:
    """
    True if two full buoy bboxes overlap enough that we should keep only one.
    Uses IoU and intersection-over-min-area so we catch both similar-size overlap
    and one-box-inside-the-other.
    """
    x1, y1, w1, h1 = b1
    x2, y2, w2, h2 = b2
    inter_x1 = max(x1, x2)
    inter_y1 = max(y1, y2)
    inter_x2 = min(x1 + w1, x2 + w2)
    inter_y2 = min(y1 + h1, y2 + h2)
    inter_w = max(0, inter_x2 - inter_x1)
    inter_h = max(0, inter_y2 - inter_y1)
    inter_area = inter_w * inter_h
    if inter_area == 0:
        return False
    area_a = w1 * h1
    area_b = w2 * h2
    if area_a <= 0 or area_b <= 0:
        return False
    union = area_a + area_b - inter_area
    if union > 0:
        iou = inter_area / union
        if iou >= FULL_BBOX_OVERLAP_IOU_THRESHOLD:
            return True
    io_min = inter_area / min(area_a, area_b)
    return io_min >= FULL_BBOX_OVERLAP_IO_MIN_THRESHOLD


def _compute_indicator_roi(
    img_shape: Tuple[int, int, int],
    diamond_boxes: list[Tuple[int, int, int, int]],
    height_factor: float = 1.0,
) -> Optional[Tuple[int, int, int, int]]:
    """
    Given a list of diamond bounding boxes (x, y, w, h), compute a single
    ROI above the buoy body where the red/green indicator should be.

    height_factor controls how tall the ROI is relative to buoy height.
    """
    if not diamond_boxes:
        return None

    img_h, img_w = img_shape[:2]
    num_diamonds = len(diamond_boxes)

    # If we only have one diamond, grow a box centered on that diamond
    # that is wider and significantly taller than its own bbox.
    if num_diamonds == 1:
        x, y, w, h = diamond_boxes[0]
        center_x = x + w // 2

        # Slightly wider than diamond bbox
        roi_w = int(w * 1.5)
        # Search column above the diamond: e.g. 2x its height
        # (scaled by height_factor).
        roi_h = int(h * 2.0 * height_factor)

        roi_x1 = max(0, center_x - roi_w // 2)
        roi_x2 = min(img_w - 1, roi_x1 + roi_w)

        roi_y2 = max(0, y)  # bottom at top of diamond bbox
        roi_y1 = max(0, roi_y2 - roi_h)
    else:
        # Two or more diamonds: use the union box, but grow it a bit wider and taller.
        x1s = [x for (x, y, w, h) in diamond_boxes]
        y1s = [y for (x, y, w, h) in diamond_boxes]
        x2s = [x + w for (x, y, w, h) in diamond_boxes]
        y2s = [y + h for (x, y, w, h) in diamond_boxes]

        buoy_x1 = max(0, min(x1s))
        buoy_x2 = min(img_w - 1, max(x2s))
        buoy_y1 = max(0, min(y1s))
        buoy_y2 = min(img_h - 1, max(y2s))
        buoy_h = max(1, buoy_y2 - buoy_y1)
        buoy_w = buoy_x2 - buoy_x1
        buoy_center_x = (buoy_x1 + buoy_x2) // 2

        # "Slightly bigger" than total left-right coverage: e.g. 1.2x width.
        roi_w = int(buoy_w * 1.2)
        roi_x1 = max(0, buoy_center_x - roi_w // 2)
        roi_x2 = min(img_w - 1, roi_x1 + roi_w)

        # Height significantly higher than buoy height above the diamonds.
        roi_h = int(buoy_h * height_factor * 1.6)
        roi_y2 = max(0, buoy_y1)  # bottom of ROI = top of diamonds
        roi_y1 = max(0, roi_y2 - roi_h)

    if roi_y1 >= roi_y2 or roi_x1 >= roi_x2:
        return None

    return roi_x1, roi_y1, roi_x2 - roi_x1, roi_y2 - roi_y1


def classify_colour_indicator_buoy(
    img_bgr: np.ndarray,
    *,
    conf_threshold: float = DEFAULT_CONF_THRESHOLD,
    max_black_brightness: float = MAX_BLACK_BRIGHTNESS,
    roi_conf_threshold: float = 0.6,
    buoy_conf_threshold: float = DEFAULT_BUOY_CONF_THRESHOLD,
    white_blob_expansion: float = 2.0,  # Reduced from 4.0 for tighter bounding boxes
    min_white_brightness: float = MIN_WHITE_BRIGHTNESS,
    min_white_blob_score: float = MIN_WHITE_BLOB_SCORE,
) -> Tuple[np.ndarray, dict]:
    """
    Full pipeline on a single BGR frame. Detects multiple independent buoys.

    Returns:
      - Annotated BGR image.
      - Info dict with:
          {
            "buoys": [
              {
                "diamond": {...},
                "white_blob_score": float,
                "white_blob_bbox": (x, y, w, h),
                "buoy_confidence": float,
                "indicator_state": "red"/"green"/"none",
                "indicator_conf": float,
                "indicator_bbox": (x, y, w, h) or None,
                "indicator_roi": (x, y, w, h),
                "full_bbox": (x, y, w, h)  # encompasses everything
              },
              ...
            ]
          }
    """
    detect_indicator = _load_indicator_detector()
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    
    # 1) Detect all black diamonds with basic confidence threshold
    all_diamonds = detect_diamonds(
        img_bgr,
        black_only=True,
        max_black_brightness=max_black_brightness,
        conf_threshold=conf_threshold,
    )
    
    if not all_diamonds:
        return img_bgr.copy(), {"buoys": []}
    
    # 2) For each diamond, check for white blob around it and compute combined confidence
    buoy_candidates = []
    for diamond in all_diamonds:
        white_blob_score, white_blob_bbox = detect_white_blob_around_diamond(
            gray,
            diamond["bbox"],
            diamond["contour"],
            expansion_factor=white_blob_expansion,
            min_white_brightness=min_white_brightness,
        )
        
        # Combined confidence: average of diamond confidence and white blob score
        diamond_conf = diamond.get("confidence", 0.0)
        buoy_confidence = (diamond_conf + white_blob_score) / 2.0
        
        buoy_candidates.append({
            "diamond": diamond,
            "white_blob_score": white_blob_score,
            "white_blob_bbox": white_blob_bbox,
            "buoy_confidence": buoy_confidence,
        })
    
    # 3) Filter by combined buoy confidence threshold AND minimum white blob score
    high_conf_buoys = [
        b for b in buoy_candidates 
        if b["buoy_confidence"] >= buoy_conf_threshold 
        and b["white_blob_score"] >= min_white_blob_score
    ]
    
    if not high_conf_buoys:
        return img_bgr.copy(), {"buoys": []}
    
    # 4) Group nearby high-confidence diamonds that likely belong to the same buoy
    def _diamonds_are_nearby(d1_bbox, d2_bbox, max_distance_factor=2.0):
        """Check if two diamonds are close enough to belong to the same buoy."""
        x1, y1, w1, h1 = d1_bbox
        x2, y2, w2, h2 = d2_bbox
        
        # Centers of the diamonds
        c1_x, c1_y = x1 + w1 // 2, y1 + h1 // 2
        c2_x, c2_y = x2 + w2 // 2, y2 + h2 // 2
        
        # Distance between centers
        distance = ((c2_x - c1_x) ** 2 + (c2_y - c1_y) ** 2) ** 0.5
        
        # Max allowed distance based on diamond sizes
        avg_size = (w1 + h1 + w2 + h2) / 4.0
        max_distance = avg_size * max_distance_factor
        
        return distance <= max_distance
    
    # Group diamonds that are nearby
    sorted_buoys = sorted(high_conf_buoys, key=lambda b: -b["buoy_confidence"])
    buoy_groups = []
    
    for buoy in sorted_buoys:
        diamond_bbox = buoy["diamond"]["bbox"]
        
        # Find existing group this diamond belongs to
        assigned_to_group = False
        for group in buoy_groups:
            # Check if this diamond is nearby any diamond in the group
            if any(_diamonds_are_nearby(diamond_bbox, existing["diamond"]["bbox"]) 
                   for existing in group):
                group.append(buoy)
                assigned_to_group = True
                break
        
        # If not assigned to any group, create a new group
        if not assigned_to_group:
            buoy_groups.append([buoy])
    
    # 5) For each buoy group, use the collective center for indicator detection
    all_final_buoys = []
    
    for group in buoy_groups:
        if len(group) == 1:
            # Single diamond - use original logic
            buoy = group[0]
            diamond = buoy["diamond"]
            dx, dy, dw, dh = diamond["bbox"]
            white_blob_bbox = buoy["white_blob_bbox"]
            
            # Compute indicator ROI: directly above the diamond, centered on it
            diamond_center_x = dx + dw // 2
            roi_w = int(dw * 3.0)  # 3x diamond width for wider indicator search
            roi_h = int(dh * 2.0)  # 2x diamond height above
            
            roi_x1 = max(0, diamond_center_x - roi_w // 2)
            roi_x2 = min(img_bgr.shape[1], roi_x1 + roi_w)
            roi_y2 = max(0, dy)  # Bottom of ROI at top of diamond
            roi_y1 = max(0, roi_y2 - roi_h)
            
            indicator_roi = (roi_x1, roi_y1, roi_x2 - roi_x1, roi_y2 - roi_y1)
            
            # Use the single diamond's properties
            representative_diamond = diamond
            combined_white_blob_bbox = white_blob_bbox
            combined_buoy_confidence = buoy["buoy_confidence"]
            
        else:
            # Multiple diamonds - use collective center and properties
            diamonds = [b["diamond"] for b in group]
            
            # Compute collective center of all diamonds
            centers_x = [d["bbox"][0] + d["bbox"][2] // 2 for d in diamonds]
            centers_y = [d["bbox"][1] + d["bbox"][3] // 2 for d in diamonds]
            collective_center_x = sum(centers_x) // len(centers_x)
            collective_center_y = sum(centers_y) // len(centers_y)
            
            # Compute collective bounding box of all diamonds
            x1s = [d["bbox"][0] for d in diamonds]
            y1s = [d["bbox"][1] for d in diamonds]
            x2s = [d["bbox"][0] + d["bbox"][2] for d in diamonds]
            y2s = [d["bbox"][1] + d["bbox"][3] for d in diamonds]
            
            collective_x1 = min(x1s)
            collective_y1 = min(y1s)
            collective_x2 = max(x2s)
            collective_y2 = max(y2s)
            collective_w = collective_x2 - collective_x1
            collective_h = collective_y2 - collective_y1
            
            # Use the highest confidence diamond as representative
            best_buoy = max(group, key=lambda b: b["buoy_confidence"])
            representative_diamond = best_buoy["diamond"]
            
            # Combine white blob bboxes
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
            
            # Average confidence
            combined_buoy_confidence = sum(b["buoy_confidence"] for b in group) / len(group)
            
            # Compute indicator ROI: above the collective center
            avg_diamond_size = sum(d["bbox"][2] + d["bbox"][3] for d in diamonds) / (2 * len(diamonds))
            roi_w = int(avg_diamond_size * 3.0)  # 3x average diamond size
            roi_h = int(avg_diamond_size * 2.0)  # 2x average diamond size above
            
            roi_x1 = max(0, collective_center_x - roi_w // 2)
            roi_x2 = min(img_bgr.shape[1], collective_center_x + roi_w // 2)
            roi_y2 = max(0, collective_y1)  # Bottom of ROI at top of collective diamonds
            roi_y1 = max(0, roi_y2 - roi_h)
            
            indicator_roi = (roi_x1, roi_y1, roi_x2 - roi_x1, roi_y2 - roi_y1)
        
        # Extract ROI and run indicator detector
        rx, ry, rw, rh = indicator_roi
        if rw > 0 and rh > 0:
            roi_img = img_bgr[ry:ry + rh, rx:rx + rw]
            state, state_conf, ind_bbox = detect_indicator(roi_img)
            # Never show "none": pick red or green based on which is more present in ROI
            if state == "none":
                state, state_conf = _indicator_red_or_green(roi_img)
        else:
            state, state_conf, ind_bbox = "red", 0.5, None  # fallback to red with low conf
        
        # Project indicator bbox to full image coordinates
        indicator_bbox_global = None
        if ind_bbox is not None:
            bx, by, bw, bh = ind_bbox
            indicator_bbox_global = (rx + bx, ry + by, bw, bh)
        
        # Compute full bounding box: union of combined white blob, diamonds, and indicator
        components = [combined_white_blob_bbox]
        if indicator_bbox_global:
            ix, iy, iw, ih = indicator_bbox_global
            components.append((ix, iy, iw, ih))
        
        # Union of all components
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
            "white_blob_score": combined_buoy_confidence,  # Use combined confidence as proxy
            "white_blob_bbox": combined_white_blob_bbox,
            "buoy_confidence": combined_buoy_confidence,
            "indicator_state": state,
            "indicator_conf": float(state_conf),
            "indicator_bbox": indicator_bbox_global,
            "indicator_roi": indicator_roi,
            "full_bbox": full_bbox,
        })
    
    # 6) Resolve overlapping full (pink) bboxes and duplicate diamonds: keep higher diamond confidence
    sorted_by_diamond_conf = sorted(
        all_final_buoys,
        key=lambda b: -(b["diamond"].get("confidence", 0.0)),
    )
    final_buoys = []
    for buoy in sorted_by_diamond_conf:
        full_bbox = buoy["full_bbox"]
        diamond_bbox = buoy["diamond"]["bbox"]
        # Suppress if full bbox overlaps (IoU or IoMin) OR diamond bbox overlaps with any kept
        if all(
            not _full_bboxes_overlap(full_bbox, kept["full_bbox"])
            and _bbox_iou(diamond_bbox, kept["diamond"]["bbox"]) < DIAMOND_BBOX_OVERLAP_IOU_THRESHOLD
            for kept in final_buoys
        ):
            final_buoys.append(buoy)
    
    # 7) Draw only the kept buoys
    annotated = img_bgr.copy()
    for buoy in final_buoys:
        diamond = buoy["diamond"]
        dx, dy, dw, dh = diamond["bbox"]
        white_blob_bbox = buoy["white_blob_bbox"]
        full_bbox = buoy["full_bbox"]
        indicator_roi = buoy["indicator_roi"]
        indicator_bbox_global = buoy["indicator_bbox"]
        state = buoy["indicator_state"]
        
        rx, ry, rw, rh = indicator_roi
        
        # 1) White blob region (cyan)
        wx, wy, ww, wh = white_blob_bbox
        cv2.rectangle(annotated, (wx, wy), (wx + ww, wy + wh), (255, 255, 0), 2)
        
        # 2) Diamond (green)
        cv2.rectangle(annotated, (dx, dy), (dx + dw, dy + dh), (0, 255, 0), 2)
        diamond_conf = diamond.get("confidence", 0.0)
        cv2.putText(
            annotated,
            f"D:{diamond_conf:.2f}",
            (dx, max(10, dy - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )
        
        # 3) Indicator ROI (orange)
        cv2.rectangle(annotated, (rx, ry), (rx + rw, ry + rh), (0, 165, 255), 2)
        
        # 4) Indicator detection (blue)
        if indicator_bbox_global:
            ix, iy, iw, ih = indicator_bbox_global
            cv2.rectangle(annotated, (ix, iy), (ix + iw, iy + ih), (255, 0, 0), 2)
        
        # 5) Full buoy bounding box (magenta, thick)
        fx, fy, fw, fh = full_bbox
        cv2.rectangle(annotated, (fx, fy), (fx + fw, fy + fh), (255, 0, 255), 3)
        
        # 6) Label with state and confidence
        label = f"{state.upper()} conf:{buoy['buoy_confidence']:.2f}"
        color = (0, 0, 255) if state == "red" else (0, 255, 0) if state == "green" else (255, 255, 255)
        cv2.putText(
            annotated,
            label,
            (fx, max(20, fy - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
            cv2.LINE_AA,
        )
    
    return annotated, {"buoys": final_buoys}


def main():
    parser = argparse.ArgumentParser(
        description="Colour indicator buoy detector: diamonds + white buoy + red/green indicator."
    )
    parser.add_argument("image", nargs="?", help="Input image path (PNG/JPEG).")
    parser.add_argument("--out", default="", help="Output image path.")
    parser.add_argument(
        "--conf-threshold",
        type=float,
        default=DEFAULT_CONF_THRESHOLD,
        help="Minimum confidence for diamonds (0–1); default 0.7",
    )
    parser.add_argument(
        "--roi-conf-threshold",
        type=float,
        default=0.6,
        help=(
            "Minimum confidence for diamonds contributing to the indicator ROI (0–1); "
            "default 0.6"
        ),
    )
    parser.add_argument(
        "--max-black",
        type=float,
        default=100.0,
        help="Max mean brightness considered black (0–255).",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not open a window; just save/print results.",
    )
    args = parser.parse_args()

    script_dir = Path(__file__).resolve().parent
    if args.image:
        image_path = Path(args.image)
        if not image_path.is_absolute():
            image_path = script_dir / image_path
    else:
        # Autoselect first image in this folder if none provided
        candidates = []
        for ext in ("*.png", "*.jpg", "*.jpeg", "*.bmp"):
            candidates.extend(script_dir.glob(ext))
        if not candidates:
            print("No image specified and no image found in script directory.")
            return 1
        image_path = sorted(candidates)[0]
        print(f"Using image: {image_path.name}")

    img = cv2.imread(str(image_path))
    if img is None:
        print(f"Failed to load image: {image_path}")
        return 1

    annotated, info = classify_colour_indicator_buoy(
        img,
        conf_threshold=args.conf_threshold,
        max_black_brightness=args.max_black,
        roi_conf_threshold=args.roi_conf_threshold,
    )

    buoys = info.get("buoys", [])
    print(f"Buoys detected: {len(buoys)}")
    for i, buoy in enumerate(buoys):
        diamond = buoy["diamond"]
        print(
            f"  [{i+1}] Buoy confidence: {buoy['buoy_confidence']:.3f} "
            f"(diamond: {diamond.get('confidence', 0.0):.3f}, white_blob: {buoy['white_blob_score']:.3f})"
        )
        print(
            f"      Diamond: center={diamond['center']} bbox={diamond['bbox']}"
        )
        print(
            f"      Indicator: state={buoy['indicator_state']} conf={buoy['indicator_conf']:.3f}"
        )
        print(
            f"      Full bbox: {buoy['full_bbox']}"
        )

    if args.out:
        out_path = Path(args.out)
        if not out_path.is_absolute():
            out_path = script_dir / out_path
        cv2.imwrite(str(out_path), annotated)
        print(f"Saved: {out_path}")

    if not args.no_show:
        cv2.imshow("Colour Indicator Buoy", annotated)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

