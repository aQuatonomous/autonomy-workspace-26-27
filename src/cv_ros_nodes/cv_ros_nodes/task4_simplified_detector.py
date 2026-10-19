"""
Task 4 Simplified Detector - ROS-compatible module.

Wraps the simplified blob detection and shape verification pipeline so it can be
used directly from the task4_supply_processor ROS2 node without any external
file-system dependencies.

Public API:
    SimplifiedTask4Detector.process_frame(bgr_image) -> List[Dict]

Each returned dict:
    {
        'type':         'yellow_supply_drop' | 'black_supply_drop',
        'vessel_bbox':  [x1, y1, x2, y2],
        'confidence':   float,
        'has_shape':    bool,
        'source':       'task4',
    }
Only detections with confidence >= SimplifiedConfig.MIN_CONFIDENCE_THRESHOLD are returned.
"""

import cv2
import numpy as np
from typing import List, Tuple, Dict, Optional


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

class SimplifiedConfig:
    """Configuration for the simplified pipeline"""

    # Color detection ranges
    YELLOW_HSV_LOWER = np.array([18, 80, 100])
    YELLOW_HSV_UPPER = np.array([35, 255, 255])

    BLACK_HSV_V_MAX = 70    # Value threshold for black detection
    BLACK_LAB_L_MAX = 40    # L channel threshold in LAB space

    # Size constraints
    MIN_BLOB_AREA = 2000
    MIN_BLOB_WIDTH = 40
    MIN_BLOB_HEIGHT = 20

    # Aspect ratio constraints - apply to ALL vessels (must be decently flatter than square)
    MAX_VESSEL_ASPECT_RATIO = 0.7  # height/width must be < 0.7 (clearly flatter than square)

    # Spatial constraints
    LOWER_FRAME_THRESHOLD = 0.4  # Only consider blobs in lower 60% (y > 40% of height)

    # Blob merging
    MERGE_DISTANCE_THRESHOLD = 100  # Pixels - merge blobs closer than this
    MERGE_VERTICAL_TOLERANCE = 50   # Allow some vertical offset when merging

    # Shape detection for verification (stricter requirements)
    SHAPE_SEARCH_HEIGHT = 220  # Pixels above blob to search for shapes
    SHAPE_SEARCH_H_PAD = 0.15  # Expand search region 15% each side
    TRIANGLE_MIN_AREA = 400    # Minimum area for black triangle
    CROSS_MIN_AREA = 1000      # Minimum area for black cross
    TRIANGLE_SIDE_SIMILARITY_THRESHOLD = 0.65  # At least two sides within 65% length ratio
    CROSS_ARM_LENGTH_RATIO = 0.3   # Cross arms must be at least 30% of total size
    CROSS_ARM_BALANCE_MAX_RATIO = 2.5  # Longer arm at most 2.5x shorter

    # Shape color constraints
    SHAPE_BLACK_HSV_V_MAX = 60
    SHAPE_BLACK_LAB_L_MAX = 35

    # Overlap resolution
    OVERLAP_THRESHOLD = 0.3  # If IoU > 30%, consider overlapping

    # Confidence threshold - detections below this are discarded
    MIN_CONFIDENCE_THRESHOLD = 0.65

    # Morphological operations
    MORPH_KERNEL_SIZE = 5


# ---------------------------------------------------------------------------
# Blob detection
# ---------------------------------------------------------------------------

class _BlobDetector:
    def __init__(self, config: SimplifiedConfig):
        self.config = config
        self.morph_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (config.MORPH_KERNEL_SIZE, config.MORPH_KERNEL_SIZE)
        )

    def detect_yellow_blobs(self, image: np.ndarray) -> List[Tuple[int, int, int, int]]:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.config.YELLOW_HSV_LOWER, self.config.YELLOW_HSV_UPPER)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morph_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.morph_kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return self._contours_to_bboxes(contours)

    def detect_black_blobs(self, image: np.ndarray) -> List[Tuple[int, int, int, int]]:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        hsv_mask = hsv[:, :, 2] < self.config.BLACK_HSV_V_MAX
        lab_mask = lab[:, :, 0] < self.config.BLACK_LAB_L_MAX
        mask = (hsv_mask & lab_mask).astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morph_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.morph_kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return self._contours_to_bboxes(contours)

    def _contours_to_bboxes(self, contours) -> List[Tuple[int, int, int, int]]:
        bboxes = []
        for contour in contours:
            if cv2.contourArea(contour) < self.config.MIN_BLOB_AREA:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if w < self.config.MIN_BLOB_WIDTH or h < self.config.MIN_BLOB_HEIGHT:
                continue
            aspect_ratio = h / w if w > 0 else float('inf')
            if aspect_ratio > self.config.MAX_VESSEL_ASPECT_RATIO:
                continue
            bboxes.append((x, y, x + w, y + h))
        return bboxes


# ---------------------------------------------------------------------------
# Spatial filter
# ---------------------------------------------------------------------------

class _SpatialFilter:
    def __init__(self, config: SimplifiedConfig):
        self.config = config

    def filter_to_lower_frame(self, bboxes: List[Tuple[int, int, int, int]],
                              image_height: int) -> List[Tuple[int, int, int, int]]:
        threshold_y = int(image_height * self.config.LOWER_FRAME_THRESHOLD)
        return [(x1, y1, x2, y2) for x1, y1, x2, y2 in bboxes
                if (y1 + y2) // 2 > threshold_y]


# ---------------------------------------------------------------------------
# Blob merger / overlap resolver
# ---------------------------------------------------------------------------

class _BlobMerger:
    def __init__(self, config: SimplifiedConfig):
        self.config = config

    def merge_nearby_blobs(self, bboxes: List[Tuple[int, int, int, int]]) -> List[Tuple[int, int, int, int]]:
        if len(bboxes) <= 1:
            return bboxes
        sorted_bboxes = sorted(bboxes, key=lambda b: b[0])
        merged = []
        group = [sorted_bboxes[0]]
        for i in range(1, len(sorted_bboxes)):
            cur = sorted_bboxes[i]
            last = group[-1]
            h_dist = cur[0] - last[2]
            if h_dist <= self.config.MERGE_DISTANCE_THRESHOLD and self._vert_prox(last, cur):
                group.append(cur)
            else:
                merged.append(self._merge_group(group))
                group = [cur]
        merged.append(self._merge_group(group))
        return merged

    def _vert_prox(self, b1, b2) -> bool:
        overlap = max(0, min(b1[3], b2[3]) - max(b1[1], b2[1]))
        if overlap > 0:
            return True
        gap = min(abs(b1[1] - b2[3]), abs(b2[1] - b1[3]))
        return gap <= self.config.MERGE_VERTICAL_TOLERANCE

    def _merge_group(self, bboxes):
        return (min(b[0] for b in bboxes), min(b[1] for b in bboxes),
                max(b[2] for b in bboxes), max(b[3] for b in bboxes))

    def resolve_overlaps(
        self, bboxes_with_color: List[Tuple[Tuple[int, int, int, int], str]]
    ) -> List[Tuple[Tuple[int, int, int, int], str]]:
        if len(bboxes_with_color) <= 1:
            return bboxes_with_color
        items = [((x1, y1, x2, y2), color, (x2-x1)*(y2-y1))
                 for (x1, y1, x2, y2), color in bboxes_with_color]
        items.sort(key=lambda x: x[2], reverse=True)
        kept: List[Tuple[Tuple[int, int, int, int], str]] = []
        for bbox, color, _ in items:
            if not any(self._iou(bbox, kb) > self.config.OVERLAP_THRESHOLD for kb, _ in kept):
                kept.append((bbox, color))
        return kept

    def _iou(self, b1, b2) -> float:
        xi1, yi1 = max(b1[0], b2[0]), max(b1[1], b2[1])
        xi2, yi2 = min(b1[2], b2[2]), min(b1[3], b2[3])
        if xi2 <= xi1 or yi2 <= yi1:
            return 0.0
        inter = (xi2 - xi1) * (yi2 - yi1)
        a1 = (b1[2]-b1[0]) * (b1[3]-b1[1])
        a2 = (b2[2]-b2[0]) * (b2[3]-b2[1])
        union = a1 + a2 - inter
        return inter / union if union > 0 else 0.0


# ---------------------------------------------------------------------------
# Shape verifier
# ---------------------------------------------------------------------------

class _ShapeVerifier:
    def __init__(self, config: SimplifiedConfig):
        self.config = config

    def detect_triangle_above_blob(self, image: np.ndarray,
                                   bbox: Tuple[int, int, int, int]) -> Tuple[bool, float]:
        x1, y1, x2, _ = bbox
        w = x2 - x1
        pad = int(w * self.config.SHAPE_SEARCH_H_PAD)
        sx1 = max(0, x1 - pad)
        sx2 = min(image.shape[1], x2 + pad)
        sy1 = max(0, y1 - self.config.SHAPE_SEARCH_HEIGHT)
        region = image[sy1:y1, sx1:sx2]
        if region.size == 0:
            return False, 0.0
        black_mask = self._black_mask(region)
        contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour) < self.config.TRIANGLE_MIN_AREA:
                continue
            for eps in (0.02, 0.04, 0.06, 0.08):
                approx = cv2.approxPolyDP(contour, eps * cv2.arcLength(contour, True), True)
                if len(approx) == 3 and self._valid_triangle(approx):
                    return True, min(1.0, cv2.contourArea(contour) / 2000.0)
        return False, 0.0

    def detect_cross_above_blob(self, image: np.ndarray,
                                bbox: Tuple[int, int, int, int]) -> Tuple[bool, float]:
        x1, y1, x2, _ = bbox
        w = x2 - x1
        pad = int(w * self.config.SHAPE_SEARCH_H_PAD)
        sx1 = max(0, x1 - pad)
        sx2 = min(image.shape[1], x2 + pad)
        sy1 = max(0, y1 - self.config.SHAPE_SEARCH_HEIGHT)
        region = image[sy1:y1, sx1:sx2]
        if region.size == 0:
            return False, 0.0
        black_mask = self._black_mask(region)

        rw = x2 - x1
        rh = y1 - sy1
        h_kern = cv2.getStructuringElement(cv2.MORPH_RECT, (max(7, rw // 6), 3))
        v_kern = cv2.getStructuringElement(cv2.MORPH_RECT, (3, max(7, rh // 6)))
        h_lines = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, h_kern)
        v_lines = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, v_kern)

        if cv2.countNonZero(h_lines) < 50 or cv2.countNonZero(v_lines) < 50:
            return False, 0.0
        if cv2.countNonZero(cv2.bitwise_and(h_lines, v_lines)) < 20:
            return False, 0.0

        cross = cv2.bitwise_or(h_lines, v_lines)
        cross_px = cv2.countNonZero(cross)
        total_px = black_mask.shape[0] * black_mask.shape[1]
        ratio = cross_px / total_px

        if (cross_px > self.config.CROSS_MIN_AREA and 0.05 < ratio < 0.8
                and self._valid_cross(h_lines, v_lines)):
            return True, min(1.0, cross_px / 3000.0)
        return False, 0.0

    # ------------------------------------------------------------------

    def _black_mask(self, region: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)
        lab = cv2.cvtColor(region, cv2.COLOR_BGR2LAB)
        mask = ((hsv[:, :, 2] < self.config.SHAPE_BLACK_HSV_V_MAX) &
                (lab[:, :, 0] < self.config.SHAPE_BLACK_LAB_L_MAX)).astype(np.uint8) * 255
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        return mask

    def _valid_triangle(self, approx: np.ndarray) -> bool:
        pts = approx.reshape(-1, 2)
        sides = sorted([
            np.linalg.norm(pts[0] - pts[1]),
            np.linalg.norm(pts[1] - pts[2]),
            np.linalg.norm(pts[2] - pts[0]),
        ])
        r1 = sides[0] / sides[1] if sides[1] > 0 else 0
        r2 = sides[1] / sides[2] if sides[2] > 0 else 0
        thr = self.config.TRIANGLE_SIDE_SIMILARITY_THRESHOLD
        return r1 > thr or r2 > thr

    def _valid_cross(self, h_lines: np.ndarray, v_lines: np.ndarray) -> bool:
        hc = max(cv2.findContours(h_lines, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0],
                 key=cv2.contourArea, default=None)
        vc = max(cv2.findContours(v_lines, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0],
                 key=cv2.contourArea, default=None)
        if hc is None or vc is None:
            return False
        _, _, hw, _ = cv2.boundingRect(hc)
        _, _, _, vh = cv2.boundingRect(vc)
        rw = h_lines.shape[1] or 1
        rh = h_lines.shape[0] or 1
        if hw / rw <= self.config.CROSS_ARM_LENGTH_RATIO:
            return False
        if vh / rh <= self.config.CROSS_ARM_LENGTH_RATIO:
            return False
        longer = max(hw, vh)
        shorter = min(hw, vh)
        return shorter > 0 and (longer / shorter) <= self.config.CROSS_ARM_BALANCE_MAX_RATIO


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

class SimplifiedTask4Detector:
    """
    ROS-friendly wrapper around the simplified Task 4 detection pipeline.

    Usage::

        detector = SimplifiedTask4Detector()
        detections = detector.process_frame(bgr_image)
    """

    def __init__(self, config: Optional[SimplifiedConfig] = None):
        self.config = config or SimplifiedConfig()
        self._blob_detector = _BlobDetector(self.config)
        self._spatial_filter = _SpatialFilter(self.config)
        self._blob_merger = _BlobMerger(self.config)
        self._shape_verifier = _ShapeVerifier(self.config)

    def process_frame(self, image: np.ndarray) -> List[Dict]:
        """
        Run the full detection pipeline on a single BGR frame.

        Returns a list of detections (dicts) with confidence >=
        SimplifiedConfig.MIN_CONFIDENCE_THRESHOLD.
        """
        h = image.shape[0]

        # Step 1 – blob detection
        yellow_bboxes = self._blob_detector.detect_yellow_blobs(image)
        black_bboxes = self._blob_detector.detect_black_blobs(image)

        # Step 2 – spatial filter (lower 60%)
        yellow_bboxes = self._spatial_filter.filter_to_lower_frame(yellow_bboxes, h)
        black_bboxes = self._spatial_filter.filter_to_lower_frame(black_bboxes, h)

        # Step 3 – merge nearby blobs of the same colour
        yellow_bboxes = self._blob_merger.merge_nearby_blobs(yellow_bboxes)
        black_bboxes = self._blob_merger.merge_nearby_blobs(black_bboxes)

        # Step 4 – resolve overlaps across both colours (keep the bigger bbox)
        bwc = [(b, 'yellow') for b in yellow_bboxes] + [(b, 'black') for b in black_bboxes]
        resolved = self._blob_merger.resolve_overlaps(bwc)
        yellow_bboxes = [b for b, c in resolved if c == 'yellow']
        black_bboxes = [b for b, c in resolved if c == 'black']

        # Step 5 – shape verification + confidence scoring
        detections: List[Dict] = []

        for bbox in yellow_bboxes:
            has_tri, tri_conf = self._shape_verifier.detect_triangle_above_blob(image, bbox)
            conf = 0.7 + (0.3 * tri_conf if has_tri else 0.0)
            if conf >= self.config.MIN_CONFIDENCE_THRESHOLD:
                x1, y1, x2, y2 = bbox
                detections.append({
                    'type': 'yellow_supply_drop',
                    'vessel_bbox': [x1, y1, x2, y2],
                    'confidence': conf,
                    'has_shape': has_tri,
                    'source': 'task4',
                })

        for bbox in black_bboxes:
            has_cross, cross_conf = self._shape_verifier.detect_cross_above_blob(image, bbox)
            conf = 0.7 + (0.3 * cross_conf if has_cross else 0.0)
            if conf >= self.config.MIN_CONFIDENCE_THRESHOLD:
                x1, y1, x2, y2 = bbox
                detections.append({
                    'type': 'black_supply_drop',
                    'vessel_bbox': [x1, y1, x2, y2],
                    'confidence': conf,
                    'has_shape': has_cross,
                    'source': 'task4',
                })

        return detections
