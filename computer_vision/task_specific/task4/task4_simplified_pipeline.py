#!/usr/bin/env python3
"""
Task 4 Supply Drop Detection - Simplified Pipeline

A much simpler approach focusing on:
1. Yellow and black blob detection using color masks
2. Filtering to lower 60% of frame
3. Aspect ratio filtering (exclude tall black blobs)
4. Merging nearby blobs of same color
5. Shape verification (triangles above yellow, crosses above black)

This replaces the overly complex previous pipeline with a straightforward approach.
"""

import cv2
import numpy as np
import os
from pathlib import Path
import argparse
from typing import List, Tuple, Dict, Optional
import math

class SimplifiedConfig:
    """Configuration for the simplified pipeline"""
    
    # Color detection ranges
    YELLOW_HSV_LOWER = np.array([18, 80, 100])
    YELLOW_HSV_UPPER = np.array([35, 255, 255])
    
    BLACK_HSV_V_MAX = 70  # Value threshold for black detection
    BLACK_LAB_L_MAX = 40  # L channel threshold in LAB space
    
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
    SHAPE_SEARCH_H_PAD = 0.15  # Expand search region 15% each side (catch off-center signs)
    TRIANGLE_MIN_AREA = 400    # Minimum area for black triangle
    CROSS_MIN_AREA = 1000      # Minimum area for black cross
    TRIANGLE_SIDE_SIMILARITY_THRESHOLD = 0.65  # At least two sides within 65% length ratio
    CROSS_ARM_LENGTH_RATIO = 0.3  # Cross arms must be at least 30% of total size
    CROSS_ARM_BALANCE_MAX_RATIO = 2.5  # Longer arm at most 2.5x shorter (more like a cross)
    
    # Shape color constraints
    SHAPE_BLACK_HSV_V_MAX = 60    # Shapes must be darker than this
    SHAPE_BLACK_LAB_L_MAX = 35    # Shapes must be darker than this in LAB
    
    # Overlap resolution
    OVERLAP_THRESHOLD = 0.3  # If IoU > 30%, consider overlapping
    
    # Confidence threshold - detections below this are discarded
    MIN_CONFIDENCE_THRESHOLD = 0.65
    
    # Morphological operations
    MORPH_KERNEL_SIZE = 5
    
    # Debug settings
    DEBUG = True
    SAVE_DEBUG_IMAGES = True

class BlobDetector:
    """Handles blob detection for yellow and black objects"""
    
    def __init__(self, config: SimplifiedConfig):
        self.config = config
        self.morph_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, 
            (config.MORPH_KERNEL_SIZE, config.MORPH_KERNEL_SIZE)
        )
    
    def detect_yellow_blobs(self, image: np.ndarray) -> Tuple[List[Tuple[int, int, int, int]], np.ndarray]:
        """Detect yellow blobs and return bounding boxes and mask"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Create yellow mask
        mask = cv2.inRange(hsv, self.config.YELLOW_HSV_LOWER, self.config.YELLOW_HSV_UPPER)
        
        # Clean up mask
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morph_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.morph_kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        bboxes = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.config.MIN_BLOB_AREA:
                continue
                
            x, y, w, h = cv2.boundingRect(contour)
            
            # Size filtering
            if w < self.config.MIN_BLOB_WIDTH or h < self.config.MIN_BLOB_HEIGHT:
                continue
            
            # Aspect ratio filtering - must be flatter than square
            aspect_ratio = h / w if w > 0 else float('inf')
            if aspect_ratio > self.config.MAX_VESSEL_ASPECT_RATIO:
                continue
                
            bboxes.append((x, y, x + w, y + h))
        
        return bboxes, mask
    
    def detect_black_blobs(self, image: np.ndarray) -> Tuple[List[Tuple[int, int, int, int]], np.ndarray]:
        """Detect black blobs and return bounding boxes and mask"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        
        # Create black mask using both HSV and LAB
        hsv_mask = hsv[:, :, 2] < self.config.BLACK_HSV_V_MAX  # Low value in HSV
        lab_mask = lab[:, :, 0] < self.config.BLACK_LAB_L_MAX  # Low L in LAB
        
        # Combine masks
        mask = (hsv_mask & lab_mask).astype(np.uint8) * 255
        
        # Clean up mask
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morph_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.morph_kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        bboxes = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.config.MIN_BLOB_AREA:
                continue
                
            x, y, w, h = cv2.boundingRect(contour)
            
            # Size filtering
            if w < self.config.MIN_BLOB_WIDTH or h < self.config.MIN_BLOB_HEIGHT:
                continue
            
            # Aspect ratio filtering - must be flatter than square (same as yellow)
            aspect_ratio = h / w if w > 0 else float('inf')
            if aspect_ratio > self.config.MAX_VESSEL_ASPECT_RATIO:
                continue
                
            bboxes.append((x, y, x + w, y + h))
        
        return bboxes, mask

class SpatialFilter:
    """Handles spatial filtering of detected blobs"""
    
    def __init__(self, config: SimplifiedConfig):
        self.config = config
    
    def filter_to_lower_frame(self, bboxes: List[Tuple[int, int, int, int]], 
                             image_height: int) -> List[Tuple[int, int, int, int]]:
        """Filter bboxes to only include those in lower 60% of frame"""
        threshold_y = int(image_height * self.config.LOWER_FRAME_THRESHOLD)
        
        filtered_bboxes = []
        for x1, y1, x2, y2 in bboxes:
            # Check if center of bbox is in lower 60%
            center_y = (y1 + y2) // 2
            if center_y > threshold_y:
                filtered_bboxes.append((x1, y1, x2, y2))
        
        return filtered_bboxes

class BlobMerger:
    """Handles merging of nearby blobs of the same color and overlap resolution"""
    
    def __init__(self, config: SimplifiedConfig):
        self.config = config
    
    def merge_nearby_blobs(self, bboxes: List[Tuple[int, int, int, int]]) -> List[Tuple[int, int, int, int]]:
        """Merge blobs that are close to each other horizontally"""
        if len(bboxes) <= 1:
            return bboxes
        
        # Sort by x coordinate
        sorted_bboxes = sorted(bboxes, key=lambda bbox: bbox[0])
        merged = []
        current_group = [sorted_bboxes[0]]
        
        for i in range(1, len(sorted_bboxes)):
            current_bbox = sorted_bboxes[i]
            last_in_group = current_group[-1]
            
            # Check horizontal distance
            horizontal_distance = current_bbox[0] - last_in_group[2]  # x1_current - x2_last
            
            # Check vertical overlap/proximity
            vertical_overlap = self._check_vertical_proximity(last_in_group, current_bbox)
            
            if (horizontal_distance <= self.config.MERGE_DISTANCE_THRESHOLD and 
                vertical_overlap):
                current_group.append(current_bbox)
            else:
                # Merge current group and start new group
                merged.append(self._merge_bbox_group(current_group))
                current_group = [current_bbox]
        
        # Don't forget the last group
        merged.append(self._merge_bbox_group(current_group))
        
        return merged
    
    def _check_vertical_proximity(self, bbox1: Tuple[int, int, int, int], 
                                 bbox2: Tuple[int, int, int, int]) -> bool:
        """Check if two bboxes are vertically close enough to merge"""
        _, y1_1, _, y2_1 = bbox1
        _, y1_2, _, y2_2 = bbox2
        
        # Check if they overlap vertically or are close
        overlap = max(0, min(y2_1, y2_2) - max(y1_1, y1_2))
        if overlap > 0:
            return True
        
        # Check if they're within tolerance
        gap = min(abs(y1_1 - y2_2), abs(y1_2 - y2_1))
        return gap <= self.config.MERGE_VERTICAL_TOLERANCE
    
    def _merge_bbox_group(self, bboxes: List[Tuple[int, int, int, int]]) -> Tuple[int, int, int, int]:
        """Merge a group of bboxes into one"""
        if len(bboxes) == 1:
            return bboxes[0]
        
        x1_min = min(bbox[0] for bbox in bboxes)
        y1_min = min(bbox[1] for bbox in bboxes)
        x2_max = max(bbox[2] for bbox in bboxes)
        y2_max = max(bbox[3] for bbox in bboxes)
        
        return (x1_min, y1_min, x2_max, y2_max)
    
    def resolve_overlaps(
        self, bboxes_with_color: List[Tuple[Tuple[int, int, int, int], str]]
    ) -> List[Tuple[Tuple[int, int, int, int], str]]:
        """Resolve overlapping bounding boxes by keeping the larger one; preserve color."""
        if len(bboxes_with_color) <= 1:
            return bboxes_with_color
        
        # (bbox, color, area) for sorting
        items = []
        for (x1, y1, x2, y2), color in bboxes_with_color:
            area = (x2 - x1) * (y2 - y1)
            items.append(((x1, y1, x2, y2), color, area))
        
        # Sort by area (largest first) so we keep the bigger one when overlapping
        items.sort(key=lambda x: x[2], reverse=True)
        
        kept = []
        for bbox, color, _ in items:
            overlaps = False
            for kept_bbox, _ in kept:
                if self._calculate_iou(bbox, kept_bbox) > self.config.OVERLAP_THRESHOLD:
                    overlaps = True
                    break
            if not overlaps:
                kept.append((bbox, color))
        
        return kept
    
    def _calculate_iou(self, bbox1: Tuple[int, int, int, int], 
                      bbox2: Tuple[int, int, int, int]) -> float:
        """Calculate Intersection over Union (IoU) of two bounding boxes"""
        x1_1, y1_1, x2_1, y2_1 = bbox1
        x1_2, y1_2, x2_2, y2_2 = bbox2
        
        # Calculate intersection
        x1_i = max(x1_1, x1_2)
        y1_i = max(y1_1, y1_2)
        x2_i = min(x2_1, x2_2)
        y2_i = min(y2_1, y2_2)
        
        if x2_i <= x1_i or y2_i <= y1_i:
            return 0.0
        
        intersection = (x2_i - x1_i) * (y2_i - y1_i)
        
        # Calculate union
        area1 = (x2_1 - x1_1) * (y2_1 - y1_1)
        area2 = (x2_2 - x1_2) * (y2_2 - y1_2)
        union = area1 + area2 - intersection
        
        return intersection / union if union > 0 else 0.0

class ShapeVerifier:
    """Handles shape detection above blobs for verification"""
    
    def __init__(self, config: SimplifiedConfig):
        self.config = config
    
    def detect_triangle_above_blob(self, image: np.ndarray, 
                                  bbox: Tuple[int, int, int, int]) -> Tuple[bool, float]:
        """Detect if there's a BLACK triangle above the given blob. Triangle must be black
        and have at least two sides of similar length (very triangular)."""
        x1, y1, x2, y2 = bbox
        w = x2 - x1
        pad = int(w * self.config.SHAPE_SEARCH_H_PAD)
        # Expand search region horizontally so we don't miss off-center signs
        search_x1 = max(0, x1 - pad)
        search_x2 = min(image.shape[1], x2 + pad)
        search_y1 = max(0, y1 - self.config.SHAPE_SEARCH_HEIGHT)
        search_y2 = y1
        search_region = image[search_y1:search_y2, search_x1:search_x2]
        
        if search_region.size == 0:
            return False, 0.0
        
        # Only consider black regions (shapes must be black)
        black_mask = self._create_black_shape_mask(search_region)
        
        contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.config.TRIANGLE_MIN_AREA:
                continue
            
            # Try several approximation levels to get a 3-vertex contour (very triangular)
            for eps_ratio in (0.02, 0.04, 0.06, 0.08):
                epsilon = eps_ratio * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                if len(approx) == 3 and self._is_valid_triangle(approx):
                    confidence = min(1.0, area / 2000.0)
                    return True, confidence
        
        return False, 0.0
    
    def _create_black_shape_mask(self, region: np.ndarray) -> np.ndarray:
        """Create a mask for black shapes using both HSV and LAB"""
        hsv = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)
        lab = cv2.cvtColor(region, cv2.COLOR_BGR2LAB)
        
        # Stricter black detection for shapes
        hsv_mask = hsv[:, :, 2] < self.config.SHAPE_BLACK_HSV_V_MAX
        lab_mask = lab[:, :, 0] < self.config.SHAPE_BLACK_LAB_L_MAX
        
        # Combine masks
        mask = (hsv_mask & lab_mask).astype(np.uint8) * 255
        
        # Clean up with morphological operations
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        return mask
    
    def _is_valid_triangle(self, approx: np.ndarray) -> bool:
        """Check if the 3-point polygon is a valid triangle with similar side lengths"""
        if len(approx) != 3:
            return False
        
        # Calculate side lengths
        points = approx.reshape(-1, 2)
        side1 = np.linalg.norm(points[0] - points[1])
        side2 = np.linalg.norm(points[1] - points[2])
        side3 = np.linalg.norm(points[2] - points[0])
        
        sides = sorted([side1, side2, side3])
        
        # Check if at least two sides are similar (within threshold)
        ratio1 = sides[0] / sides[1] if sides[1] > 0 else 0
        ratio2 = sides[1] / sides[2] if sides[2] > 0 else 0
        
        # At least one pair of sides should be similar
        return (ratio1 > self.config.TRIANGLE_SIDE_SIMILARITY_THRESHOLD or 
                ratio2 > self.config.TRIANGLE_SIDE_SIMILARITY_THRESHOLD)
    
    def detect_cross_above_blob(self, image: np.ndarray, 
                               bbox: Tuple[int, int, int, int]) -> Tuple[bool, float]:
        """Detect if there's a BLACK cross above the given blob. Cross must be black
        and consist of vertical and horizontal bars intersecting (more like a perfect cross)."""
        x1, y1, x2, y2 = bbox
        w = x2 - x1
        pad = int(w * self.config.SHAPE_SEARCH_H_PAD)
        search_x1 = max(0, x1 - pad)
        search_x2 = min(image.shape[1], x2 + pad)
        search_y1 = max(0, y1 - self.config.SHAPE_SEARCH_HEIGHT)
        search_y2 = y1
        search_region = image[search_y1:search_y2, search_x1:search_x2]
        
        if search_region.size == 0:
            return False, 0.0
        
        # Only black regions (shape must be black)
        black_mask = self._create_black_shape_mask(search_region)
        
        # Use morphological operations to detect cross-like structures
        # Horizontal line detection (adaptive size based on region)
        region_width = x2 - x1
        region_height = search_y2 - search_y1
        
        h_kernel_width = max(7, region_width // 6)
        v_kernel_height = max(7, region_height // 6)
        
        horizontal_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (h_kernel_width, 3))
        horizontal_lines = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, horizontal_kernel)
        
        # Vertical line detection
        vertical_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, v_kernel_height))
        vertical_lines = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, vertical_kernel)
        
        # Check if both horizontal and vertical components exist
        h_pixels = cv2.countNonZero(horizontal_lines)
        v_pixels = cv2.countNonZero(vertical_lines)
        
        if h_pixels < 50 or v_pixels < 50:  # Need both components
            return False, 0.0
        
        # Combine horizontal and vertical lines
        cross_mask = cv2.bitwise_or(horizontal_lines, vertical_lines)
        
        # Find intersection points (where lines cross)
        intersection = cv2.bitwise_and(horizontal_lines, vertical_lines)
        intersection_pixels = cv2.countNonZero(intersection)
        
        # Must have a clear intersection
        if intersection_pixels < 20:
            return False, 0.0
        
        # Check if we have enough cross-like structure
        cross_pixels = cv2.countNonZero(cross_mask)
        total_region_pixels = black_mask.shape[0] * black_mask.shape[1]
        
        # Cross should be substantial but not fill the entire region
        cross_ratio = cross_pixels / total_region_pixels
        
        if (cross_pixels > self.config.CROSS_MIN_AREA and 
            cross_ratio > 0.05 and cross_ratio < 0.8):  # Between 5% and 80% of region
            
            # Additional validation: check arm length ratio
            if self._validate_cross_proportions(horizontal_lines, vertical_lines):
                confidence = min(1.0, cross_pixels / 3000.0)
                return True, confidence
        
        return False, 0.0
    
    def _validate_cross_proportions(self, h_lines: np.ndarray, v_lines: np.ndarray) -> bool:
        """Validate that the cross has proper proportions (more like a perfect cross:
        both arms substantial and not too imbalanced)."""
        h_contours, _ = cv2.findContours(h_lines, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        v_contours, _ = cv2.findContours(v_lines, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not h_contours or not v_contours:
            return False
        
        h_contour = max(h_contours, key=cv2.contourArea)
        v_contour = max(v_contours, key=cv2.contourArea)
        h_x, h_y, h_w, h_h = cv2.boundingRect(h_contour)
        v_x, v_y, v_w, v_h = cv2.boundingRect(v_contour)
        
        region_width = h_lines.shape[1]
        region_height = h_lines.shape[0]
        h_arm_ratio = h_w / region_width if region_width else 0
        v_arm_ratio = v_h / region_height if region_height else 0
        
        # Both arms at least 30% of their dimension
        if h_arm_ratio <= self.config.CROSS_ARM_LENGTH_RATIO or v_arm_ratio <= self.config.CROSS_ARM_LENGTH_RATIO:
            return False
        
        # Arms should not be too imbalanced (more like a proper cross)
        # Use length of arms: horizontal arm ~ h_w, vertical arm ~ v_h
        longer = max(h_w, v_h)
        shorter = min(h_w, v_h)
        if shorter <= 0:
            return False
        return (longer / shorter) <= self.config.CROSS_ARM_BALANCE_MAX_RATIO

class DebugVisualizer:
    """Handles debug visualization and output saving"""
    
    def __init__(self, config: SimplifiedConfig, output_dir: str):
        self.config = config
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
    
    def save_step(self, image: np.ndarray, step_name: str, image_name: str):
        """Save a debug image for a specific step"""
        if not self.config.SAVE_DEBUG_IMAGES:
            return
        
        filename = f"{step_name}_{image_name}.png"
        filepath = self.output_dir / filename
        cv2.imwrite(str(filepath), image)
    
    def visualize_blobs(self, image: np.ndarray, yellow_bboxes: List, black_bboxes: List, 
                       step_name: str, image_name: str) -> np.ndarray:
        """Visualize detected blobs on image"""
        vis_image = image.copy()
        
        # Draw yellow bboxes
        for x1, y1, x2, y2 in yellow_bboxes:
            cv2.rectangle(vis_image, (x1, y1), (x2, y2), (0, 255, 255), 2)
            cv2.putText(vis_image, "YELLOW", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # Draw black bboxes
        for x1, y1, x2, y2 in black_bboxes:
            cv2.rectangle(vis_image, (x1, y1), (x2, y2), (255, 255, 255), 2)
            cv2.putText(vis_image, "BLACK", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        self.save_step(vis_image, step_name, image_name)
        return vis_image
    
    def visualize_final_results(self, image: np.ndarray, detections: List[Dict], 
                              image_name: str) -> np.ndarray:
        """Visualize final detection results with confidence scores"""
        vis_image = image.copy()
        
        for detection in detections:
            x1, y1, x2, y2 = detection['bbox']
            color = detection['color']
            confidence = detection['confidence']
            has_shape = detection['has_shape']
            
            # Choose visualization color
            if color == 'yellow':
                draw_color = (0, 255, 255)
            else:  # black
                draw_color = (255, 255, 255)
            
            # Draw bounding box
            thickness = 3 if has_shape else 2
            cv2.rectangle(vis_image, (x1, y1), (x2, y2), draw_color, thickness)
            
            # Draw label
            label = f"{color.upper()}"
            if has_shape:
                shape_type = "TRI" if color == 'yellow' else "CROSS"
                label += f" + {shape_type}"
            label += f" ({confidence:.2f})"
            
            cv2.putText(vis_image, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw_color, 2)
        
        self.save_step(vis_image, "final_results", image_name)
        return vis_image

class SimplifiedTask4Pipeline:
    """Main pipeline class that orchestrates the detection process"""
    
    def __init__(self, config: SimplifiedConfig = None):
        self.config = config or SimplifiedConfig()
        self.blob_detector = BlobDetector(self.config)
        self.spatial_filter = SpatialFilter(self.config)
        self.blob_merger = BlobMerger(self.config)
        self.shape_verifier = ShapeVerifier(self.config)
    
    def process_image(self, image_path: str, output_dir: str) -> List[Dict]:
        """Process a single image and return detection results"""
        # Load image
        image = cv2.imread(image_path)
        if image is None:
            raise ValueError(f"Could not load image: {image_path}")
        
        image_name = Path(image_path).stem
        debugger = DebugVisualizer(self.config, output_dir)
        
        print(f"Processing {image_name}...")
        
        # Step 1: Detect yellow and black blobs
        yellow_bboxes, yellow_mask = self.blob_detector.detect_yellow_blobs(image)
        black_bboxes, black_mask = self.blob_detector.detect_black_blobs(image)
        
        debugger.save_step(yellow_mask, "01_yellow_mask", image_name)
        debugger.save_step(black_mask, "01_black_mask", image_name)
        debugger.visualize_blobs(image, yellow_bboxes, black_bboxes, "02_initial_blobs", image_name)
        
        print(f"  Initial detection: {len(yellow_bboxes)} yellow, {len(black_bboxes)} black blobs")
        
        # Step 2: Filter to lower 60% of frame
        yellow_bboxes = self.spatial_filter.filter_to_lower_frame(yellow_bboxes, image.shape[0])
        black_bboxes = self.spatial_filter.filter_to_lower_frame(black_bboxes, image.shape[0])
        
        debugger.visualize_blobs(image, yellow_bboxes, black_bboxes, "03_spatial_filtered", image_name)
        
        print(f"  After spatial filtering: {len(yellow_bboxes)} yellow, {len(black_bboxes)} black blobs")
        
        # Step 3: Merge nearby blobs
        yellow_bboxes = self.blob_merger.merge_nearby_blobs(yellow_bboxes)
        black_bboxes = self.blob_merger.merge_nearby_blobs(black_bboxes)
        
        debugger.visualize_blobs(image, yellow_bboxes, black_bboxes, "04_merged_blobs", image_name)
        
        print(f"  After merging: {len(yellow_bboxes)} yellow, {len(black_bboxes)} black blobs")
        
        # Step 3.5: Resolve overlaps between all vessels (keep the bigger bbox when overlapping)
        bboxes_with_color = [(b, 'yellow') for b in yellow_bboxes] + [(b, 'black') for b in black_bboxes]
        resolved = self.blob_merger.resolve_overlaps(bboxes_with_color)
        yellow_bboxes = [b for b, c in resolved if c == 'yellow']
        black_bboxes = [b for b, c in resolved if c == 'black']
        
        debugger.visualize_blobs(image, yellow_bboxes, black_bboxes, "05_overlap_resolved", image_name)
        
        print(f"  After overlap resolution: {len(yellow_bboxes)} yellow, {len(black_bboxes)} black blobs")
        
        # Step 4: Shape verification and final results
        detections = []
        
        # Process yellow blobs (look for triangles above)
        for bbox in yellow_bboxes:
            has_triangle, triangle_confidence = self.shape_verifier.detect_triangle_above_blob(image, bbox)
            base_confidence = 0.7  # Base confidence for blob detection
            final_confidence = base_confidence + (0.3 * triangle_confidence) if has_triangle else base_confidence
            
            detections.append({
                'bbox': bbox,
                'color': 'yellow',
                'confidence': final_confidence,
                'has_shape': has_triangle,
                'shape_confidence': triangle_confidence
            })
        
        # Process black blobs (look for crosses above)
        for bbox in black_bboxes:
            has_cross, cross_confidence = self.shape_verifier.detect_cross_above_blob(image, bbox)
            base_confidence = 0.7  # Base confidence for blob detection
            final_confidence = base_confidence + (0.3 * cross_confidence) if has_cross else base_confidence
            
            detections.append({
                'bbox': bbox,
                'color': 'black',
                'confidence': final_confidence,
                'has_shape': has_cross,
                'shape_confidence': cross_confidence
            })
        
        # Filter detections by confidence threshold
        detections = [d for d in detections if d['confidence'] >= self.config.MIN_CONFIDENCE_THRESHOLD]
        
        # Visualize final results
        debugger.visualize_final_results(image, detections, image_name)
        
        print(f"  Final detections: {len(detections)} total (conf >= {self.config.MIN_CONFIDENCE_THRESHOLD})")
        for i, det in enumerate(detections):
            shape_info = f" + {('triangle' if det['color'] == 'yellow' else 'cross')}" if det['has_shape'] else ""
            print(f"    {i+1}. {det['color']}{shape_info} (conf: {det['confidence']:.3f})")
        
        return detections
    
    def process_all_images(self, input_dir: str, output_dir: str) -> Dict[str, List[Dict]]:
        """Process all images in input directory"""
        input_path = Path(input_dir)
        output_path = Path(output_dir)
        
        if not input_path.exists():
            raise ValueError(f"Input directory does not exist: {input_dir}")
        
        # Find all image files
        image_extensions = {'.jpg', '.jpeg', '.png', '.bmp', '.tiff'}
        image_files = [f for f in input_path.iterdir() 
                      if f.suffix.lower() in image_extensions]
        
        if not image_files:
            raise ValueError(f"No image files found in {input_dir}")
        
        print(f"Found {len(image_files)} images to process")
        
        all_results = {}
        
        for image_file in sorted(image_files):
            image_output_dir = output_path / image_file.stem
            try:
                results = self.process_image(str(image_file), str(image_output_dir))
                all_results[image_file.name] = results
            except Exception as e:
                print(f"Error processing {image_file.name}: {e}")
                all_results[image_file.name] = []
        
        return all_results

def main():
    parser = argparse.ArgumentParser(description="Task 4 Simplified Supply Drop Detection")
    parser.add_argument("--input", "-i", 
                       default="computer_vision/task_specific/task4/inputs",
                       help="Input directory containing images")
    parser.add_argument("--output", "-o",
                       default="computer_vision/task_specific/task4/simplified_debug_outputs",
                       help="Output directory for debug images")
    parser.add_argument("--single", "-s",
                       help="Process single image file instead of directory")
    
    args = parser.parse_args()
    
    # Create pipeline
    config = SimplifiedConfig()
    pipeline = SimplifiedTask4Pipeline(config)
    
    try:
        if args.single:
            # Process single image
            output_dir = Path(args.output) / Path(args.single).stem
            results = pipeline.process_image(args.single, str(output_dir))
            print(f"\nProcessed {args.single}")
            print(f"Found {len(results)} detections")
        else:
            # Process all images
            results = pipeline.process_all_images(args.input, args.output)
            
            # Summary
            print(f"\n{'='*60}")
            print("PROCESSING SUMMARY")
            print(f"{'='*60}")
            
            total_detections = 0
            for image_name, detections in results.items():
                total_detections += len(detections)
                print(f"{image_name}: {len(detections)} detections")
            
            print(f"\nTotal detections across all images: {total_detections}")
            print(f"Debug images saved to: {args.output}")
    
    except Exception as e:
        print(f"Error: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    exit(main())