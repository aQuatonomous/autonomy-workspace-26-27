#!/usr/bin/env python3
"""
Maritime Distance Estimator Node - Specs-Based Approach

Subscribes to /combined/detection_info and estimates distance for each detection using
camera specifications (AR0234, 3.56mm focal length, 3μm pixels, 1920×1200) instead
of calibration files. Uses class-specific reference dimensions from RoboBoat 2026 specs.

This replaces the old buoy_distance_estimator.py with a more accurate specs-based approach
that works consistently across different capture methods and doesn't depend on calibration files.

Subscribes: /combined/detection_info (combined detections from all cameras)
Publishes:  /combined/detection_info_with_distance (same structure + distance_m field per detection)

Camera Specifications (AR0234, Arducam B0495):
- Sensor: 1920(H) × 1200(V) pixels
- Pixel size: 3 μm × 3 μm  
- Focal length: 3.56 mm
- FOV: 98°(D) × 85°(H) × 69°(V)
- Resolution: 1920×1200@50fps (USB 3.0)

Distance Formula: distance_m = (fy * reference_height_m) / height_px
where fy is computed from specs: fy = focal_length_mm / pixel_size_mm

Reference dimensions from RoboBoat 2026 specifications:
- Small buoys (1 ft above waterline): black_buoy, green_buoy, red_buoy, yellow_buoy  
- Pole buoys (39 in above waterline): green_pole_buoy, red_pole_buoy
- Dock (16 in height), digits (24 in banners), etc.
"""

import json
import os
from typing import Optional, Dict, List

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import ParameterType
from std_msgs.msg import String

# AR0234 Camera Specifications
PIXEL_SIZE_UM = 3.0                    # 3 μm × 3 μm square pixels
FOCAL_LENGTH_MM = 3.56                 # 3.56 mm focal length
SENSOR_WIDTH_PX = 1920                 # 1920×1200 active area
SENSOR_HEIGHT_PX = 1200
HORIZONTAL_FOV_DEG = 85.0              # From manufacturer specs
VERTICAL_FOV_DEG = 69.0

# Computed focal length in pixels: f_px = f_mm / (pixel_pitch_mm)
FY_PX = FOCAL_LENGTH_MM / (PIXEL_SIZE_UM / 1000.0)  # ~1186.67 pixels
FX_PX = FY_PX  # Square pixels
CX_PX = SENSOR_WIDTH_PX / 2.0
CY_PX = SENSOR_HEIGHT_PX / 2.0

# K matrix for native 1920×1200 resolution (scaled for other resolutions)
K_NATIVE = np.array([
    [FX_PX, 0, CX_PX],
    [0, FY_PX, CY_PX],
    [0, 0, 1.0],
], dtype=np.float64)

# Reference dimensions (meters) - RoboBoat 2026 specifications / calibration test buoys
# Most objects: height above waterline for consistent distance measurement
# Indicator buoys: actual measured width (20 inches) for width-based estimation
# Note: Small buoy dimensions are task-specific and configured via get_task_dimensions()
DEFAULT_REFERENCE_M = 0.305  # Default to 1 ft for unknown objects


def get_task_dimensions(task_number):
    """
    Get reference dimensions based on task number.
    
    Task 2: Black, green, red buoys = 0.5 ft (0.1524m)
    Task 3: Green, red, yellow buoys = 1 ft (0.3048m), black buoys = 0.5 ft (0.1524m)
    
    Args:
        task_number: Task number (2 or 3)
        
    Returns:
        Dictionary of reference dimensions in meters
    """
    # Base dimensions (same for all tasks)
    base_dimensions = {
        # Navigation markers (same for all tasks)
        'cross': 0.203,             # 8 in diamond size (vertex-to-vertex)
        'triangle': 0.203,          # 8 in diamond size (estimated similar)
        
        # Infrastructure (same for all tasks)
        'red_indicator_buoy': 0.508,     # 20 in total width (actual measured)
        'green_indicator_buoy': 0.508,   # 20 in total width (actual measured)
        'yellow_supply_drop': 0.406,    # 16.0 in width (height not specified)
        'black_supply_drop': 0.406,     # 16.0 in width (height not specified)
        
        # Pole buoys (same for all tasks)
        'green_pole_buoy': 0.991,   # 39 in above waterline (950400)
        'red_pole_buoy': 0.991,     # 39 in above waterline (950410)
        
        # Docking number banners (same for all tasks)
        'digit_1': 0.610,          # 24 in × 24 in banner
        'digit_2': 0.610,          # 24 in × 24 in banner
        'digit_3': 0.610,          # 24 in × 24 in banner
    }
    
    # Task-specific small buoy dimensions
    if task_number == 2:
        # Task 2: All small buoys are 0.5 ft
        task_dimensions = {
            'black_buoy': 0.1524,   # 0.5 ft
            'green_buoy': 0.1524,   # 0.5 ft
            'red_buoy': 0.1524,     # 0.5 ft
            'yellow_buoy': 0.1524,  # 0.5 ft (not typically used in task 2)
        }
    elif task_number == 3:
        # Task 3: Green/red/yellow = 1 ft, black = 0.5 ft
        task_dimensions = {
            'black_buoy': 0.1524,   # 0.5 ft
            'green_buoy': 0.3048,   # 1 ft
            'red_buoy': 0.3048,     # 1 ft
            'yellow_buoy': 0.3048,  # 1 ft
        }
    else:
        # Default/fallback (use calibration reference values)
        task_dimensions = {
            'black_buoy': 0.254,    # 10 in (calibration reference)
            'green_buoy': 0.254,    # 10 in (calibration reference)
            'red_buoy': 0.254,      # 10 in (calibration reference)
            'yellow_buoy': 0.254,  # 10 in (calibration reference)
        }
    
    return {**base_dimensions, **task_dimensions}


class MaritimeDistanceEstimatorNode(Node):
    """
    Estimates distance for combined detections using AR0234 camera specifications.
    
    Uses pinhole model with camera intrinsics computed from manufacturer specs
    rather than calibration files. More accurate and consistent across different
    capture methods (video extraction, photo apps, terminal capture, etc.).
    """

    def __init__(self):
        super().__init__('maritime_distance_estimator')
        
        # One-point calibration: scale factor from measured distance vs specs estimate.
        # Set to measured_distance_m / distance_specs for one known reference object.
        self.declare_parameter('distance_scale_factor', 1.0)
        pval = self.get_parameter('distance_scale_factor').get_parameter_value()
        if pval.type == ParameterType.PARAMETER_STRING:
            self._distance_scale_factor = float(pval.string_value)
        else:
            self._distance_scale_factor = pval.double_value
        
        # Task parameter: determines buoy reference dimensions
        # Task 2: All small buoys = 0.5 ft
        # Task 3: Green/red/yellow = 1 ft, black = 0.5 ft
        self.declare_parameter('task', 2)  # Default to task 2
        pval_task = self.get_parameter('task').get_parameter_value()
        if pval_task.type == ParameterType.PARAMETER_STRING:
            self._task = int(pval_task.string_value)
        else:
            self._task = pval_task.integer_value
        
        # Get task-specific reference dimensions
        self._reference_dimensions = get_task_dimensions(self._task)
        
        # Camera intrinsics from AR0234 specifications
        self._native_fy = FY_PX
        self._native_fx = FX_PX
        self._native_width = SENSOR_WIDTH_PX
        self._native_height = SENSOR_HEIGHT_PX
        
        # Current frame dimensions (updated from combined detection messages)
        self._frame_width = SENSOR_WIDTH_PX
        self._frame_height = SENSOR_HEIGHT_PX
        
        self.get_logger().info(
            f'Maritime Distance Estimator initialized'
        )
        self.get_logger().info(
            f'Task {self._task} configuration loaded'
        )
        if self._distance_scale_factor != 1.0:
            self.get_logger().info(
                f'One-point calibration: distance_scale_factor={self._distance_scale_factor}'
            )
        self.get_logger().info(
            f'Camera specs: {SENSOR_WIDTH_PX}×{SENSOR_HEIGHT_PX}, {PIXEL_SIZE_UM}μm pixels, {FOCAL_LENGTH_MM}mm focal length'
        )
        self.get_logger().info(
            f'Native fy: {self._native_fy:.1f} px, loaded {len(self._reference_dimensions)} object reference dimensions'
        )
        # Log task-specific buoy dimensions
        buoy_dims = {k: v for k, v in self._reference_dimensions.items() 
                    if 'buoy' in k and 'pole' not in k and 'indicator' not in k}
        self.get_logger().info(
            f'Small buoy dimensions (Task {self._task}): {buoy_dims}'
        )

        # Subscribe to combined detections, publish with distance estimates
        self._sub = self.create_subscription(
            String, 
            '/combined/detection_info', 
            self._callback, 
            10
        )
        self._pub = self.create_publisher(
            String, 
            '/combined/detection_info_with_distance', 
            10
        )
        
        self.get_logger().info('Subscribed to: /combined/detection_info')
        self.get_logger().info('Publishing to: /combined/detection_info_with_distance')

    def _get_effective_fy(self, frame_width: int, frame_height: int) -> float:
        """
        Compute effective fy for given frame dimensions.
        Scales native focal length if frame size differs from 1920×1200.
        """
        if frame_width == self._native_width and frame_height == self._native_height:
            return self._native_fy
        
        # Scale fy based on height ratio (focal length scales with resolution)
        scale_y = frame_height / self._native_height
        return self._native_fy * scale_y

    def _estimate_distance_m(self, detection: Dict) -> Optional[float]:
        """
        Estimate distance for a single detection using pinhole model.
        
        For indicator buoys: Uses width-based estimation with 20-inch reference
        For other objects: Uses height-based estimation
        
        Args:
            detection: Detection dict with bbox, class_name, camera_id
            
        Returns:
            Distance in meters, or None if calculation fails or object size is unknown
        """
        # Extract bbox and validate
        bbox = detection.get('bbox')
        if not bbox or len(bbox) < 4:
            return None
            
        class_name = detection.get('class_name', '')
        
        # Dock always returns None (sizes vary enormously and are unknown)
        if class_name == 'dock':
            return None
            
        x1, y1, x2, y2 = float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3])
        width_px = max(1.0, x2 - x1)
        height_px = max(1.0, y2 - y1)
        
        # Get reference dimension for this object class using task-specific dimensions
        reference_dimension_m = self._reference_dimensions.get(class_name, DEFAULT_REFERENCE_M)
        
        # Compute effective focal length for current frame size
        fy_eff = self._get_effective_fy(self._frame_width, self._frame_height)
        fx_eff = fy_eff  # Square pixels, so fx = fy
        
        # Special handling for indicator buoys: use width-based estimation
        if class_name in ['red_indicator_buoy', 'green_indicator_buoy']:
            # For indicator buoys, use bounding box width with 20-inch reference
            distance_m = (fx_eff * reference_dimension_m) / width_px
            self.get_logger().debug(
                f'Indicator buoy distance: bbox_width={width_px:.1f}px, '
                f'reference={reference_dimension_m:.3f}m (20 inches), distance={distance_m:.2f}m'
            )
        else:
            # Standard height-based estimation for all other objects
            distance_m = (fy_eff * reference_dimension_m) / height_px
        
        # Apply one-point calibration scale (measured_distance / distance_specs)
        distance_m = distance_m * self._distance_scale_factor
        distance_m = max(0.01, distance_m)  # avoid non-positive
        return distance_m

    def _callback(self, msg: String) -> None:
        """
        Process combined detection message and add distance estimates.
        
        Reads combined detections, estimates distance for each detection,
        and republishes with distance_m field added to each detection.
        """
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Invalid JSON in combined detections: {e}')
            return

        # Update frame dimensions from camera_info if available
        camera_info = data.get('camera_info', {})
        if 'frame_width' in camera_info and 'frame_height' in camera_info:
            new_width = int(camera_info['frame_width'])
            new_height = int(camera_info['frame_height'])
            if new_width != self._frame_width or new_height != self._frame_height:
                self._frame_width = new_width
                self._frame_height = new_height
                fy_eff = self._get_effective_fy(new_width, new_height)
                self.get_logger().info(
                    f'Updated frame size to {new_width}×{new_height}, fy_eff: {fy_eff:.1f} px'
                )

        # Process detections and add distance estimates
        detections = data.get('detections', [])
        enhanced_detections = []
        
        for detection in detections:
            # Copy detection and add distance estimate
            enhanced_det = dict(detection)
            distance_m = self._estimate_distance_m(detection)
            
            if distance_m is not None:
                enhanced_det['distance_m'] = round(distance_m, 2)
                enhanced_det['distance_ft'] = round(distance_m * 3.28084, 1)
                # Add reference dimension used for transparency/debugging
                class_name = detection.get('class_name', '')
                enhanced_det['reference_height_m'] = self._reference_dimensions.get(class_name, DEFAULT_REFERENCE_M)
            else:
                enhanced_det['distance_m'] = None
                enhanced_det['distance_ft'] = None
                enhanced_det['reference_height_m'] = None
            
            enhanced_detections.append(enhanced_det)

        # Update data with enhanced detections
        enhanced_data = dict(data)
        enhanced_data['detections'] = enhanced_detections
        enhanced_data['distance_estimation'] = {
            'method': 'specs_based',
            'camera_model': 'AR0234',
            'focal_length_mm': FOCAL_LENGTH_MM,
            'pixel_size_um': PIXEL_SIZE_UM,
            'native_resolution': f'{SENSOR_WIDTH_PX}×{SENSOR_HEIGHT_PX}',
            'current_resolution': f'{self._frame_width}×{self._frame_height}',
            'effective_fy_px': round(self._get_effective_fy(self._frame_width, self._frame_height), 1),
            'num_object_references': len(self._reference_dimensions),
            'distance_scale_factor': self._distance_scale_factor,
            'task': self._task,
        }

        # Publish enhanced detections
        out_msg = String()
        out_msg.data = json.dumps(enhanced_data)
        self._pub.publish(out_msg)

        # Log summary
        num_with_distance = sum(1 for d in enhanced_detections if d.get('distance_m') is not None)
        if enhanced_detections:
            self.get_logger().debug(
                f'Processed {len(enhanced_detections)} detections, {num_with_distance} with distance estimates'
            )


def main(args=None):
    """Main entry point for maritime distance estimator node."""
    rclpy.init(args=args)
    node = MaritimeDistanceEstimatorNode()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info('Maritime distance estimator shutting down...')
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()