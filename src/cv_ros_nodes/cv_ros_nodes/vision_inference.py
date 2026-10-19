"""
TensorRT YOLO Inference ROS2 Node

Subscribes to: /camera{N}/image_preprocessed
Publishes to: /camera{N}/detections (Image with bounding boxes)
              /camera{N}/detection_info (String with detection details)

Usage: python3 vision_inference.py --camera_id 0
"""

import os
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import tensorrt as trt
import numpy as np
from typing import List
import ctypes
import time
import json
import argparse

from cv_ros_nodes.workspace_paths import (
    computer_vision_root,
    default_engine_path,
    default_number_engine_path,
)

CV_ROOT = str(computer_vision_root())
DEFAULT_ENGINE_PATH = default_engine_path()
DEFAULT_NUMBER_ENGINE_PATH = default_number_engine_path()


# ============================================================================
# CUDA MEMORY MANAGEMENT
# ============================================================================
class CudaRuntime:
    def __init__(self):
        try:
            self.cuda = ctypes.CDLL("libcudart.so")
        except:
            self.cuda = ctypes.CDLL("libcudart.so.12")
        
        # Define function signatures
        self.cuda.cudaMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
        self.cuda.cudaMalloc.restype = ctypes.c_int
        
        self.cuda.cudaMemcpy.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int]
        self.cuda.cudaMemcpy.restype = ctypes.c_int
        
        self.cuda.cudaMemcpyAsync.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int, ctypes.c_void_p]
        self.cuda.cudaMemcpyAsync.restype = ctypes.c_int
        
        self.cuda.cudaStreamCreate.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
        self.cuda.cudaStreamCreate.restype = ctypes.c_int
        
        self.cuda.cudaStreamSynchronize.argtypes = [ctypes.c_void_p]
        self.cuda.cudaStreamSynchronize.restype = ctypes.c_int
        
        self.cuda.cudaStreamDestroy.argtypes = [ctypes.c_void_p]
        self.cuda.cudaStreamDestroy.restype = ctypes.c_int
        
        self.cuda.cudaFree.argtypes = [ctypes.c_void_p]
        self.cuda.cudaFree.restype = ctypes.c_int
        
        self.cuda.cudaDeviceSynchronize.restype = ctypes.c_int
        
        # Memory copy kinds
        self.cudaMemcpyHostToDevice = 1
        self.cudaMemcpyDeviceToHost = 2
    
    def malloc(self, size):
        """Allocate GPU memory"""
        ptr = ctypes.c_void_p()
        result = self.cuda.cudaMalloc(ctypes.byref(ptr), size)
        if result != 0:
            raise RuntimeError(f"CUDA malloc failed: {result}")
        return ptr.value
    
    def memcpy_htod(self, dst, src, size):
        """Copy from host to device"""
        result = self.cuda.cudaMemcpy(dst, src, size, self.cudaMemcpyHostToDevice)
        if result != 0:
            raise RuntimeError(f"CUDA memcpy H2D failed: {result}")
    
    def memcpy_dtoh(self, dst, src, size):
        """Copy from device to host"""
        result = self.cuda.cudaMemcpy(dst, src, size, self.cudaMemcpyDeviceToHost)
        if result != 0:
            raise RuntimeError(f"CUDA memcpy D2H failed: {result}")
    
    def free(self, ptr):
        """Free GPU memory"""
        if ptr:
            self.cuda.cudaFree(ptr)
    
    def synchronize(self):
        """Synchronize CUDA operations"""
        self.cuda.cudaDeviceSynchronize()
    
    def create_stream(self):
        """Create a CUDA stream for async operations"""
        stream = ctypes.c_void_p()
        result = self.cuda.cudaStreamCreate(ctypes.byref(stream))
        if result != 0:
            raise RuntimeError(f"CUDA stream creation failed: {result}")
        return stream.value
    
    def memcpy_htod_async(self, dst, src, size, stream):
        """Copy from host to device asynchronously"""
        result = self.cuda.cudaMemcpyAsync(dst, src, size, self.cudaMemcpyHostToDevice, stream)
        if result != 0:
            raise RuntimeError(f"CUDA memcpy H2D async failed: {result}")
    
    def memcpy_dtoh_async(self, dst, src, size, stream):
        """Copy from device to host asynchronously"""
        result = self.cuda.cudaMemcpyAsync(dst, src, size, self.cudaMemcpyDeviceToHost, stream)
        if result != 0:
            raise RuntimeError(f"CUDA memcpy D2H async failed: {result}")
    
    def stream_synchronize(self, stream):
        """Synchronize a specific CUDA stream"""
        self.cuda.cudaStreamSynchronize(stream)
    
    def destroy_stream(self, stream):
        """Destroy a CUDA stream"""
        if stream:
            self.cuda.cudaStreamDestroy(stream)


# ============================================================================
# TENSORRT INFERENCE ENGINE
# ============================================================================
class TensorRTInference:
    def __init__(self, engine_path: str, imgsz: int = 640):
        """Load the TensorRT model file. imgsz: input spatial size (e.g. 640 or 960)."""
        self.imgsz = imgsz
        self.logger = trt.Logger(trt.Logger.WARNING)
        self.cuda = CudaRuntime()
        
        # Load engine
        with open(engine_path, "rb") as f:
            engine_data = f.read()
        
        runtime = trt.Runtime(self.logger)
        self.engine = runtime.deserialize_cuda_engine(engine_data)
        self.context = self.engine.create_execution_context()
        
        # Get input/output names and shapes
        self.input_name = None
        self.output_name = None
        
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            mode = self.engine.get_tensor_mode(name)
            if mode == trt.TensorIOMode.INPUT:
                self.input_name = name
                self.input_shape = self.engine.get_tensor_shape(name)
            else:
                self.output_name = name
                self.output_shape = self.engine.get_tensor_shape(name)
        
        print(f"Input: {self.input_name}, Shape: {self.input_shape}")
        print(f"Output: {self.output_name}, Shape: {self.output_shape}")
        
        # Allocate GPU memory
        self.input_size = np.prod(self.input_shape) * np.dtype(np.float32).itemsize
        self.output_size = np.prod(self.output_shape) * np.dtype(np.float32).itemsize
        
        self.d_input = self.cuda.malloc(self.input_size)
        self.d_output = self.cuda.malloc(self.output_size)
        
        # Set tensor addresses
        self.context.set_tensor_address(self.input_name, self.d_input)
        self.context.set_tensor_address(self.output_name, self.d_output)
        
        # Create CUDA stream for async operations
        self.stream = self.cuda.create_stream()
    
    def __del__(self):
        """Cleanup GPU memory"""
        if hasattr(self, 'cuda'):
            if hasattr(self, 'stream'):
                self.cuda.destroy_stream(self.stream)
            if hasattr(self, 'd_input'):
                self.cuda.free(self.d_input)
            if hasattr(self, 'd_output'):
                self.cuda.free(self.d_output)
    
    def preprocess(self, image: np.ndarray, is_rgb: bool = False):
        """Prepare image for the model with letterbox (preserve aspect ratio). Returns (tensor, letterbox_info).
        If is_rgb=True, image is already RGB (e.g. from Gazebo sim); otherwise assumed BGR.
        Optimized: uses faster cv2 operations and avoids unnecessary copies."""
        orig_h, orig_w = image.shape[:2]
        scale = min(self.imgsz / orig_w, self.imgsz / orig_h)
        new_w = int(round(orig_w * scale))
        new_h = int(round(orig_h * scale))
        # Use INTER_AREA for downscaling (faster), INTER_LINEAR for upscaling
        interp = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
        img_resized = cv2.resize(image, (new_w, new_h), interpolation=interp)
        pad_left = (self.imgsz - new_w) // 2
        pad_top = (self.imgsz - new_h) // 2
        
        # Pre-allocate canvas as float32 to avoid conversion later
        canvas = np.zeros((self.imgsz, self.imgsz, 3), dtype=np.float32)
        canvas[pad_top:pad_top + new_h, pad_left:pad_left + new_w] = img_resized.astype(np.float32) / 255.0

        # Convert BGR->RGB if needed, then transpose in one step
        if not is_rgb:
            # Swap channels: BGR -> RGB (faster than cvtColor for this case)
            canvas = canvas[:, :, ::-1]  # Reverse last dimension
        
        # Transpose HWC -> CHW and add batch dimension in one step
        img_batch = np.expand_dims(canvas.transpose(2, 0, 1), axis=0)

        letterbox_info = {
            'pad_left': pad_left,
            'pad_top': pad_top,
            'scale': scale,
            'orig_w': orig_w,
            'orig_h': orig_h,
        }
        return img_batch, letterbox_info
    
    def infer(self, input_data: np.ndarray) -> np.ndarray:
        """Run model inference on GPU with async operations for better performance"""
        # Ensure input is contiguous and float32
        input_data = np.ascontiguousarray(input_data, dtype=np.float32)
        
        # Copy input to GPU asynchronously
        self.cuda.memcpy_htod_async(
            self.d_input,
            input_data.ctypes.data,
            self.input_size,
            self.stream
        )
        
        # Execute inference asynchronously (using stream)
        self.context.execute_async_v3(stream_handle=self.stream)
        
        # Allocate output array (do this while GPU is working)
        output = np.empty(self.output_shape, dtype=np.float32)
        
        # Copy output from GPU asynchronously
        self.cuda.memcpy_dtoh_async(
            output.ctypes.data,
            self.d_output,
            self.output_size,
            self.stream
        )
        
        # Synchronize stream (waits for all async operations to complete)
        self.cuda.stream_synchronize(self.stream)
        
        return output
    
    def postprocess_yolo(self, output: np.ndarray, conf_threshold=0.25, iou_threshold=0.45) -> List:
        """Convert raw model output into bounding boxes"""
        # Output shape: (1, C, 8400) -> reshape to (8400, C)
        # Supports both (1, 13, 8400) for 8-class model and (1, 27, 8400) for 22-class model
        predictions = output[0].transpose((1, 0))
        
        # Extract boxes and scores
        boxes = predictions[:, :4]
        objectness = predictions[:, 4:5]
        class_scores = predictions[:, 5:]
        
        # Calculate final scores - handle low objectness by trying multiple methods
        # Method 1: objectness * class_scores (standard YOLO)
        scores_method1 = objectness * class_scores
        
        # Method 2: sigmoid(objectness) * class_scores
        objectness_sigmoid = 1.0 / (1.0 + np.exp(-np.clip(objectness, -500, 500)))
        scores_method2 = objectness_sigmoid * class_scores
        
        # Method 3: class_scores only (ignore objectness if it's too low)
        scores_method3 = class_scores
        
        # Choose best method based on max score
        max1 = np.max(scores_method1)
        max2 = np.max(scores_method2)
        max3 = np.max(scores_method3)
        
        if max3 >= max(max1, max2) * 0.5:  # If class_scores alone gives reasonable scores
            scores = scores_method3
        elif max2 > max1:
            scores = scores_method2
        else:
            scores = scores_method1
        
        # Get class IDs and max scores
        class_ids = np.argmax(scores, axis=1)
        max_scores = np.max(scores, axis=1)
        
        # Filter by confidence
        valid = max_scores > conf_threshold
        boxes = boxes[valid]
        scores = max_scores[valid]
        class_ids = class_ids[valid]
        
        if len(boxes) == 0:
            return []
        
        # Convert boxes to corner format
        max_x = boxes[:, 0].max()
        max_y = boxes[:, 1].max()
        
        sz = self.imgsz
        if max_x <= 1.0 and max_y <= 1.0:
            # Normalized center format
            x_center, y_center, w, h = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
            x1 = (x_center - w / 2) * sz
            y1 = (y_center - h / 2) * sz
            x2 = (x_center + w / 2) * sz
            y2 = (y_center + h / 2) * sz
        elif max_x <= sz and max_y <= sz:
            # Pixel center format
            x_center, y_center, w, h = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
            x1 = x_center - w / 2
            y1 = y_center - h / 2
            x2 = x_center + w / 2
            y2 = y_center + h / 2
        else:
            # Already in corner format
            x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
        
        boxes_corners = np.stack([x1, y1, x2, y2], axis=1)
        # Clip to image bounds
        boxes_corners = np.clip(boxes_corners, 0, self.imgsz)
        
        # Filter invalid boxes
        valid_boxes = (boxes_corners[:, 2] > boxes_corners[:, 0]) & (boxes_corners[:, 3] > boxes_corners[:, 1])
        boxes_corners = boxes_corners[valid_boxes]
        scores = scores[valid_boxes]
        class_ids = class_ids[valid_boxes]
        
        if len(boxes_corners) == 0:
            return []
        
        # Apply NMS
        try:
            indices = cv2.dnn.NMSBoxes(
                boxes_corners.tolist(),
                scores.tolist(),
                conf_threshold,
                iou_threshold
            )
            
            if indices is not None and len(indices) > 0:
                indices = indices.flatten()
                return [
                    {
                        'box': boxes_corners[i],
                        'score': float(scores[i]),
                        'class_id': int(class_ids[i])
                    }
                    for i in indices
                ]
        except Exception as e:
            print(f"NMS error: {e}")
            return [
                {
                    'box': boxes_corners[i],
                    'score': float(scores[i]),
                    'class_id': int(class_ids[i])
                }
                for i in range(len(boxes_corners))
            ]
        
        return []
    
    def draw_detections(self, image: np.ndarray, detections: List, scale_to_original=True, boxes_in_original_space=False, stale=False) -> np.ndarray:
        """Draw bounding boxes on image. If boxes_in_original_space=True, box coords are already in image space (no scaling). If stale=True, draw dimmer and label as previous (for smooth display when inference_interval>1)."""
        img_copy = image.copy()
        orig_h, orig_w = image.shape[:2]
        color = (0, 180, 0) if stale else (0, 255, 0)  # BGR: dimmer green for stale
        suffix = " (prev)" if stale else ""

        for det in detections:
            x1, y1, x2, y2 = det['box']
            score = det['score']
            class_id = int(det['class_id'])

            if boxes_in_original_space:
                x1, y1, x2, y2 = int(round(x1)), int(round(y1)), int(round(x2)), int(round(y2))
            elif scale_to_original and (orig_w != self.imgsz or orig_h != self.imgsz):
                scale_x = orig_w / float(self.imgsz)
                scale_y = orig_h / float(self.imgsz)
                x1 = int(x1 * scale_x)
                y1 = int(y1 * scale_y)
                x2 = int(x2 * scale_x)
                y2 = int(y2 * scale_y)
            else:
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            
            # Ensure coordinates are within image bounds
            x1 = max(0, min(x1, orig_w))
            y1 = max(0, min(y1, orig_h))
            x2 = max(0, min(x2, orig_w))
            y2 = max(0, min(y2, orig_h))
            
            # Draw rectangle
            cv2.rectangle(img_copy, (x1, y1), (x2, y2), color, 2)
            # Label: number detection (class_id 20,21,22 = digits 1,2,3) or generic
            if 20 <= class_id <= 22:
                label = f"{class_id - 19}: {score:.2f}{suffix}"
            else:
                label = f"Class {class_id}: {score:.2f}{suffix}"
            cv2.putText(img_copy, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        return img_copy


# Number detection: map model class 0,1,2 (digits 1,2,3) to reserved IDs to avoid collision with main model (0-8)
NUMBER_CLASS_OFFSET = 20  # class_id 20=digit1, 21=digit2, 22=digit3


def letterbox_boxes_to_original(detections: List, letterbox_info: dict) -> List:
    """Transform detection boxes from 640x640 letterbox space to original image coordinates."""
    pad_left = letterbox_info['pad_left']
    pad_top = letterbox_info['pad_top']
    scale = letterbox_info['scale']
    out = []
    for det in detections:
        x1, y1, x2, y2 = det['box']
        x1_o = (x1 - pad_left) / scale
        y1_o = (y1 - pad_top) / scale
        x2_o = (x2 - pad_left) / scale
        y2_o = (y2 - pad_top) / scale
        out.append({**det, 'box': np.array([x1_o, y1_o, x2_o, y2_o], dtype=np.float64)})
    return out


# ============================================================================
# ROS2 INFERENCE NODE
# ============================================================================
class InferenceNode(Node):

    def __init__(
        self,
        camera_id: int,
        engine_path: str = "model.engine",
        conf_threshold: float = 0.25,
        enable_number_detection: bool = False,
        number_detection_engine: str = "",
        number_conf_threshold: float = 0.25,
        inference_interval: int = 1,
        sim_image_rgb: bool = True,
        draw_stale_boxes: bool = True,
    ):
        super().__init__(f'inference_node_camera{camera_id}')
        self.camera_id = camera_id
        self.conf_threshold = conf_threshold
        self.sim_image_rgb = sim_image_rgb
        self.inference_interval = max(1, int(inference_interval))
        self.draw_stale_boxes = draw_stale_boxes
        self.number_conf_threshold = number_conf_threshold
        self.enable_number_detection = enable_number_detection
        self.number_inferencer = None

        # Main TensorRT engine (640x640)
        self.get_logger().info(f'Loading TensorRT engine from {engine_path}...')
        self.inferencer = TensorRTInference(engine_path, imgsz=640)
        self.get_logger().info('TensorRT engine loaded successfully!')

        # Optional number detection engine (960x960)
        if enable_number_detection and number_detection_engine:
            import os
            if os.path.isfile(number_detection_engine):
                try:
                    self.number_inferencer = TensorRTInference(number_detection_engine, imgsz=960)
                    self.get_logger().info(f'Number detection engine loaded from {number_detection_engine}')
                except Exception as e:
                    self.get_logger().warn(f'Failed to load number detection engine: {e}. Running without number detection.')
            else:
                self.get_logger().warn(f'Number detection engine not found: {number_detection_engine}. Running without number detection.')
        
        # ROS2 components
        self.bridge = CvBridge()
        self.frame_count = 0
        self.total_inference_time = 0
        # Frame-skip: store last detections and last detection_info so we can publish every frame
        self._last_detections = []
        self._last_detection_info_json = None
        # Actual output FPS: timestamps of when we publish detection_info (sliding window)
        self._output_timestamps = []
        self._output_fps_window = 60  # number of publishes to use for output FPS (e.g. ~2 s at 30 Hz)
        
        # QoS depth 1: only process latest frame to avoid backlog and improve effective FPS
        qos_latest = QoSProfile(depth=1)
        self.subscription = self.create_subscription(
            Image,
            f'/camera{camera_id}/image_preprocessed',
            self.image_callback,
            qos_latest
        )
        
        # Publisher for images with bounding boxes
        self.detection_image_pub = self.create_publisher(
            Image,
            f'/camera{camera_id}/detections',
            10
        )
        
        # Publisher for detection information (JSON)
        self.detection_info_pub = self.create_publisher(
            String,
            f'/camera{camera_id}/detection_info',
            10
        )
        
        # Display FPS: rate at which we publish detection images (for rqt)
        self._display_timestamps = []
        self.get_logger().info(f'Inference node for camera{camera_id} ready! (inference_interval={self.inference_interval}, sim_image_rgb={self.sim_image_rgb})')
        self.get_logger().info(f'Subscribed to: /camera{camera_id}/image_preprocessed')
        self.get_logger().info(f'Publishing to: /camera{camera_id}/detections, /camera{camera_id}/detection_info')
    
    def image_callback(self, msg: Image):
        """Process incoming images: run inference every inference_interval frames; publish detection image every frame."""
        try:
            # Sim (Gazebo/ros_gz_bridge) often publishes RGB; real cameras typically BGR. Use sim_image_rgb=true for sim.
            encoding = 'rgb8' if self.sim_image_rgb else 'bgr8'
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)
            orig_h, orig_w = cv_image.shape[:2]
            self.frame_count += 1
            # Run inference every Nth frame, or on first frame so we have detections to reuse
            run_inference = (
                (self.frame_count % self.inference_interval == 0) or
                (len(self._output_timestamps) == 0)
            )

            if run_inference:
                # Full pipeline: letterbox preprocess -> infer -> postprocess -> transform boxes to orig
                input_data, letterbox_info = self.inferencer.preprocess(cv_image, is_rgb=self.sim_image_rgb)
                start_time = time.time()
                output = self.inferencer.infer(input_data)
                inference_time = time.time() - start_time
                self.total_inference_time += inference_time

                detections = self.inferencer.postprocess_yolo(
                    output,
                    conf_threshold=self.conf_threshold
                )
                # Diagnostic: when 0 detections, log max raw score so we can see if model outputs anything
                if len(detections) == 0 and self.frame_count % 30 == 1:
                    predictions = output[0].transpose((1, 0))
                    obj = predictions[:, 4:5]
                    cls = predictions[:, 5:]
                    if obj.max() < 0.01:
                        obj = 1.0 / (1.0 + np.exp(-np.clip(obj, -500, 500)))
                    raw_scores = obj * cls
                    max_raw = float(np.max(raw_scores))
                    self.get_logger().warn(
                        f'Zero detections (conf>={self.conf_threshold}). Max raw score this frame: {max_raw:.4f}. '
                        'Try sim_image_rgb:=true (default for sim) or conf_threshold:=0.05.'
                    )
                # Transform main-model boxes from 640x640 letterbox to original image coordinates
                detections = letterbox_boxes_to_original(detections, letterbox_info)

                if self.number_inferencer is not None:
                    try:
                        num_input, num_letterbox = self.number_inferencer.preprocess(cv_image, is_rgb=self.sim_image_rgb)
                        num_output = self.number_inferencer.infer(num_input)
                        num_dets = self.number_inferencer.postprocess_yolo(
                            num_output,
                            conf_threshold=self.number_conf_threshold
                        )
                        for d in num_dets:
                            d['class_id'] = int(d['class_id']) + NUMBER_CLASS_OFFSET
                            d['source'] = 'number_detection'
                        num_dets = letterbox_boxes_to_original(num_dets, num_letterbox)
                        detections = detections + num_dets
                    except Exception as e:
                        self.get_logger().warn(f'Number detection failed: {e}')
                self._last_detections = detections
                now = time.time()
                self._output_timestamps.append(now)
                if len(self._output_timestamps) > self._output_fps_window:
                    self._output_timestamps.pop(0)
                inference_fps = (len(self._output_timestamps) - 1) / (self._output_timestamps[-1] - self._output_timestamps[0]) if len(self._output_timestamps) >= 2 else (1.0 / inference_time if inference_time > 0 else 0.0)

                detection_info = {
                    'camera_id': self.camera_id,
                    'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                    'frame_id': msg.header.frame_id,
                    'num_detections': len(detections),
                    'inference_time_ms': inference_time * 1000,
                    'fps': 1.0 / inference_time if inference_time > 0 else 0,
                    'output_fps': round(inference_fps, 2),
                    'frame_width': orig_w,
                    'frame_height': orig_h,
                    'detections': []
                }
                for det in detections:
                    x1, y1, x2, y2 = float(det['box'][0]), float(det['box'][1]), float(det['box'][2]), float(det['box'][3])
                    detection_info['detections'].append({
                        'class_id': int(det['class_id']),
                        'score': float(det['score']),
                        'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                        'width': float(x2 - x1), 'height': float(y2 - y1),
                        'bbox': [x1, y1, x2, y2]
                    })
                self._last_detection_info_json = json.dumps(detection_info)
                info_msg = String()
                info_msg.data = self._last_detection_info_json
                self.detection_info_pub.publish(info_msg)

                if self.frame_count % 15 == 0:
                    self.get_logger().info(
                        f'Processed {self.frame_count} frames | Inference FPS: {inference_fps:.2f} | Det: {len(detections)}'
                    )
            else:
                inference_fps = (len(self._output_timestamps) - 1) / (self._output_timestamps[-1] - self._output_timestamps[0]) if len(self._output_timestamps) >= 2 else 0.0
                # Publish detection_info even on non-inference frames so combiner knows camera is alive
                if self._last_detection_info_json:
                    info_msg = String()
                    info_msg.data = self._last_detection_info_json
                    self.detection_info_pub.publish(info_msg)

            # Publish detection image every frame. Draw boxes: fresh when we ran inference, or last (stale) for smooth display.
            if run_inference:
                result_image = self.inferencer.draw_detections(
                    cv_image, self._last_detections, boxes_in_original_space=True, stale=False
                )
            elif self.draw_stale_boxes and len(self._last_detections) > 0:
                result_image = self.inferencer.draw_detections(
                    cv_image, self._last_detections, boxes_in_original_space=True, stale=True
                )
            else:
                result_image = cv_image.copy()
            now = time.time()
            self._display_timestamps.append(now)
            if len(self._display_timestamps) > self._output_fps_window:
                self._display_timestamps.pop(0)
            display_fps = (len(self._display_timestamps) - 1) / (self._display_timestamps[-1] - self._display_timestamps[0]) if len(self._display_timestamps) >= 2 else 0.0
            cv2.putText(result_image, f"Display FPS: {display_fps:.1f} | Inference FPS: {inference_fps:.1f} | Det: {len(self._last_detections)}",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            out_encoding = 'rgb8' if self.sim_image_rgb else 'bgr8'
            detection_msg = self.bridge.cv2_to_imgmsg(result_image, encoding=out_encoding)
            detection_msg.header = msg.header
            self.detection_image_pub.publish(detection_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='ROS2 TensorRT Inference Node')
    parser.add_argument('--camera_id', type=int, default=0,
                        help='Camera ID (0, 1, or 2)')
    parser.add_argument(
        '--engine_path',
        type=str,
        default=DEFAULT_ENGINE_PATH,
        help=f'Path to TensorRT engine file (default: {DEFAULT_ENGINE_PATH})'
    )
    parser.add_argument('--conf_threshold', type=float, default=0.25,
                        help='Confidence threshold (0.0-1.0)')
    parser.add_argument('--enable_number_detection', type=str, default='false',
                        help='Enable docking number detection (digits 1,2,3); use "true" from launch')
    parser.add_argument(
        '--number_detection_engine',
        type=str,
        default='',
        help=f'Path to number detection TensorRT engine (e.g. {DEFAULT_NUMBER_ENGINE_PATH})'
    )
    parser.add_argument('--number_conf_threshold', type=float, default=0.25,
                        help='Confidence threshold for number detection (0.0-1.0)')
    parser.add_argument('--inference_interval', type=int, default=1,
                        help='Run inference every N frames (1=every frame). Use 2 or 3 for smoother display FPS on Jetson.')
    parser.add_argument('--sim_image_rgb', type=str, default='true',
                        help='If true, treat images as RGB (use for Gazebo sim; bridge often publishes RGB). Use false for real BGR cameras.')
    parser.add_argument('--draw_stale_boxes', type=str, default='true',
                        help='When inference_interval>1, draw last detections on non-inference frames (smooth display; boxes may lag slightly).')
    args_parsed, _ = parser.parse_known_args(args=args)

    if args_parsed.camera_id not in [0, 1, 2]:
        print("Error: camera_id must be 0, 1, or 2")
        return

    node = InferenceNode(
        camera_id=args_parsed.camera_id,
        engine_path=args_parsed.engine_path,
        conf_threshold=args_parsed.conf_threshold,
        enable_number_detection=args_parsed.enable_number_detection.lower() == 'true',
        number_detection_engine=args_parsed.number_detection_engine or '',
        number_conf_threshold=args_parsed.number_conf_threshold,
        inference_interval=args_parsed.inference_interval,
        sim_image_rgb=args_parsed.sim_image_rgb.lower() == 'true',
        draw_stale_boxes=args_parsed.draw_stale_boxes.lower() == 'true',
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