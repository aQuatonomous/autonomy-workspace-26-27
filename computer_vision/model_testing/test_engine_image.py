#!/usr/bin/env python3
"""
Simple test script to run TensorRT engine inference on a single image.
This is useful for debugging detection issues.
"""

import sys
import os
import cv2
import argparse

"""Simple engine test helper."""

# Add models/ (where test_inference.py now lives) to path to import TensorRTInference
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CV_ROOT = os.path.dirname(SCRIPT_DIR)
MODELS_DIR = os.path.join(CV_ROOT, "models")
DEFAULT_ENGINE = os.path.join(CV_ROOT, "model_building_and_training", "model.engine")
sys.path.insert(0, MODELS_DIR)

from test_inference import TensorRTInference
import numpy as np


def postprocess_yolo_fixed(output: np.ndarray, conf_threshold=0.25, iou_threshold=0.45, debug=False, imgsz=640):
    """
    Fixed postprocessing that handles both (1, 13, 8400) and (1, 27, 8400) formats.
    """
    # Output shape: (1, C, 8400) -> reshape to (8400, C)
    predictions = output[0].transpose((1, 0))  # (8400, C)
    num_classes = predictions.shape[1] - 5  # Total - 4 boxes - 1 objectness
    
    if debug:
        print(f"\n=== DEBUG INFO ===")
        print(f"Output shape: {output.shape}")
        print(f"Predictions shape: {predictions.shape}")
        print(f"Number of classes detected: {num_classes}")
        print(f"Expected format: 4 boxes + 1 objectness + {num_classes} classes = {predictions.shape[1]}")
    
    # Extract boxes (first 4 values)
    boxes = predictions[:, :4]
    objectness = predictions[:, 4:5]  # Objectness score
    class_scores = predictions[:, 5:]  # Class scores
    
    if debug:
        print(f"\nBoxes stats:")
        print(f"  Min/Max x: {boxes[:, 0].min():.4f} / {boxes[:, 0].max():.4f}")
        print(f"  Min/Max y: {boxes[:, 1].min():.4f} / {boxes[:, 1].max():.4f}")
        print(f"  Min/Max w: {boxes[:, 2].min():.4f} / {boxes[:, 2].max():.4f}")
        print(f"  Min/Max h: {boxes[:, 3].min():.4f} / {boxes[:, 3].max():.4f}")
        print(f"\nObjectness stats:")
        print(f"  Min/Max: {objectness.min():.4f} / {objectness.max():.4f}")
        print(f"  Mean: {objectness.mean():.4f}")
        print(f"  Values > 0.1: {(objectness > 0.1).sum()}")
        print(f"  Values > 0.25: {(objectness > 0.25).sum()}")
        print(f"  Values > 0.5: {(objectness > 0.5).sum()}")
        print(f"\nClass scores stats:")
        print(f"  Shape: {class_scores.shape}")
        print(f"  Min/Max: {class_scores.min():.4f} / {class_scores.max():.4f}")
        print(f"  Mean: {class_scores.mean():.4f}")
    
    # Calculate final scores - try multiple methods
    # Method 1: objectness * class_scores (standard YOLO)
    scores_method1 = objectness * class_scores
    
    # Method 2: Apply sigmoid to objectness first (if it's logit)
    objectness_sigmoid = 1.0 / (1.0 + np.exp(-np.clip(objectness, -500, 500)))
    scores_method2 = objectness_sigmoid * class_scores
    
    # Method 3: Use class scores directly (ignore objectness)
    scores_method3 = class_scores
    
    # Method 4: Apply sigmoid to class scores (if they're logits)
    class_scores_sigmoid = 1.0 / (1.0 + np.exp(-np.clip(class_scores, -500, 500)))
    scores_method4 = class_scores_sigmoid
    
    # Method 5: sigmoid(objectness) * sigmoid(class_scores)
    scores_method5 = objectness_sigmoid * class_scores_sigmoid
    
    # Try all methods and pick the one with highest max score
    all_methods = [
        ("objectness * class_scores", scores_method1),
        ("sigmoid(objectness) * class_scores", scores_method2),
        ("class_scores only", scores_method3),
        ("sigmoid(class_scores)", scores_method4),
        ("sigmoid(objectness) * sigmoid(class_scores)", scores_method5),
    ]
    
    best_method = None
    best_max = -1
    for method_name, method_scores in all_methods:
        method_max = np.max(method_scores)
        if method_max > best_max:
            best_max = method_max
            best_method = method_name
            scores = method_scores
    
    if debug:
        print(f"\nTrying different scoring methods:")
        for method_name, method_scores in all_methods:
            method_max = np.max(method_scores)
            method_mean = np.mean(method_scores)
            method_above_thresh = (method_scores > conf_threshold).sum()
            print(f"  {method_name}: max={method_max:.4f}, mean={method_mean:.6f}, above_thresh={method_above_thresh}")
        print(f"\nSelected method: {best_method} (max score: {best_max:.4f})")
    
    # Get class IDs and max scores
    class_ids = np.argmax(scores, axis=1)
    max_scores = np.max(scores, axis=1)
    
    if debug:
        print(f"\nFinal scores stats:")
        print(f"  Min/Max: {max_scores.min():.4f} / {max_scores.max():.4f}")
        print(f"  Mean: {max_scores.mean():.4f}")
        print(f"  Values > 0.1: {(max_scores > 0.1).sum()}")
        print(f"  Values > 0.25: {(max_scores > 0.25).sum()}")
        print(f"  Values > 0.5: {(max_scores > 0.5).sum()}")
        print(f"  Using threshold: {conf_threshold}")
    
    # Filter by confidence
    valid = max_scores > conf_threshold
    boxes = boxes[valid]
    scores = max_scores[valid]
    class_ids = class_ids[valid]
    
    if debug:
        print(f"\nAfter filtering:")
        print(f"  Valid detections: {len(boxes)}")
    
    if len(boxes) == 0:
        if debug:
            print("No detections found!")
        return []
    
    # Check box format and convert to corner format
    max_x = boxes[:, 0].max()
    max_y = boxes[:, 1].max()
    max_w = boxes[:, 2].max()
    max_h = boxes[:, 3].max()
    
    if debug:
        print(f"\nBox format check:")
        print(f"  Max x: {max_x:.4f}, Max y: {max_y:.4f}")
        print(f"  Max w: {max_w:.4f}, Max h: {max_h:.4f}")
    
    # Convert from center format to corner format
    if max_x <= 1.0 and max_y <= 1.0:
        # Normalized center format
        x_center, y_center, w, h = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
        x1 = (x_center - w / 2) * imgsz
        y1 = (y_center - h / 2) * imgsz
        x2 = (x_center + w / 2) * imgsz
        y2 = (y_center + h / 2) * imgsz
    elif max_x <= imgsz and max_y <= imgsz:
        # Pixel center format
        x_center, y_center, w, h = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
        x1 = x_center - w / 2
        y1 = y_center - h / 2
        x2 = x_center + w / 2
        y2 = y_center + h / 2
    else:
        # Already in corner format (pixel coordinates)
        x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    
    boxes_corners = np.stack([x1, y1, x2, y2], axis=1)
    
    # Clip to image bounds
    boxes_corners[:, 0] = np.clip(boxes_corners[:, 0], 0, imgsz)
    boxes_corners[:, 1] = np.clip(boxes_corners[:, 1], 0, imgsz)
    boxes_corners[:, 2] = np.clip(boxes_corners[:, 2], 0, imgsz)
    boxes_corners[:, 3] = np.clip(boxes_corners[:, 3], 0, imgsz)
    
    # Filter out invalid boxes (width or height <= 0)
    valid_boxes = (boxes_corners[:, 2] > boxes_corners[:, 0]) & (boxes_corners[:, 3] > boxes_corners[:, 1])
    boxes_corners = boxes_corners[valid_boxes]
    scores = scores[valid_boxes]
    class_ids = class_ids[valid_boxes]
    
    if debug:
        print(f"\nAfter clipping and validation:")
        print(f"  Valid boxes: {len(boxes_corners)}")
        if len(boxes_corners) > 0:
            print(f"  Sample box: {boxes_corners[0]}")
    
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
            if debug:
                print(f"\nAfter NMS:")
                print(f"  Detections: {len(indices)}")
            return [
                {
                    'box': boxes_corners[i],
                    'score': float(scores[i]),
                    'class_id': int(class_ids[i])
                }
                for i in indices
            ]
    except Exception as e:
        if debug:
            print(f"NMS error: {e}")
        # Return all boxes if NMS fails
        return [
            {
                'box': boxes_corners[i],
                'score': float(scores[i]),
                'class_id': int(class_ids[i])
            }
            for i in range(len(boxes_corners))
        ]
    
    return []


def test_image(image_path: str, engine_path: str, conf_threshold: float = 0.25, debug: bool = True):
    """
    Test the TensorRT engine on a single image.
    
    Args:
        image_path: Path to input image
        engine_path: Path to TensorRT engine file
        conf_threshold: Confidence threshold for detections
        debug: Enable debug output
    """
    print("=" * 70)
    print("Testing TensorRT Engine on Single Image")
    print("=" * 70)
    print(f"Image: {image_path}")
    print(f"Engine: {engine_path}")
    print(f"Confidence threshold: {conf_threshold}")
    print("=" * 70)
    
    # Check if files exist
    if not os.path.isfile(image_path):
        print(f"ERROR: Image file not found: {image_path}")
        return False
    
    if not os.path.isfile(engine_path):
        print(f"ERROR: Engine file not found: {engine_path}")
        return False
    
    try:
        # Load inference engine
        print("\n[1/4] Loading TensorRT engine...")
        inferencer = TensorRTInference(engine_path)
        print("✓ Engine loaded successfully!")
        
        # Load image
        print(f"\n[2/4] Loading image: {image_path}")
        image = cv2.imread(image_path)
        if image is None:
            print(f"ERROR: Could not load image {image_path}")
            return False
        
        print(f"✓ Image loaded: {image.shape[1]}x{image.shape[0]} pixels")
        
        # Preprocess
        print(f"\n[3/4] Preprocessing image...")
        input_data = inferencer.preprocess(image)
        print(f"✓ Preprocessed: {input_data.shape}")
        
        # Run inference
        print(f"\n[4/4] Running inference...")
        import time
        start_time = time.time()
        output = inferencer.infer(input_data)
        inference_time = time.time() - start_time
        print(f"✓ Inference completed in {inference_time*1000:.2f} ms")
        
        # Post-process
        print(f"\n[5/5] Post-processing detections...")
        # Handle different output formats: (1, 13, 8400) vs (1, 27, 8400)
        detections = postprocess_yolo_fixed(output, conf_threshold=conf_threshold, debug=debug, imgsz=640)
        print(f"\n{'='*70}")
        print(f"RESULTS: Found {len(detections)} detection(s)")
        print(f"{'='*70}")
        
        if len(detections) > 0:
            print("\nDetections:")
            for i, det in enumerate(detections):
                x1, y1, x2, y2 = det['box']
                print(f"  [{i+1}] Class ID: {det['class_id']}, Confidence: {det['score']:.3f}")
                print(f"      Box: ({int(x1)}, {int(y1)}) -> ({int(x2)}, {int(y2)})")
                print(f"      Size: {int(x2-x1)}x{int(y2-y1)} pixels")
        else:
            print("\n⚠ WARNING: No detections found!")
            print("  This could mean:")
            print("  - The confidence threshold is too high (try lowering --conf)")
            print("  - The model is not detecting objects in this image")
            print("  - There's an issue with the engine file or preprocessing")
        
        # Draw detections and save
        result_image = inferencer.draw_detections(image, detections)
        output_path = os.path.join(os.path.dirname(image_path), "test_result.jpg")
        cv2.imwrite(output_path, result_image)
        print(f"\n✓ Result image saved to: {output_path}")
        
        return len(detections) > 0
        
    except Exception as e:
        print(f"\nERROR: {type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Test TensorRT engine on a single image",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Test with default engine and confidence threshold
  python test_engine_image.py /path/to/image.jpg
  
  # Test with custom engine path and lower confidence threshold
  python test_engine_image.py /path/to/image.jpg --engine /path/to/model.engine --conf 0.1
  
  # Test with debug output disabled
  python test_engine_image.py /path/to/image.jpg --no-debug
        """
    )
    
    parser.add_argument("image", help="Path to input image file")
    parser.add_argument(
        "--engine",
        default=DEFAULT_ENGINE,
        help=f"Path to TensorRT engine file (default: {DEFAULT_ENGINE})"
    )
    parser.add_argument(
        "--conf",
        type=float,
        default=0.25,
        help="Confidence threshold (default: 0.25)"
    )
    parser.add_argument(
        "--no-debug",
        action="store_true",
        help="Disable debug output"
    )
    
    args = parser.parse_args()
    
    success = test_image(
        args.image,
        args.engine,
        conf_threshold=args.conf,
        debug=not args.no_debug
    )
    
    sys.exit(0 if success else 1)

