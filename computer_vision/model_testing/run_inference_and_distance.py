#!/usr/bin/env python3
"""
Run TensorRT YOLO inference on a single image and estimate distance for each detection
using camera calibration and class-specific reference dimensions (RoboBoat 2026).
Outputs distances in meters and feet.
"""

import argparse
import os
import sys

import cv2
import numpy as np
import yaml

# Paths
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CV_ROOT = os.path.dirname(SCRIPT_DIR)
MODELS_DIR = os.path.join(CV_ROOT, "models")
sys.path.insert(0, MODELS_DIR)

from test_inference import TensorRTInference

# Default calibration (camera_grid_calibration_2)
DEFAULT_CALIB = os.path.join(CV_ROOT, "camera_grid_calibration_2", "camera_calib.npz")
DEFAULT_ENGINE = os.path.join(CV_ROOT, "model_building_and_training", "model.engine")
CLASS_MAPPING_PATH = os.path.join(CV_ROOT, "class_mapping.yaml")

# Reference dimensions (m) - same as maritime_distance_estimator.py (1 ft for small buoys, 39 in for pole)
REFERENCE_DIMENSIONS = {
    "black_buoy": 0.305,
    "green_buoy": 0.305,
    "green_pole_buoy": 0.991,
    "red_buoy": 0.305,
    "red_pole_buoy": 0.991,
    "yellow_buoy": 0.305,
    "cross": 0.203,
    "triangle": 0.203,
    "dock": 0.406,
    "red_indicator_buoy": 0.432,
    "green_indicator_buoy": 0.432,
    "yellow_supply_drop": 0.406,
    "black_supply_drop": 0.406,
    "digit_1": 0.610,
    "digit_2": 0.610,
    "digit_3": 0.610,
}
DEFAULT_REFERENCE_M = 0.3
M_TO_FT = 3.28084


def load_calibration(npz_path: str):
    if not npz_path or not os.path.isfile(npz_path):
        return None
    try:
        data = np.load(npz_path)
        K = data["K"]
        img_size = data["image_size"]
        calib_w = int(img_size[0]) if img_size.shape else 640
        calib_h = int(img_size[1]) if img_size.shape and len(img_size) > 1 else 480
        return {"K": K, "calib_width": calib_w, "calib_height": calib_h}
    except Exception:
        return None


def load_class_mapping(yaml_path: str):
    """Return dict class_id (int) -> class_name (str)."""
    if not os.path.isfile(yaml_path):
        return {}
    with open(yaml_path) as f:
        data = yaml.safe_load(f)
    classes = data.get("classes") or {}
    return {int(k): str(v) for k, v in classes.items()}


def estimate_distance_m(bbox_orig, frame_width: int, frame_height: int, calib: dict, reference_height_m: float, class_name: str = None) -> float:
    """Pinhole: distance_m = (fy_eff * reference_height_m) / height_px. Optional height_scale for reflection."""
    x1, y1, x2, y2 = bbox_orig[0], bbox_orig[1], bbox_orig[2], bbox_orig[3]
    height_px = max(1.0, y2 - y1)
    scale = HEIGHT_SCALE.get(class_name, 1.0)
    height_px = height_px * scale
    fy = calib["K"][1, 1]
    cw, ch = calib["calib_width"], calib["calib_height"]
    if cw > 0 and ch > 0 and frame_width > 0 and frame_height > 0:
        fy_eff = fy * (frame_height / ch)
    else:
        fy_eff = fy
    return (fy_eff * reference_height_m) / height_px


def main():
    parser = argparse.ArgumentParser(description="Run TensorRT inference and estimate distances")
    parser.add_argument("image", help="Path to image file")
    parser.add_argument("--engine", default=DEFAULT_ENGINE, help="Path to TensorRT engine")
    parser.add_argument("--calib", default=DEFAULT_CALIB, help="Path to camera_calib.npz")
    parser.add_argument("--conf", type=float, default=0.25, help="Confidence threshold")
    parser.add_argument("--out", default="", help="Output image path (default: result_inference_distance.jpg in image dir)")
    parser.add_argument("--no-save", action="store_true", help="Do not save result image")
    args = parser.parse_args()

    image_path = os.path.abspath(args.image)
    if not os.path.isfile(image_path):
        print(f"Error: Image not found: {image_path}")
        sys.exit(1)

    # Load calibration
    calib = load_calibration(args.calib)
    if calib is None:
        print(f"Warning: Calibration not found at {args.calib}; distances will use default focal length.")
        calib = {"K": np.array([[500.0, 0, 320.0], [0, 500.0, 240.0], [0, 0, 1.0]]), "calib_width": 640, "calib_height": 480}

    # Class ID -> name
    id_to_name = load_class_mapping(CLASS_MAPPING_PATH)
    if not id_to_name:
        print("Warning: class_mapping.yaml not found; class names will be 'class_N'.")

    # Load image
    image = cv2.imread(image_path)
    if image is None:
        print(f"Error: Could not load image {image_path}")
        sys.exit(1)
    orig_h, orig_w = image.shape[:2]
    scale_x = orig_w / 640.0
    scale_y = orig_h / 640.0

    # Load engine and run inference
    print(f"Loading TensorRT engine: {args.engine}")
    inferencer = TensorRTInference(args.engine)
    input_data = inferencer.preprocess(image)
    output = inferencer.infer(input_data)
    detections = inferencer.postprocess_yolo(output, conf_threshold=args.conf, debug=False)

    print(f"\nDetections: {len(detections)}")
    print("-" * 60)

    for det in detections:
        box = det["box"]  # in 640x640 space
        x1_640, y1_640, x2_640, y2_640 = box[0], box[1], box[2], box[3]
        x1 = x1_640 * scale_x
        y1 = y1_640 * scale_y
        x2 = x2_640 * scale_x
        y2 = y2_640 * scale_y
        bbox_orig = [x1, y1, x2, y2]

        class_id = int(det["class_id"])
        class_name = id_to_name.get(class_id, f"class_{class_id}")
        ref_m = REFERENCE_DIMENSIONS.get(class_name, DEFAULT_REFERENCE_M)
        dist_m = estimate_distance_m(bbox_orig, orig_w, orig_h, calib, ref_m, class_name)
        dist_ft = dist_m * M_TO_FT

        print(f"  {class_name}: {dist_m:.2f} m  ({dist_ft:.2f} ft)  [ref={ref_m:.3f} m, conf={det['score']:.2f}]")

    # Draw boxes and distances on image
    img_out = image.copy()
    for det in detections:
        box = det["box"]
        x1 = int(box[0] * scale_x)
        y1 = int(box[1] * scale_y)
        x2 = int(box[2] * scale_x)
        y2 = int(box[3] * scale_y)
        class_id = int(det["class_id"])
        class_name = id_to_name.get(class_id, f"class_{class_id}")
        ref_m = REFERENCE_DIMENSIONS.get(class_name, DEFAULT_REFERENCE_M)
        bbox_orig = [x1, y1, x2, y2]
        dist_m = estimate_distance_m(bbox_orig, orig_w, orig_h, calib, ref_m, class_name)
        dist_ft = dist_m * M_TO_FT

        cv2.rectangle(img_out, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label = f"{class_name}: {dist_m:.1f}m ({dist_ft:.1f}ft)"
        cv2.putText(img_out, label, (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    if not args.no_save:
        out_path = args.out or os.path.join(os.path.dirname(image_path), "result_inference_distance.jpg")
        cv2.imwrite(out_path, img_out)
        print(f"\nResult image saved to: {out_path}")

    return detections


if __name__ == "__main__":
    main()

