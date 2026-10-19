#!/usr/bin/env python3
"""
Distance estimation using ONLY camera specifications (no calibration file).

Camera: AR0234 sensor, 1920×1200, 3 µm pixels, 3.56 mm focal length, FOV 98°D×85°H×69°V.
Uses pinhole model: distance = (fy * reference_height_m) / height_px.
Object sizes from RoboBoat 2026 specs.
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

DEFAULT_ENGINE = os.path.join(CV_ROOT, "model_building_and_training", "model.engine")
CLASS_MAPPING_PATH = os.path.join(CV_ROOT, "class_mapping.yaml")

# -----------------------------------------------------------------------------
# Camera intrinsics from manufacturer spec (no calibration file)
# AR0234: 1920(H)×1200(V), pixel 3 µm × 3 µm, focal length 3.56 mm
# -----------------------------------------------------------------------------
PIXEL_SIZE_UM = 3.0
FOCAL_LENGTH_MM = 3.56
SENSOR_WIDTH_PX = 1920
SENSOR_HEIGHT_PX = 1200
# Focal length in pixels: f_px = f_mm / (pixel_pitch_mm) = 3.56 / 0.003
FY_PX = FOCAL_LENGTH_MM / (PIXEL_SIZE_UM / 1000.0)  # ~1186.67
FX_PX = FY_PX  # square pixels
CX_PX = SENSOR_WIDTH_PX / 2.0
CY_PX = SENSOR_HEIGHT_PX / 2.0
CALIB_WIDTH = SENSOR_WIDTH_PX
CALIB_HEIGHT = SENSOR_HEIGHT_PX

# K matrix for 1920×1200 (we scale fy if image is different size)
K_1920x1200 = np.array([
    [FX_PX, 0, CX_PX],
    [0, FY_PX, CY_PX],
    [0, 0, 1.0],
], dtype=np.float64)

# Reference dimensions (m) - RoboBoat 2026 known sizes
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


def get_calib_from_specs(frame_width: int, frame_height: int) -> dict:
    """Build calib dict from specs. Scales fy/fx if image size != 1920×1200."""
    if frame_width == CALIB_WIDTH and frame_height == CALIB_HEIGHT:
        return {"K": K_1920x1200.copy(), "calib_width": CALIB_WIDTH, "calib_height": CALIB_HEIGHT}
    scale_y = frame_height / CALIB_HEIGHT
    scale_x = frame_width / CALIB_WIDTH
    K = np.array([
        [FX_PX * scale_x, 0, CX_PX * scale_x],
        [0, FY_PX * scale_y, CY_PX * scale_y],
        [0, 0, 1.0],
    ], dtype=np.float64)
    return {"K": K, "calib_width": frame_width, "calib_height": frame_height}


def load_class_mapping(yaml_path: str) -> dict:
    if not os.path.isfile(yaml_path):
        return {}
    with open(yaml_path) as f:
        data = yaml.safe_load(f)
    classes = data.get("classes") or {}
    return {int(k): str(v) for k, v in classes.items()}


def estimate_distance_m(bbox_orig, frame_width: int, frame_height: int, calib: dict, reference_height_m: float) -> float:
    """Pinhole: distance_m = (fy * reference_height_m) / height_px."""
    x1, y1, x2, y2 = bbox_orig[0], bbox_orig[1], bbox_orig[2], bbox_orig[3]
    height_px = max(1.0, y2 - y1)
    fy = calib["K"][1, 1]
    return (fy * reference_height_m) / height_px


def main():
    parser = argparse.ArgumentParser(
        description="Distance from camera specs only (AR0234, 3.56mm, 1920×1200). No calibration file."
    )
    parser.add_argument("image", help="Path to image file")
    parser.add_argument("--engine", default=DEFAULT_ENGINE, help="Path to TensorRT engine")
    parser.add_argument("--conf", type=float, default=0.25, help="Confidence threshold")
    parser.add_argument("--out", default="", help="Output image path")
    parser.add_argument("--no-save", action="store_true", help="Do not save result image")
    parser.add_argument("--override-class", type=str, default="", help="Use this class for ref/label for all detections (e.g. red_buoy)")
    args = parser.parse_args()
    override_class = args.override_class.strip() or None

    image_path = os.path.abspath(args.image)
    if not os.path.isfile(image_path):
        print(f"Error: Image not found: {image_path}")
        sys.exit(1)

    id_to_name = load_class_mapping(CLASS_MAPPING_PATH)
    if not id_to_name:
        print("Warning: class_mapping.yaml not found; class names will be class_N.")

    image = cv2.imread(image_path)
    if image is None:
        print(f"Error: Could not load image {image_path}")
        sys.exit(1)
    orig_h, orig_w = image.shape[:2]
    scale_x = orig_w / 640.0
    scale_y = orig_h / 640.0

    # Calibration from specs only (no .npz)
    calib = get_calib_from_specs(orig_w, orig_h)
    print(f"Using camera spec: {SENSOR_WIDTH_PX}×{SENSOR_HEIGHT_PX}, {PIXEL_SIZE_UM} µm, f={FOCAL_LENGTH_MM} mm")
    print(f"  fy (at image size {orig_w}×{orig_h}): {calib['K'][1, 1]:.1f} px")

    print(f"Loading TensorRT engine: {args.engine}")
    inferencer = TensorRTInference(args.engine)
    input_data = inferencer.preprocess(image)
    output = inferencer.infer(input_data)
    detections = inferencer.postprocess_yolo(output, conf_threshold=args.conf, debug=False)

    print(f"\nDetections: {len(detections)}")
    print("-" * 60)

    for det in detections:
        box = det["box"]
        x1 = box[0] * scale_x
        y1 = box[1] * scale_y
        x2 = box[2] * scale_x
        y2 = box[3] * scale_y
        bbox_orig = [x1, y1, x2, y2]
        class_id = int(det["class_id"])
        class_name = override_class or id_to_name.get(class_id, f"class_{class_id}")
        ref_m = REFERENCE_DIMENSIONS.get(class_name, DEFAULT_REFERENCE_M)
        dist_m = estimate_distance_m(bbox_orig, orig_w, orig_h, calib, ref_m)
        dist_ft = dist_m * M_TO_FT
        print(f"  {class_name}: {dist_m:.2f} m  ({dist_ft:.2f} ft)  [ref={ref_m:.3f} m, conf={det['score']:.2f}]")

    img_out = image.copy()
    for det in detections:
        box = det["box"]
        x1 = int(box[0] * scale_x)
        y1 = int(box[1] * scale_y)
        x2 = int(box[2] * scale_x)
        y2 = int(box[3] * scale_y)
        class_id = int(det["class_id"])
        class_name = override_class or id_to_name.get(class_id, f"class_{class_id}")
        ref_m = REFERENCE_DIMENSIONS.get(class_name, DEFAULT_REFERENCE_M)
        bbox_orig = [x1, y1, x2, y2]
        dist_m = estimate_distance_m(bbox_orig, orig_w, orig_h, calib, ref_m)
        dist_ft = dist_m * M_TO_FT
        cv2.rectangle(img_out, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label = f"{class_name}: {dist_m:.1f}m ({dist_ft:.1f}ft)"
        cv2.putText(img_out, label, (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    if not args.no_save:
        out_path = args.out or os.path.join(os.path.dirname(image_path), "result_specs_only.jpg")
        cv2.imwrite(out_path, img_out)
        print(f"\nResult saved to: {out_path}")

    return detections


if __name__ == "__main__":
    main()

