#!/usr/bin/env python3
"""
Estimate distance and angle from forward + left camera images using YOLO
(aqua_main.pt) and the same formulas as maritime_distance_estimator and
vision_combiner (AR0234 specs, camera mounting angles).

No HSV/color detection — model-only.

Usage:
  python3 estimate_distance_angle.py --forward IMG_FORWARD.jpg --left IMG_LEFT.jpg
  python3 estimate_distance_angle.py --forward IMG_FORWARD.jpg --left IMG_LEFT.jpg --model camera_calibration/aqua_main.pt --conf 0.25 --draw
"""

import argparse
import math
import os
import sys
from pathlib import Path

import cv2
import yaml
from ultralytics import YOLO

# --- Same as maritime_distance_estimator.py ---
PIXEL_SIZE_UM = 3.0
FOCAL_LENGTH_MM = 3.56
SENSOR_WIDTH_PX = 1920
SENSOR_HEIGHT_PX = 1200
HORIZONTAL_FOV_DEG = 85.0
VERTICAL_FOV_DEG = 69.0

FY_PX = FOCAL_LENGTH_MM / (PIXEL_SIZE_UM / 1000.0)
CX_PX = SENSOR_WIDTH_PX / 2.0
CY_PX = SENSOR_HEIGHT_PX / 2.0

# --- Same as vision_combiner.py: camera 0=left, 1=center(forward), 2=right ---
CAMERA_MOUNTING_ANGLES_DEG = [-70.0, 0.0, 70.0]

# Reference dimensions (m) — same as maritime_distance_estimator
REFERENCE_DIMENSIONS = {
    "black_buoy": 0.254,
    "green_buoy": 0.254,
    "red_buoy": 0.254,
    "yellow_buoy": 0.254,
    "green_pole_buoy": 0.991,
    "red_pole_buoy": 0.991,
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
DEFAULT_REFERENCE_M = 0.305


def get_effective_fy(frame_width: int, frame_height: int) -> float:
    """Scale focal length if frame size differs from native 1920×1200."""
    if frame_width == SENSOR_WIDTH_PX and frame_height == SENSOR_HEIGHT_PX:
        return FY_PX
    scale_y = frame_height / SENSOR_HEIGHT_PX
    return FY_PX * scale_y


def get_fx_fy_from_fov(frame_width: int, frame_height: int):
    """Combiner-style intrinsics from FOV (for bearing/elevation)."""
    fx = frame_width / (2.0 * math.tan(math.radians(HORIZONTAL_FOV_DEG / 2.0)))
    fy = frame_height / (2.0 * math.tan(math.radians(VERTICAL_FOV_DEG / 2.0)))
    cx = frame_width / 2.0
    cy = frame_height / 2.0
    return fx, fy, cx, cy


def estimate_distance_m(bbox, frame_width: int, frame_height: int, reference_m: float, scale_factor: float = 1.0) -> float:
    """Same formula as maritime_distance_estimator: distance_m = (fy_eff * reference_m) / height_px * scale."""
    x1, y1, x2, y2 = bbox[0], bbox[1], bbox[2], bbox[3]
    height_px = max(1.0, y2 - y1)
    fy_eff = get_effective_fy(frame_width, frame_height)
    distance_m = (fy_eff * reference_m) / height_px
    distance_m = distance_m * scale_factor
    return max(0.01, distance_m)


def compute_bearing_elevation(camera_id: int, bbox, frame_width: int, frame_height: int) -> dict:
    """Same as vision_combiner._compute_bearing_and_elevation."""
    fx, fy, cx, cy = get_fx_fy_from_fov(frame_width, frame_height)
    u_center = (bbox[0] + bbox[2]) / 2.0
    v_center = (bbox[1] + bbox[3]) / 2.0
    camera_angle_rad = math.atan2(u_center - cx, fx)
    camera_angle_deg = math.degrees(camera_angle_rad)
    mounting_deg = CAMERA_MOUNTING_ANGLES_DEG[camera_id]
    bearing_deg = mounting_deg + camera_angle_deg
    elevation_rad = math.atan2(cy - v_center, fy)
    elevation_deg = math.degrees(elevation_rad)
    return {
        "bearing_deg": bearing_deg,
        "elevation_deg": elevation_deg,
        "camera_angle_deg": camera_angle_deg,
        "mounting_angle_deg": mounting_deg,
    }


def load_class_mapping():
    """Load class_id -> class_name from class_mapping.yaml (fallback: model names)."""
    for base in [Path(__file__).resolve().parent.parent, Path.cwd()]:
        for rel in ["class_mapping.yaml"]:
            path = base / rel
            if path.is_file():
                try:
                    with open(path) as f:
                        data = yaml.safe_load(f)
                    classes = data.get("classes") or {}
                    return {int(k): str(v) for k, v in classes.items()}
                except Exception:
                    pass
    return {}


def process_image(path: str, camera_id: int, model, id_to_name: dict,
                  scale_factor: float, conf_threshold: float,
                  draw_path: str = None):
    """Load image, run YOLO, compute distance and angle for each detection. Returns list of dicts."""
    if not isinstance(model, YOLO):
        model = YOLO(str(model))
    img = cv2.imread(path)
    if img is None:
        print(f"Could not read image: {path}", file=sys.stderr)
        return []
    h, w = img.shape[:2]
    results_list = model.predict(
        source=img,
        conf=conf_threshold,
        verbose=False,
    )
    results = []
    mounting_deg = CAMERA_MOUNTING_ANGLES_DEG[camera_id]
    out_img = img.copy() if draw_path else None
    for r in results_list:
        if r.boxes is None or len(r.boxes) == 0:
            continue
        names = r.names or {}
        for b in r.boxes:
            cls_id = int(b.cls.item())
            confidence = float(b.conf.item())
            xyxy = [float(x) for x in b.xyxy.squeeze(0).tolist()]
            x1, y1, x2, y2 = xyxy
            bbox = xyxy
            height_px = max(1.0, y2 - y1)
            class_name = id_to_name.get(cls_id) or names.get(cls_id, f"class_{cls_id}")
            reference_m = REFERENCE_DIMENSIONS.get(class_name, DEFAULT_REFERENCE_M)
            dist_m = estimate_distance_m(bbox, w, h, reference_m, scale_factor)
            dist_ft = dist_m * 3.28084
            angle_info = compute_bearing_elevation(camera_id, bbox, w, h)
            rec = {
                "bbox": bbox,
                "height_px": height_px,
                "class_name": class_name,
                "class_id": cls_id,
                "distance_m": dist_m,
                "distance_ft": dist_ft,
                "bearing_deg": angle_info["bearing_deg"],
                "elevation_deg": angle_info["elevation_deg"],
                "camera_angle_deg": angle_info["camera_angle_deg"],
                "mounting_angle_deg": mounting_deg,
                "confidence": confidence,
            }
            results.append(rec)
            if out_img is not None:
                ix1, iy1 = int(x1), int(y1)
                ix2, iy2 = int(x2), int(y2)
                cv2.rectangle(out_img, (ix1, iy1), (ix2, iy2), (0, 255, 0), 2)
                label = f"{class_name} d={dist_m:.2f}m b={angle_info['bearing_deg']:.1f}° c={confidence:.2f}"
                cv2.putText(out_img, label, (ix1, iy1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    if draw_path and out_img is not None:
        cv2.imwrite(draw_path, out_img)
    return results


def main():
    ap = argparse.ArgumentParser(description="Estimate distance and angle from forward + left camera images (YOLO only)")
    ap.add_argument("--forward", required=True, help="Path to forward (center) camera image")
    ap.add_argument("--left", required=True, help="Path to left camera image")
    ap.add_argument("--model", default=None,
                    help="Path to YOLO .pt model (default: camera_calibration/aqua_main.pt)")
    ap.add_argument("--conf", type=float, default=0.25, help="YOLO confidence threshold (default 0.25)")
    ap.add_argument("--scale", type=float, default=0.87,
                    help="Distance scale factor from one-point calibration (default 0.87)")
    ap.add_argument("--draw", action="store_true", help="Save images with bboxes and labels")
    args = ap.parse_args()

    forward_path = Path(args.forward)
    left_path = Path(args.left)
    if not forward_path.is_file():
        print(f"Error: Forward image not found: {forward_path}", file=sys.stderr)
        return 1
    if not left_path.is_file():
        print(f"Error: Left image not found: {left_path}", file=sys.stderr)
        return 1

    # Default model: camera_calibration/aqua_main.pt (next to this script or from CV root)
    script_dir = Path(__file__).resolve().parent
    cv_root = script_dir.parent
    if args.model:
        model_path = Path(args.model)
    else:
        model_path = script_dir / "aqua_main.pt"
        if not model_path.is_file():
            model_path = cv_root / "camera_calibration" / "aqua_main.pt"
        if not model_path.is_file():
            model_path = cv_root / "model_building_and_training" / "aqua_main.pt"
    if not model_path.is_file():
        print(f"Error: Model not found: {model_path}", file=sys.stderr)
        return 1

    id_to_name = load_class_mapping()
    model = YOLO(str(model_path))

    forward_results = process_image(
        str(forward_path), camera_id=1, model=model, id_to_name=id_to_name,
        scale_factor=args.scale, conf_threshold=args.conf,
        draw_path=str(forward_path.parent / "angle_test_forward_out.jpg") if args.draw else None,
    )
    left_results = process_image(
        str(left_path), camera_id=0, model=model, id_to_name=id_to_name,
        scale_factor=args.scale, conf_threshold=args.conf,
        draw_path=str(left_path.parent / "angle_test_left_out.jpg") if args.draw else None,
    )

    print("=" * 60)
    print("Distance & angle (YOLO aqua_main.pt, same formulas as maritime_distance_estimator + vision_combiner)")
    print("  Model: {}  |  conf threshold: {}".format(model_path, args.conf))
    print("  Scale factor: {}  |  Camera 0 = left (-70.0°)  |  Camera 1 = forward (0°)".format(args.scale))
    print("=" * 60)
    print()
    print("FORWARD camera (center, mounting 0°) —", args.forward)
    if not forward_results:
        print("  No detections.")
    else:
        for i, r in enumerate(forward_results):
            print("  Object {}:  class = {}  |  distance = {:.2f} m ({:.1f} ft)  |  bearing = {:.1f}°  |  elevation = {:.1f}°  |  confidence = {:.2f}  (height_px = {})".format(
                i + 1, r["class_name"], r["distance_m"], r["distance_ft"], r["bearing_deg"], r["elevation_deg"], r["confidence"], r["height_px"]))
    print()
    print("LEFT camera (mounting -70.0°) —", args.left)
    if not left_results:
        print("  No detections.")
    else:
        for i, r in enumerate(left_results):
            print("  Object {}:  class = {}  |  distance = {:.2f} m ({:.1f} ft)  |  bearing = {:.1f}°  |  elevation = {:.1f}°  |  confidence = {:.2f}  (height_px = {})".format(
                i + 1, r["class_name"], r["distance_m"], r["distance_ft"], r["bearing_deg"], r["elevation_deg"], r["confidence"], r["height_px"]))
    print()
    if args.draw and (forward_results or left_results):
        print("Drew bboxes and labels (--draw): angle_test_forward_out.jpg, angle_test_left_out.jpg")
    return 0


if __name__ == "__main__":
    sys.exit(main())
