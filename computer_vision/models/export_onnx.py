#!/usr/bin/env python3
"""Export YOLO .pt weights to ONNX for TensorRT conversion.
   Usage: python export_onnx.py [path_to.pt]
   Default: aqua_main.pt -> aqua_main.onnx (input 640x640, FP32)."""

import argparse
import os

from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser(description="Export YOLO .pt to ONNX")
    parser.add_argument(
        "weights",
        nargs="?",
        default=os.path.join(os.path.dirname(__file__), "..", "model_building_and_training", "aqua_main.pt"),
        help="Path to .pt weights (default: model_building_and_training/aqua_main.pt)",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=640,
        help="Input size (default: 640)",
    )
    args = parser.parse_args()

    path = os.path.abspath(args.weights)
    if not os.path.isfile(path):
        raise FileNotFoundError(f"Weights not found: {path}")

    model = YOLO(path)
    # Export to ONNX (same dir as .pt, name from stem)
    model.export(format="onnx", imgsz=args.imgsz)
    out = os.path.splitext(path)[0] + ".onnx"
    print(f"Exported: {out}")
    return out


if __name__ == "__main__":
    main()

