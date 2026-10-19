#!/usr/bin/env bash
# Export best.pt to number_detection.onnx (imgsz=960), then build TensorRT engine in model_building_and_training/.
# Run from repo root or from this directory. Requires: Python + ultralytics, trtexec on PATH or TRTEXEC set.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CV_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
PT="$SCRIPT_DIR/best.pt"
ONNX="$SCRIPT_DIR/number_detection.onnx"
ENGINE="$CV_DIR/model_building_and_training/number_detection.engine"
TRTEXEC="${TRTEXEC:-/usr/src/tensorrt/bin/trtexec}"

if [ ! -f "$PT" ]; then
  echo "Error: best.pt not found at $PT"
  exit 1
fi

echo "Exporting best.pt -> number_detection.onnx (imgsz=960) ..."
(cd "$SCRIPT_DIR" && python3 -c "
from ultralytics import YOLO
m = YOLO('best.pt')
m.export(format='onnx', imgsz=960)
")
# Ultralytics writes best.onnx (same stem as best.pt); rename to number_detection.onnx
if [ -f "$SCRIPT_DIR/best.onnx" ]; then
  mv "$SCRIPT_DIR/best.onnx" "$ONNX"
fi
if [ ! -f "$ONNX" ]; then
  echo "Error: ONNX export failed"
  exit 1
fi
echo "ONNX: $ONNX"

echo "Building TensorRT engine (FP16) -> $ENGINE ..."
"$TRTEXEC" --onnx="$ONNX" --saveEngine="$ENGINE" --fp16 --memPoolSize=workspace:4096 --skipInference

echo "Done. Engine: $ENGINE"
