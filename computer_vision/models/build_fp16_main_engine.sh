#!/bin/bash
# Build FP16 TensorRT engine for faster inference (~2x speed, minimal accuracy loss).
# Run from repo root or computer_vision:  bash computer_vision/models/build_fp16_main_engine.sh
# Output: computer_vision/model_building_and_training/model.engine (overwrites existing).

set -e
CV_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$CV_ROOT"
ONNX="${CV_ROOT}/model_building_and_training/aqua_main.onnx"
ENGINE="${CV_ROOT}/model_building_and_training/model.engine"
TRTEXEC="/usr/src/tensorrt/bin/trtexec"

if [[ ! -f "$ONNX" ]]; then
  echo "ONNX not found: $ONNX"
  echo "Export first: python computer_vision/models/export_onnx.py computer_vision/model_building_and_training/aqua_main.pt"
  exit 1
fi
if [[ ! -x "$TRTEXEC" ]]; then
  echo "trtexec not found: $TRTEXEC"
  exit 1
fi

echo "Building FP16 engine (this can take 5–15 min on Jetson)..."
"$TRTEXEC" --onnx="$ONNX" --saveEngine="$ENGINE" --fp16 --memPoolSize=workspace:4096 --skipInference
echo "Done. Engine saved to: $ENGINE"
ls -la "$ENGINE"

