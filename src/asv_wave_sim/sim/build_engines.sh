#!/bin/bash
# Builds the CV TensorRT engines for THIS machine's GPU + TensorRT version from the repo's ONNX
# models. Engines are GPU- and TensorRT-specific, so they go into a cache keyed by both
# ($CV_ENGINE_CACHE/<gpu>_trt<version>/); the Jetson engines in computer_vision/ are untouched.
#
#   build_engines.sh             build any engine missing for this machine
#   build_engines.sh --force     rebuild
#   build_engines.sh --check     exit 0 when both engines exist for this machine
#   build_engines.sh --print-dir print the engine directory for this machine
set -e
SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$SIM_DIR/../../.." && pwd)"
[ -f "$SIM_DIR/config.local.env" ] && . "$SIM_DIR/config.local.env"
. "$SIM_DIR/config.env"
[ -d /usr/lib/wsl/lib ] && export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}/usr/lib/wsl/lib"

engine_dir() {
  local gpu trt
  gpu="$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -n 1 | tr -cs 'A-Za-z0-9.' '_' | sed 's/_$//')"
  trt="$(python3 -c 'import tensorrt; print(tensorrt.__version__)' 2>/dev/null)"
  [ -n "$gpu" ] && [ -n "$trt" ] || return 1
  echo "$CV_ENGINE_CACHE/${gpu}_trt${trt}"
}

DIR="$(engine_dir)" || { echo "Needs an NVIDIA GPU (nvidia-smi) and TensorRT for Python (setup_sim.sh --with-cv)." >&2; exit 1; }
MAIN_ONNX="$WS/computer_vision/model_building_and_training/aqua_main.onnx"
NUM_ONNX="$WS/computer_vision/task_specific/Docking/number_detection/number_detection.onnx"
MAIN="$DIR/aqua_main.engine"
NUM="$DIR/number_detection.engine"

case "${1:-}" in
  --print-dir) echo "$DIR"; exit 0 ;;
  --check) [ -s "$MAIN" ] && [ -s "$NUM" ]; exit $? ;;
esac
FORCE=0; [ "${1:-}" = "--force" ] && FORCE=1

TRTEXEC="${TRTEXEC:-/usr/src/tensorrt/bin/trtexec}"
[ -x "$TRTEXEC" ] || { echo "trtexec not found at $TRTEXEC (package libnvinfer-bin)." >&2; exit 1; }
mkdir -p "$DIR"

build() {  # onnx engine
  if [ -s "$2" ] && [ $FORCE = 0 ]; then echo "up to date: $2"; return; fi
  [ -f "$1" ] || { echo "ONNX model missing: $1" >&2; exit 1; }
  echo "Building $(basename "$2") from $(basename "$1") (FP16; a few minutes)..."
  "$TRTEXEC" --onnx="$1" --saveEngine="$2.tmp" --fp16 --memPoolSize=workspace:2048 --skipInference >"$2.log" 2>&1 ||
    { echo "trtexec failed; see $2.log" >&2; rm -f "$2.tmp"; exit 1; }
  mv "$2.tmp" "$2"
  echo "  -> $2"
}

build "$MAIN_ONNX" "$MAIN"
build "$NUM_ONNX" "$NUM"
echo "Engines for this machine: $DIR"
