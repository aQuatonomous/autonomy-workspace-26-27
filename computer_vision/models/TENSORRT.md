# TensorRT Setup and Model Conversion

Machine-specific TensorRT usage, ONNX→engine conversion, and where artifacts live. For the full pipeline design (train → ONNX → engine → validate → deploy), see [README.md](../README.md) and [NODES.md](../NODES.md).

**Pipeline (summary):** `.pt` → `.onnx` (export) → `.engine` (trtexec) → `vision_inference` / launch.

---

## What is installed

- **TensorRT Python**: 10.3.0
- **trtexec**: `/usr/src/tensorrt/bin/trtexec` (path may differ on x86 Think vs Jetson)
- **CUDA**: 12.6 (nvidia-smi)
- **GPU**: Development on Think (x86 + NVIDIA GPU); deployment target e.g. NVIDIA Orin (Jetson)

---

## trtexec

### Option 1: Full path

```bash
/usr/src/tensorrt/bin/trtexec --onnx=model.onnx --saveEngine=model.engine
```

### Option 2: Add to PATH (recommended)

In `~/.bashrc`:

```bash
export PATH=$PATH:/usr/src/tensorrt/bin
```

Then `source ~/.bashrc` and run:

```bash
trtexec --onnx=model.onnx --saveEngine=model.engine
```

---

## ONNX → TensorRT engine

**TensorRT 10:** use `--memPoolSize=workspace:N` instead of deprecated `--workspace=N`.

```bash
# FP32 (default)
/usr/src/tensorrt/bin/trtexec --onnx=aqua_main.onnx --saveEngine=model.engine --memPoolSize=workspace:4096 --skipInference

# FP16 (faster, good accuracy on Jetson/Think) — used for model_building_and_training/model.engine
/usr/src/tensorrt/bin/trtexec --onnx=aqua_main.onnx --saveEngine=model.engine --fp16 --memPoolSize=workspace:4096 --skipInference

# INT8 (fastest; requires calibration)
/usr/src/tensorrt/bin/trtexec --onnx=aqua_main.onnx --saveEngine=model_int8.engine --int8 --memPoolSize=workspace:4096 --skipInference
```

**Precision:** FP32 = highest accuracy; FP16 = ~2× faster, <1% loss; INT8 = ~4× faster, needs calibration.

---

## Python runtime (load and run engine)

```python
import tensorrt as trt

with open('model.engine', 'rb') as f:
    engine_data = f.read()

runtime = trt.Runtime(trt.Logger(trt.Logger.WARNING))
engine = runtime.deserialize_cuda_engine(engine_data)
context = engine.create_execution_context()
# Allocate buffers and run inference (see TensorRT Python API)
```

---

## Where things live

| Path | Role |
|------|------|
| `model_building_and_training/aqua_main.pt` | Current trained PyTorch weights (YOLO) |
| `model_building_and_training/aqua_main.onnx` | ONNX (export via `python models/export_onnx.py model_building_and_training/aqua_main.pt`) |
| `model_building_and_training/weights.pt` | Previous weights (kept as single old record) |
| `models/export_onnx.py` | Export .pt → .onnx |
| `model_testing/test_inference.py` | TensorRT inference and validation on images/video |
| `model_testing/test_onnx_single_image.py` | ONNX validation |
| `model_building_and_training/model.engine` | TensorRT engine used by `vision_inference` |
| `vision_inference --engine_path`, launch `engine_path:=` | Override engine path |

---

## Engine specifications (YOLO)

- **Input:** `[1, 3, 640, 640]`, `float32`, CHW, values in [0, 1]
- **Output:** `[1, 27, 8400]` (example), `float32`, detection predictions

---

## Quick test

```bash
# Example with a downloaded ONNX
wget https://download.onnxruntime.ai/onnx/models/resnet50.tar.gz
tar xzf resnet50.tar.gz
/usr/src/tensorrt/bin/trtexec --onnx=resnet50/model.onnx --saveEngine=resnet50.engine
# "Engine built successfully" if okay
```

For YOLO: export `aqua_main.pt` → `aqua_main.onnx` with `python models/export_onnx.py`, build engine with trtexec (see above), then run `model_testing/test_inference.py` or `vision_inference` with `model_building_and_training/model.engine`.

---

## Number detection model (docking digits 1, 2, 3)

Separate YOLO model for docking number detection. Same directory as the main engine: `model_building_and_training/`.

**Export best.pt to ONNX (imgsz=960):**

From `task_specific/Docking/number_detection/`:

```bash
# With Ultralytics installed (one-time export)
python -c "
from ultralytics import YOLO
model = YOLO('best.pt')
model.export(format='onnx', imgsz=960)
"
# Rename best.onnx → number_detection.onnx if desired
mv best.onnx number_detection.onnx
```

Or use the script `task_specific/Docking/number_detection/build_number_engine.sh` (exports and builds).

**Build TensorRT engine (input 1×3×960×960):**

```bash
trtexec --onnx=number_detection.onnx --saveEngine=number_detection.engine --fp16 --memPoolSize=workspace:4096 --skipInference
```

Copy or build the engine into **model_building_and_training** so it lives next to `model.engine`:

```bash
cp number_detection.engine /path/to/computer_vision/model_building_and_training/number_detection.engine
```

**Runtime:** `vision_inference` loads this second engine when `enable_number_detection:=true` and `number_detection_engine` points to `model_building_and_training/number_detection.engine`. Number classes are remapped to **class_id 20 = digit 1**, **21 = digit 2**, **22 = digit 3** to avoid collision with the main model (0–8).

