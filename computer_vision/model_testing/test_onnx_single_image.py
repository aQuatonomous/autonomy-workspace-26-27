"""
inspect_engine.py
"""
import os
import tensorrt as trt

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CV_ROOT = os.path.dirname(SCRIPT_DIR)
engine_path = os.path.join(CV_ROOT, "model_building_and_training", "model.engine")

logger = trt.Logger(trt.Logger.INFO)
with open(engine_path, "rb") as f:
    engine_data = f.read()

runtime = trt.Runtime(logger)
engine = runtime.deserialize_cuda_engine(engine_data)

print("== Bindings ==")
# TensorRT 10.x uses num_io_tensors instead of num_bindings
for i in range(engine.num_io_tensors):
    name = engine.get_tensor_name(i)
    shape = engine.get_tensor_shape(name)
    dtype = engine.get_tensor_dtype(name)
    mode = engine.get_tensor_mode(name)
    io = "Input" if mode == trt.TensorIOMode.INPUT else "Output"
    print(f"{i}: {name} | {io} | shape={shape} | dtype={dtype}")

