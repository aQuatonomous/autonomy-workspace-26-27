# Number Detection Model - YOLO11s

A YOLO11-based object detection model trained to detect and classify numbers 1, 2, and 3 in images. This model is designed for RoboBoat competition applications where number detection is required for navigation tasks.

## Model Overview

- **Architecture**: YOLO11s (Small)
- **Task**: Object Detection
- **Classes**: 3 (Numbers: '1', '2', '3')
- **Input Size**: 960x960 pixels
- **Framework**: Ultralytics YOLO v8.3.252
- **Format**: PyTorch (.pt)

## Performance Metrics

### Final Training Results (Epoch 100)
- **mAP50**: 0.9727 (97.27%)
- **mAP50-95**: 0.8604 (86.04%)
- **Precision**: 0.9730 (97.30%)
- **Recall**: 0.9579 (95.79%)

### Best Model Performance
The `best.pt` weights file contains the model checkpoint with the best validation mAP50 score achieved during training.

## Model Specifications

### Architecture Details
- **Total Layers**: 181
- **Parameters**: 9,428,953
- **Gradients**: 9,428,937
- **GFLOPs**: 21.6
- **Model Size**: ~18 MB (best.pt)

### Training Configuration
- **Epochs**: 100
- **Batch Size**: 8
- **Image Size**: 960x960
- **Optimizer**: Auto (AdamW)
- **Learning Rate**: 
  - Initial (lr0): 0.01
  - Final (lrf): 0.01
- **Momentum**: 0.937
- **Weight Decay**: 0.0005
- **Warmup Epochs**: 3.0
- **Patience**: 50 (early stopping)

### Data Augmentation
- **Mosaic**: 1.0 (enabled)
- **Mixup**: 0.0 (disabled)
- **Copy-Paste**: 0.0 (disabled)
- **Horizontal Flip**: 0.5 (50% probability)
- **HSV-Hue**: 0.015
- **HSV-Saturation**: 0.7
- **HSV-Value**: 0.4
- **Auto Augment**: RandAugment
- **Erasing**: 0.4

### Loss Functions
- **Box Loss Weight**: 7.5
- **Class Loss Weight**: 0.5
- **DFL Loss Weight**: 1.5

## Dataset Information

- **Training Images**: 859
- **Validation Images**: 183
- **Test Images**: 190
- **Total Images**: 1,232
- **Dataset Source**: Roboflow (numbers-123, version 8)
- **License**: CC BY 4.0

### Class Distribution
- Class 0: '1'
- Class 1: '2'
- Class 2: '3'

## Usage

### Basic Inference

```python
from ultralytics import YOLO

# Load the model
model = YOLO('best.pt')

# Run inference on an image
results = model('path/to/image.jpg')

# Process results
for result in results:
    boxes = result.boxes
    for box in boxes:
        class_id = int(box.cls[0])
        confidence = float(box.conf[0])
        class_name = ['1', '2', '3'][class_id]
        print(f"Detected: {class_name} (confidence: {confidence:.2f})")
```

### Script (recommended)

Run the provided script on the included test image:

```bash
cd /home/lorenzo/Repos/School/autonomy-ws-25-26/computer_vision/task_specific/Docking/number_detection
python3 number_detection.py --model best.pt --source test_numbers.png --conf 0.25 --imgsz 960
```

### Command Line Inference

```bash
# Single image
yolo predict model=best.pt source=image.jpg conf=0.25

# Directory of images
yolo predict model=best.pt source=path/to/images/ conf=0.25

# Video
yolo predict model=best.pt source=video.mp4 conf=0.25
```

### Recommended Settings
- **Confidence Threshold**: 0.25 (default) or 0.5 for higher precision
- **IOU Threshold**: 0.7 (default)
- **Max Detections**: 300 (default)

## Hardware Requirements

### Training
- **GPU**: NVIDIA GPU with CUDA support (tested on RTX 2070 Max-Q)
- **VRAM**: Minimum 6GB (8GB recommended)
- **RAM**: 16GB+ recommended
- **Storage**: ~2GB for dataset + checkpoints

### Inference
- **GPU**: Optional (CPU inference supported but slower)
- **RAM**: 4GB+ recommended
- **Inference Speed**: 
  - GPU: ~45-50ms per image (960x960)
  - CPU: ~200-500ms per image (varies by hardware)

## File Structure

```
number_model_export/
├── README.md                    # This file
├── best.pt                      # Best model weights (recommended)
├── test_image_1.jpg            # Test output image 1
└── test_image_2.jpg            # Test output image 2
```

## Training History

The model was trained for 100 epochs with the following progression:
- **Initial Performance** (Epoch 1): mAP50 = 0.2626
- **Mid Training** (Epoch 50): mAP50 = 0.9512
- **Final Performance** (Epoch 100): mAP50 = 0.9727

Training was performed with:
- Automatic Mixed Precision (AMP) enabled
- Disk caching for dataset
- 2 worker processes
- Deterministic training (seed=0)

## Limitations & Notes

1. **Image Size**: Model is trained on 960x960 images. For best results, resize input images to this size or use YOLO's automatic resizing.

2. **Class Scope**: Model only detects numbers 1, 2, and 3. For other numbers, retraining is required.

3. **Environment**: Model trained on water/boat scenes. Performance may vary on different backgrounds.

4. **Confidence Threshold**: Lower thresholds (0.25) may produce more detections but with potential false positives. Higher thresholds (0.5+) improve precision but may miss some detections.

## Test Results

The model was tested on two RoboBoat competition images:
- **Image 1**: Detected numbers '2' and '3' (see test_image_1.jpg)
- **Image 2**: Detected number '2' (see test_image_2.jpg)

Note: These test images may contain numbers that differ from the detected classes, indicating potential areas for model improvement or dataset expansion.

## License

- **Model**: Trained model weights are provided for use in RoboBoat competition
- **Dataset**: CC BY 4.0 (from Roboflow)
- **Framework**: Ultralytics YOLO (AGPL-3.0)

## Citation

If you use this model, please cite:
- Ultralytics YOLO: https://github.com/ultralytics/ultralytics
- Dataset: Roboflow numbers-123 dataset v8

## Contact & Support

For questions or issues related to this model, please refer to:
- Ultralytics Documentation: https://docs.ultralytics.com
- YOLO Community: https://community.ultralytics.com

---

**Model Version**: 1.0  
**Training Date**: January 2026  
**Framework Version**: Ultralytics 8.3.252  
**PyTorch Version**: 2.5.1+cu121
