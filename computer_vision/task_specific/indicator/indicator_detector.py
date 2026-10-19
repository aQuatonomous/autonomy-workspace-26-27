# indicator_detector.py

import cv2
import numpy as np
from collections import deque


# Rolling window for temporal smoothing
state_history = deque(maxlen=5)


def detect_indicator(img: np.ndarray):
    """
    Detects red/green indicator anywhere in the image.
    Input: img = BGR numpy image (full image, no ROI needed)
    Output: (state_string, confidence_float, bbox_tuple)
            bbox_tuple = (x, y, width, height) or None if not found
    """

    # Handle empty image
    if img is None or img.size == 0:
        state_history.append("none")
        return "none", 0.0, None

    # STEP 1 — COLOR MASKING (less strict thresholds to include more shades)
    B, G, R = cv2.split(img)
    
    # Red indicator: more strict thresholds (since most should be green)
    red_mask = (R > 120) & (R > G + 25) & (R > B + 25) & (G < 150) & (B < 150)
    
    # Green indicator: more lenient for bright outdoor conditions
    green_mask = (G > 80) & (G > R + 5) & (G > B + 5) & (R < 200) & (B < 200)
    
    # Combined mask for any indicator
    combined_mask = red_mask | green_mask
    
    # STEP 2 — FIND BOUNDING BOX FROM MASK
    ys, xs = np.where(combined_mask)
    
    if len(xs) == 0:
        # Fallback: ratio-based (more lenient for green detection)
        red_mask_fallback = (R > 100) & (R > G * 1.4) & (R > B * 1.4)
        green_mask_fallback = (G > 70) & (G > R * 1.1) & (G > B * 1.1)
        combined_mask = red_mask_fallback | green_mask_fallback
        ys, xs = np.where(combined_mask)
        
        if len(xs) == 0:
            # Last resort: use full ROI mean-based (caller will use _indicator_red_or_green)
            state_history.append("none")
            return "none", 0.0, None
    
    # Get bounding box coordinates
    x1, x2 = xs.min(), xs.max()
    y1, y2 = ys.min(), ys.max()
    bbox = (x1, y1, x2 - x1, y2 - y1)
    
    # STEP 3 — CLASSIFICATION (count pixels in each mask within bounding box)
    # Extract the region around the detected indicator
    roi = img[y1:y2+1, x1:x2+1]
    B_roi, G_roi, R_roi = cv2.split(roi)
    
    # Use same adjusted thresholds for classification
    red_mask_roi = (R_roi > 120) & (R_roi > G_roi + 25) & (R_roi > B_roi + 25) & (G_roi < 150) & (B_roi < 150)
    green_mask_roi = (G_roi > 80) & (G_roi > R_roi + 5) & (G_roi > B_roi + 5) & (R_roi < 200) & (B_roi < 200)
    
    # Fallback masks (favor green detection)
    if red_mask_roi.sum() == 0 and green_mask_roi.sum() == 0:
        red_mask_roi = (R_roi > 100) & (R_roi > G_roi * 1.4) & (R_roi > B_roi * 1.4)
        green_mask_roi = (G_roi > 70) & (G_roi > R_roi * 1.1) & (G_roi > B_roi * 1.1)
    
    r_votes = red_mask_roi.sum()
    g_votes = green_mask_roi.sum()
    
    # Calculate confidence
    total_votes = r_votes + g_votes
    if total_votes == 0:
        state_history.append("none")
        return "none", 0.0, bbox
    
    conf_red = r_votes / total_votes
    conf_green = g_votes / total_votes
    
    # STEP 4 — CREATE FINAL DECISION
    # Lower threshold since we're using more lenient color detection
    if max(conf_red, conf_green) < 0.2:
        state = "none"
        conf = 1.0 - max(conf_red, conf_green)
    else:
        if conf_red > conf_green:
            state = "red"
            conf = conf_red
        else:
            state = "green"
            conf = conf_green
    
    # STEP 5 — (Disabled) temporal smoothing
    # For single-image or sparse frame processing (like our debug pipeline),
    # smoothing across calls can cause the wrong colour to "stick" from
    # previous images. We keep the history deque for potential future use,
    # but return the per-image result directly here.
    state_history.append(state)
    return state, conf, bbox


def visualize_detection(img, bbox, state, confidence):
    """
    Returns the original image without any annotations.
    (Label and confidence are printed to console only)
    
    Args:
        img: Full image (BGR numpy array)
        bbox: Tuple (x, y, width, height) of bounding box, or None (unused)
        state: Detection state ("red", "green", or "none") (unused)
        confidence: Confidence value (0.0 to 1.0) (unused)
    
    Returns:
        Original image unchanged
    """
    return img.copy()


def create_mask_visualization(img):
    """
    Creates a mask visualization where only red and green pixels are shown,
    everything else becomes white.
    
    Args:
        img: Full image (BGR numpy array)
    
    Returns:
        Image with mask applied (red/green pixels visible, rest white)
    """
    B, G, R = cv2.split(img)
    
    # Create masks with same relaxed thresholds as detection
    red_mask = (R > 120) & (R > G + 20) & (R > B + 20) & (G < 140) & (B < 140)
    green_mask = (G > 120) & (G > R + 20) & (G > B + 20) & (R < 140) & (B < 140)
    
    # Fallback masks
    red_mask_fallback = (R > 100) & (R > G * 1.3) & (R > B * 1.3)
    green_mask_fallback = (G > 100) & (G > R * 1.3) & (G > B * 1.3)
    
    # Combine all masks
    red_combined = red_mask | red_mask_fallback
    green_combined = green_mask | green_mask_fallback
    combined_mask = red_combined | green_combined
    
    # Create output image: white background
    mask_img = np.ones_like(img) * 255
    
    # Copy original pixels where mask is True
    mask_img[combined_mask] = img[combined_mask]
    
    return mask_img


if __name__ == "__main__":
    import sys
    import os
    from pathlib import Path
    
    # Get script directory
    script_dir = Path(__file__).parent
    
    # Default image name or get from command line
    if len(sys.argv) > 1:
        image_path = Path(sys.argv[1])
        if not image_path.is_absolute():
            image_path = script_dir / image_path
    else:
        # Look for common image files in the same directory
        image_extensions = ['.jpg', '.jpeg', '.png', '.bmp']
        image_files = []
        for ext in image_extensions:
            image_files.extend(list(script_dir.glob(f'*{ext}')))
            image_files.extend(list(script_dir.glob(f'*{ext.upper()}')))
        
        if not image_files:
            print("Error: No image file found in the same directory.")
            print("Usage: python3 light_detection.py [image_filename]")
            print("Or place an image file (.jpg, .png, etc.) in the same directory.")
            sys.exit(1)
        
        # Use the first image found
        image_path = image_files[0]
        print(f"Using image: {image_path.name}")
    
    # Load image
    img = cv2.imread(str(image_path))
    if img is None:
        print(f"Error: Could not load image from {image_path}")
        sys.exit(1)
    
    print(f"Image loaded: {img.shape[1]}x{img.shape[0]}")
    
    # Debug: Check color ranges in image
    B, G, R = cv2.split(img)
    print(f"Color ranges - R: [{R.min()}-{R.max()}], G: [{G.min()}-{G.max()}], B: [{B.min()}-{B.max()}]")
    print(f"Color means - R: {R.mean():.1f}, G: {G.mean():.1f}, B: {B.mean():.1f}")
    
    # Run detection on entire image (no hardcoded ROI)
    state, confidence, bbox = detect_indicator(img)
    
    # Print label and confidence to console
    print(f"\nDetection Result:")
    print(f"  Label: {state.upper()}")
    print(f"  Confidence: {confidence:.3f}")
    
    # Visualize and save (no bounding box, just original image)
    result_img = visualize_detection(img, bbox, state, confidence)
    
    
    
    # Create and save mask visualization
    mask_img = create_mask_visualization(img)
    mask_output_path = script_dir / f"{image_path.stem}_mask{image_path.suffix}"
    cv2.imwrite(str(mask_output_path), mask_img)
    print(f"Mask visualization saved to: {mask_output_path}")
