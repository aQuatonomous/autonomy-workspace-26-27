#!/usr/bin/env python3
"""Detect red/orange buoy in image via color, output bbox and height_px for calibration."""
import argparse
import sys
import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser(description="Get bounding box of red/orange buoy for calibration")
    parser.add_argument("image", help="Path to image")
    parser.add_argument("--output", "-o", metavar="PATH", help="Save image with bounding box drawn to PATH")
    parser.add_argument(
        "--right-half",
        action="store_true",
        help="Only consider red/orange blobs in the right half of the image (center x > width/2)",
    )
    args = parser.parse_args()
    img = cv2.imread(args.image)
    if img is None:
        print("Could not read image", args.image, file=sys.stderr)
        return 1
    h, w = img.shape[:2]
    print(f"Image size: {w} x {h}")

    # HSV: red/orange buoy (two ranges for red wrap-around)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    low1 = np.array([0, 100, 100])
    high1 = np.array([15, 255, 255])
    low2 = np.array([160, 100, 100])
    high2 = np.array([180, 255, 255])
    mask1 = cv2.inRange(hsv, low1, high1)
    mask2 = cv2.inRange(hsv, low2, high2)
    mask = cv2.bitwise_or(mask1, mask2)
    # Orange range (common for "red" buoys)
    low_o = np.array([10, 120, 120])
    high_o = np.array([25, 255, 255])
    mask_o = cv2.inRange(hsv, low_o, high_o)
    mask = cv2.bitwise_or(mask, mask_o)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        print("No red/orange region found. Try adjusting color or provide bbox manually.", file=sys.stderr)
        return 1
    # Optionally restrict to right half (center x > w/2) to avoid picking a close buoy on the left
    if args.right_half:
        candidates = []
        for c in contours:
            rx, ry, rw, rh = cv2.boundingRect(c)
            if rw < 5 or rh < 5:
                continue
            cx = rx + rw / 2.0
            if cx > w / 2.0:
                candidates.append(c)
        contours = candidates if candidates else contours
    # Largest contour by area
    best = max(contours, key=cv2.contourArea)
    x, y, bw, bh = cv2.boundingRect(best)
    # Sanity: object should be a reasonable fraction of image
    if bw < 5 or bh < 5:
        print("Detected region too small.", file=sys.stderr)
        return 1
    height_px = bh
    print(f"bbox (x,y,w,h): {x}, {y}, {bw}, {bh}")
    print(f"height_px (for distance): {height_px}")
    if args.output:
        out = img.copy()
        cv2.rectangle(out, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
        cv2.putText(out, f"height_px={height_px}", (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imwrite(args.output, out)
        print(f"Saved bbox image: {args.output}")
    print()
    print("Use with compute_distance_scale.py:")
    print(f"  python3 computer_vision/model_testing/compute_distance_scale.py --measured-m MEASURED_M --reference-m REF_M --height-px {height_px}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
