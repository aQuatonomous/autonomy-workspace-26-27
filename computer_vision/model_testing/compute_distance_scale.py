#!/usr/bin/env python3
"""
Compute one-point calibration scale factor for the maritime distance estimator.

Given: measured distance to object (m), real object height (m), bbox height (px).
Output: distance_scale_factor = measured_distance_m / distance_specs
        and the ros2 run command to use it.

Usage:
  python3 compute_distance_scale.py --measured-m 3.0 --reference-m 0.305 --height-px 120
  python3 compute_distance_scale.py --measured-m 2.87 --reference-m 0.305 --height-px 126
"""

import argparse

# AR0234: same as maritime_distance_estimator.py
FY_PX = 3.56 / (3.0 / 1000.0)  # ~1186.67


def main():
    parser = argparse.ArgumentParser(
        description="Compute distance_scale_factor for one-point calibration"
    )
    parser.add_argument(
        "--measured-m",
        type=float,
        required=True,
        help="Measured distance from camera to object (meters)",
    )
    parser.add_argument(
        "--reference-m",
        type=float,
        required=True,
        help="Real height (or width) of object in meters (e.g. 0.305 for 1 ft buoy)",
    )
    parser.add_argument(
        "--height-px",
        type=float,
        required=True,
        help="Bounding box height in pixels (y2 - y1 from detection)",
    )
    args = parser.parse_args()

    if args.height_px < 1:
        print("Error: height_px must be >= 1")
        return 1

    distance_specs = (FY_PX * args.reference_m) / args.height_px
    scale = args.measured_m / distance_specs

    print("One-point calibration")
    print("  measured_distance_m:", args.measured_m)
    print("  reference_height_m: ", args.reference_m)
    print("  height_px:          ", args.height_px)
    print("  distance_specs:     ", f"{distance_specs:.3f} m")
    print("  distance_scale_factor:", f"{scale:.4f}")
    print()
    print("Run the distance estimator with:")
    print(
        f'  ros2 run cv_ros_nodes maritime_distance_estimator --ros-args -p distance_scale_factor:={scale:.4f}'
    )
    return 0


if __name__ == "__main__":
    exit(main())

