#!/usr/bin/env python3
"""
Multi-point distance calibration: read a CSV of (image_path, measured_m, reference_m),
detect buoy in each image to get height_px, compute scale factor per row, then average.

CSV format (header required):
  image_path,measured_m,reference_m[,right_half]

Optional right_half: 1, true, or yes = only detect buoy in the right half of the image (use when a closer buoy on the left would be detected otherwise).

Paths in image_path are relative to the CSV's directory or repo root.
"""
import argparse
import csv
import os
import subprocess
import sys
from typing import Optional

# Same as maritime_distance_estimator / compute_distance_scale
FY_PX = 3.56 / (3.0 / 1000.0)


def get_height_px_for_image(
    script_dir: str, repo_root: str, image_path: str, right_half: bool = False
) -> Optional[float]:
    """Run get_buoy_bbox.py on image; return height_px or None on failure."""
    resolved = image_path if os.path.isabs(image_path) else os.path.normpath(
        os.path.join(repo_root, image_path)
    )
    if not os.path.isfile(resolved):
        return None
    cmd = [sys.executable, os.path.join(script_dir, "get_buoy_bbox.py"), resolved]
    if right_half:
        cmd.append("--right-half")
    try:
        out = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=30,
            cwd=repo_root,
        )
    except Exception:
        return None
    if out.returncode != 0:
        return None
    for line in out.stdout.splitlines():
        if line.strip().startswith("height_px (for distance):"):
            try:
                return float(line.split(":")[-1].strip())
            except ValueError:
                return None
    return None


def main():
    parser = argparse.ArgumentParser(
        description="Compute average distance_scale_factor from multiple calibration photos (CSV)"
    )
    parser.add_argument(
        "csv_path",
        help="Path to CSV with columns: image_path, measured_m, reference_m",
    )
    parser.add_argument(
        "--repo-root",
        default=os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        help="Repo root for resolving image_path (default: computer_vision parent)",
    )
    args = parser.parse_args()
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_dir = os.path.dirname(os.path.abspath(args.csv_path))

    if not os.path.isfile(args.csv_path):
        print(f"Error: CSV not found: {args.csv_path}", file=sys.stderr)
        return 1

    rows = []
    with open(args.csv_path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames or "image_path" not in reader.fieldnames:
            print("Error: CSV must have header with image_path, measured_m, reference_m", file=sys.stderr)
            return 1
        for row in reader:
            image_path = (row.get("image_path") or "").strip()
            try:
                measured_m = float((row.get("measured_m") or "0").strip())
                reference_m = float((row.get("reference_m") or "0").strip())
            except ValueError:
                continue
            if not image_path or measured_m <= 0 or reference_m <= 0:
                continue
            rh = (row.get("right_half") or "0").strip().lower() in ("1", "true", "yes")
            rows.append({
                "image_path": image_path,
                "measured_m": measured_m,
                "reference_m": reference_m,
                "right_half": rh,
            })

    if not rows:
        print("Error: No valid rows in CSV", file=sys.stderr)
        return 1

    scales = []
    for i, r in enumerate(rows):
        height_px = get_height_px_for_image(
            script_dir, args.repo_root, r["image_path"], right_half=r.get("right_half", False)
        )
        if height_px is None or height_px < 1:
            print(f"  Skip row {i+1}: could not get height_px for {r['image_path']}", file=sys.stderr)
            continue
        distance_specs = (FY_PX * r["reference_m"]) / height_px
        scale = r["measured_m"] / distance_specs
        scales.append(scale)
        print(f"  {r['image_path']}: measured={r['measured_m']} m, height_px={height_px} -> scale={scale:.4f}")

    if not scales:
        print("Error: No rows produced a valid scale factor", file=sys.stderr)
        return 1

    avg_scale = sum(scales) / len(scales)
    print()
    print("Multi-point calibration")
    print(f"  Points used: {len(scales)} / {len(rows)}")
    print(f"  Scale factors: min={min(scales):.4f}, max={max(scales):.4f}, mean={avg_scale:.4f}")
    print()
    print("Use this value when launching:")
    print(f"  ros2 launch cv_ros_nodes launch_cv.py distance_scale_factor:={avg_scale:.4f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
