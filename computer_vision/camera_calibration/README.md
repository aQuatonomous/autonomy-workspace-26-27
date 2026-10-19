# Camera calibration (one-point distance)

## Current calibration

| | |
|---|--|
| **Distance scale factor** | **0.8700** |
| Calibration points | 6 photos |
| Reference object | 10 in buoy (0.254 m) |
| Data | `calibration_measurements.csv` |

**Launch with:**
```bash
ros2 launch cv_ros_nodes launch_cv.py distance_scale_factor:=0.8700
```

To recompute after adding photos to the CSV:
```bash
python3 computer_vision/camera_calibration/calibrate_from_csv.py computer_vision/camera_calibration/calibration_measurements.csv
```

---

## What is "distance scale factor"?

The pipeline estimates distance with a **pinhole formula** using the camera's specs (focal length, pixel size). That formula often doesn't match the real world exactly (lens, mounting, etc.), so we correct it with **one number**: the **distance scale factor**.

- **Meaning:** "Multiply every estimated distance by this number to get the corrected distance."
- **1.0** = no change (use the formula as-is).
- **&lt; 1.0** (e.g. **0.77**): the formula was *over*estimating distance; we shrink estimates so they match reality.
- **&gt; 1.0**: the formula was *under*estimating; we increase estimates.

We get that number by comparing **one (or more) real measurement** to what the formula gives:  
**scale_factor = measured_distance_m / distance_from_formula**.

Example: you measure the buoy at **3 m**, but the formula says **3.9 m** → scale = 3 / 3.9 ≈ **0.77**. After calibration, all distances are multiplied by 0.77.

---

## One photo

1. Take one photo with the object at a **known distance** and **known size** (e.g. buoy 118 in away, 10 in tall).
2. Get bbox height in pixels:
   ```bash
   cd computer_vision
   python3 camera_calibration/get_buoy_bbox.py camera_calibration/WIN_xxx.jpg
   ```
   If the wrong red/orange object is detected (e.g. a closer buoy on the left), use `--right-half` to only consider blobs in the right half of the image:
   ```bash
   python3 camera_calibration/get_buoy_bbox.py camera_calibration/WIN_xxx.jpg --right-half
   ```
   Use `-o out.jpg` to save an image with the bbox drawn so you can verify.
3. Compute scale factor (use the `height_px` from step 2 and your measured distance/size in meters):
   ```bash
   python3 computer_vision/model_testing/compute_distance_scale.py --measured-m 2.997 --reference-m 0.254 --height-px 77
   ```
4. Use the printed `distance_scale_factor` when launching (e.g. `distance_scale_factor:=0.7657`).

---

## Multiple photos (better calibration)

Put one row per photo in `calibration_measurements.csv`:

```csv
image_path,measured_m,reference_m,right_half
camera_calibration/WIN_20260214_15_46_18_Pro.jpg,2.9972,0.254,0
camera_calibration/photo_5m.jpg,5.08,0.254,1
```

- **image_path**: path to the image (relative to repo root or absolute).
- **measured_m**: distance from camera to object in **meters**.
- **reference_m**: real height (or width) of the object in **meters** (e.g. 0.254 for 10 inches).
- **right_half** (optional): set to `1` or `true` if the target buoy is on the right and a closer one on the left would otherwise be detected; then only the right half of the image is used for detection.

Then run (from repo root):

```bash
python3 computer_vision/camera_calibration/calibrate_from_csv.py computer_vision/camera_calibration/calibration_measurements.csv
```

The script finds the buoy in each image, computes a scale factor per photo, and prints the **average** scale factor and the launch command. Use that single value for `distance_scale_factor` when you run the pipeline.
