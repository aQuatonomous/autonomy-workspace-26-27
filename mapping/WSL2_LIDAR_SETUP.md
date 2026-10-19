# Unitree 4D LiDAR on Windows (WSL2 + USB passthrough)

This guide gets the Unitree 4D LiDAR publishing `/unilidar/cloud` on your Windows laptop by running the driver inside WSL2 with the LiDAR attached via USB passthrough.

---

## 1. Windows: Install usbipd and attach the LiDAR

### 1.1 Install usbipd-win

- **Option A:** Download the latest installer from [usbipd-win releases](https://github.com/dorssel/usbipd-win/releases) and run it.
- **Option B:** From an **Administrator** PowerShell:  
  `winget install usbipd`

### 1.2 Plug in the Unitree 4D LiDAR

Connect the LiDAR via USB to your laptop.

### 1.3 Bind and attach the device to WSL

Open **PowerShell or Command Prompt** (Admin for bind, normal for attach):

```powershell
# List USB devices and note the BUSID of the Unitree LiDAR (e.g. 1-4)
usbipd list

# Bind the device (one time per device; use the BUSID from the list)
usbipd bind --busid <BUSID>

# Attach it to WSL (do this each time you open WSL and want to use the LiDAR)
usbipd attach --wsl --busid <BUSID>
```

Example: if the LiDAR is on bus `1-4`:

```powershell
usbipd bind --busid 1-4
usbipd attach --wsl --busid 1-4
```

To detach when done: `usbipd detach --busid <BUSID>`.

---

## 2. WSL: Check the device and serial port

Inside your WSL terminal (e.g. Ubuntu):

```bash
# Confirm the LiDAR appears
lsusb

# Find the serial port (often ttyUSB0 or ttyACM0)
ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true
```

If you see `/dev/ttyUSB0` (or similar) but get "Permission denied" when running the driver, add your user to the `dialout` group and re-login:

```bash
sudo usermod -aG dialout $USER
# Then log out of WSL and open a new terminal
```

---

## 2b. WSL: Build the unified workspace (one-time)

From the **repo root** (this repo is now a single ROS 2 workspace):

```bash
cd ~/Repos/School/autonomy-ws-25-26
source /opt/ros/jazzy/setup.bash
./build.sh   # builds unitree_lidar_ros2, pointcloud_filters, etc.
source install/setup.bash
```

The **unitree_lidar_ros2** package lives under `src/unitree_lidar_ros2`.  
The C++ SDK is vendored at `src/unitree_lidar_sdk` (prebuilt lib used at build time). No separate clone is needed.

**After changing any Python code** (e.g. range filter, buoy detector), rebuild:  
`colcon build --symlink-install --packages-select pointcloud_filters` then `source install/setup.bash`.

---

## 4. WSL: Set the serial port and run the pipeline

The launch file defaults to `/dev/ttyUSB0`. If your LiDAR is on another port (e.g. `/dev/ttyACM0`), override with `lidar_port:=/dev/ttyACM0`.

**Single terminal – full pipeline (driver + filter + detector + tracker + RViz):**

From the **repo root** (after building):

```bash
cd ~/Repos/School/autonomy-ws-25-26
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch pointcloud_filters buoy_pipeline.launch.py launch_lidar_driver:=true
```

If the LiDAR is on a different serial port (e.g. `/dev/ttyACM0`):

```bash
ros2 launch pointcloud_filters buoy_pipeline.launch.py launch_lidar_driver:=true lidar_port:=/dev/ttyACM0
```

**Defaults:** The launch uses **+X forward** (`rotate_yaw_bias_deg:=180`) and **15° tilt correction** (`tilt_correction_deg:=15`) by default. If the wall is not on +X, try `rotate_yaw_bias_deg:=0` or `yaw_offset_deg:=±10` in the range filter.

**Alternative – run driver and pipeline separately:**

**Terminal 1 – LiDAR driver only (e.g. custom port):**

```bash
cd ~/Repos/School/autonomy-ws-25-26
source install/setup.bash
ros2 run unitree_lidar_ros2 unitree_lidar_ros2_node --ros-args -p port:=/dev/ttyACM0
```

**Terminal 2 – Pipeline without launching the driver:**

```bash
cd ~/Repos/School/autonomy-ws-25-26
source install/setup.bash
ros2 launch pointcloud_filters buoy_pipeline.launch.py launch_lidar_driver:=false
```

Check that the cloud is publishing:

```bash
ros2 topic hz /unilidar/cloud
```

**Point count:** The Unitree 4D outputs ~2k points per scan. After the 220° FOV filter you get ~1.2k points; the buoy detector then runs RANSAC (if enabled) and clusters. So a “sparse” cloud is normal for this sensor and pipeline.

---

### 4b. Unitree L1 speed and accuracy

The L1 has a **fixed scan rate** (~21,600 pts/s); there are **no driver parameters** in unilidar_sdk to increase scan rate or change hardware accuracy.

- **Faster perception / less clutter:** Reduce `range_max` (and optionally `z_max`) so fewer points are processed and latency drops.
- **Denser view:** Enable point cloud accumulation in the range filter: `enable_accumulation:=true accumulation_window:=0.2` (adds latency).
- **Accuracy:** Unitree recommends mounting the L1 on a thermally conductive plate (e.g. aluminium) with at least 10 mm clearance; no software setting improves hardware accuracy.

---

## 5. Troubleshooting

### "can't open configuration file .cyclonedds.xml" / "rmw handle is invalid"
ROS2 is using CycloneDDS and trying to load a missing config. **Fix:** use FastRTPS instead when launching (from mapping folder):
```bash
cd ~/Repos/School/autonomy-ws-25-26/mapping && source install/setup.bash && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && ros2 launch pointcloud_filters buoy_pipeline.launch.py launch_lidar_driver:=false
```

### "ModuleNotFoundError: No module named 'sklearn'"
Install scikit-learn: `sudo apt install python3-sklearn` (or re-run `./setup_wsl_build.sh`, which installs it).

### RViz crashes when moving/orbiting to view the point cloud
WSLg/OpenGL can crash under load. **Try the lighter config** (from mapping folder):

```bash
cd ~/Repos/School/autonomy-ws-25-26/mapping
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source install/setup.bash
rviz2 -d install/pointcloud_filters/share/pointcloud_filters/rviz/buoy_pipeline_wsl.rviz
```

If it still crashes, run the pipeline without RViz and inspect data in the terminal, e.g. `ros2 topic echo /buoy_detections` or `ros2 topic hz /unilidar/cloud`.

---

## 6. Other troubleshooting

| Issue | What to try |
|-------|-------------|
| No `/dev/ttyUSB*` or `/dev/ttyACM*` in WSL | After `usbipd attach`, run `lsusb`; if the device appears, install `usbipd list` and ensure the correct busid is attached. Re-attach or reboot WSL. |
| Permission denied on serial port | `sudo usermod -aG dialout $USER` and log out and back in to WSL. |
| Driver fails to open port | Confirm the port with `ls /dev/tty*` and pass it with `-p port:=/dev/ttyACM0` (or the correct device). |
| No `/unilidar/cloud` | Ensure the driver node is running and that no errors appear in its terminal. Check the driver repo for 4D-specific configuration. |
| RViz fixed frame | Set Fixed Frame to `unilidar_lidar` and add a PointCloud2 display with topic `/unilidar/cloud`. |

---

## 7. Summary

1. **Windows:** Install usbipd-win → plug LiDAR → `usbipd bind --busid <BUSID>` → `usbipd attach --wsl --busid <BUSID>`.
2. **WSL:** Verify with `lsusb` and `ls /dev/ttyUSB* /dev/ttyACM*`; add user to `dialout` if needed.
3. **WSL:** Build the unified workspace from `~/Repos/School/autonomy-ws-25-26` (driver is in `src/unitree_lidar_ros2`, SDK in `src/unitree_lidar_sdk`).
4. **WSL:** Run `buoy_pipeline.launch.py` from the root workspace (or run the driver with `port:=/dev/ttyACM0` and the pipeline with `launch_lidar_driver:=false`).
5. Check `ros2 topic hz /unilidar/cloud` and RViz with fixed frame `unilidar_lidar`.

For overrides, tuning, and troubleshooting, see [COMPETITION.md](COMPETITION.md) and [README.md](README.md).
