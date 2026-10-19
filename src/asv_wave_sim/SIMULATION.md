# Full Boat Simulation Stack – Complete Guide

This document covers running the **complete boat simulation:** Gazebo Sim + ArduPilot SITL + MAVROS + ROS 2 bridges. The boat floats on water, is controlled by ArduPilot (Rover), and can be driven via MAVProxy or ROS 2.

**For a minimal test (Gazebo + waves only, no boat), see [README.md](README.md).**

*last updated on aug 15th 2026 by Nathaniel FD
---

## Table of Contents

1. [What This Is](#what-this-is)
2. [Architecture](#architecture)
3. [External Dependencies](#external-dependencies)
4. [How to Run](#how-to-run)
5. [How to Control the Boat](#how-to-control-the-boat)
6. [Running with Computer Vision](#running-with-computer-vision)
7. [Ports and Communication](#ports-and-communication)
8. [Issues We Hit and How We Fixed Them](#issues-we-hit-and-how-we-fixed-them)
9. [Troubleshooting](#troubleshooting)

---

## What This Is

The full simulation stack consists of:

- **Gazebo Sim (gz sim)** – Runs the aquatonomous boat world (`aquatonomous_world.sdf`) with water, buoys, gates, and the **ourboat** model.
- **ros_gz_bridge** – Bridges Gazebo topics (lidar, cameras) to ROS 2.
- **MAVROS** (ROS 2) – Talks to ArduPilot over MAVLink.
- **ArduPilot SITL** (Rover, JSON model) – Simulates the autopilot; receives sensor data from Gazebo and sends motor commands back.

**Goal:** Run the sim, connect MAVROS to SITL, arm the boat, and send velocity/position setpoints so the boat moves in Gazebo.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│  YOU                                                                            │
│  • MAVProxy (terminal): mode GUIDED, arm throttle, position/velocity            │
│  • ROS 2 (terminal): /mavros/*, setpoints, services                             │
└─────────────────────────────────────────────────────────────────────────────────┘
         │                                    │
         ▼                                    ▼
┌──────────────────────┐           ┌──────────────────────┐
│  MAVROS (ROS 2)      │           │  ArduPilot SITL      │
│  • fcu_url :=        │  TCP      │  • Listens 5760      │
│    tcp://localhost:  │◄─────────►│  • Rover, JSON model │
│    5760              │  5760     │  • Sends to 9002     │
└──────────────────────┘           └──────────┬───────────┘
         │                                    │ UDP (JSON FDM)
         │                                    │ pose/IMU ← → motor cmds
         │                                    ▼
         │                         ┌──────────────────────┐
         │                         │  ArduPilotPlugin     │
         │                         │  (in Gazebo, ourboat)│
         │                         │  • Listens 127.0.0.1:│
         │                         │    9002              │
         │                         │  • Drives thrusters  │
         │                         └──────────┬───────────┘
         │                                    │
         │                                    ▼
         │                         ┌──────────────────────┐
         │                         │  Gazebo Sim          │
         │                         │  • World + water     │
         │                         │  • ourboat model     │
         │                         │  • WavesModel,       │
         │                         │    Hydrodynamics     │
         │                         └──────────┬───────────┘
         │                                    │
         │                                    │ topics (lidar, cameras)
         ▼                                    ▼
┌──────────────────────┐           ┌──────────────────────┐
│  ros_gz_bridge       │◄─────────►│  Gazebo topics       │
│  • /laser_points     │           │  • /lidar_topic/     │
│  • /camera0..2/      │           │    points, camera*   │
│    image_raw         │           │                      │
└──────────────────────┘           └──────────────────────┘
```

**Summary:**

- **MAVROS ↔ SITL:** TCP on port **5760**. MAVROS is the ROS 2 interface to the autopilot.
- **SITL ↔ Gazebo:** UDP on port **9002**. The ArduPilotPlugin receives pose/IMU and sends motor commands; the plugin applies thrust so the boat moves.
- **Gazebo ↔ ROS 2:** ros_gz_bridge republishes Gazebo topics as ROS 2 (lidar, cameras).

Gazebo must be running **before** SITL starts so the plugin binds to 9002 when SITL connects.

---

## External Dependencies

The full boat simulation requires these components (they live outside the `autonomy-ws-25-26` repository):

> **Before building anything below:** export `GZ_VERSION=harmonic` in your
> shell first (`export GZ_VERSION=harmonic`, or add it to `~/.bashrc`).
> `gz-waves` and other Gazebo-dependent builds select which Gazebo version's
> libraries to link against based on this variable at **build time** — not
> just at launch time. If it's unset during `colcon build`, the build will
> silently target the wrong Gazebo version (Garden instead of Harmonic) and
> fail with "missing dependency" errors that don't obviously point back to
> this cause.

> **Note:** the scripts in this repo check both `~/Repos/School/<name>` and
> `~/<name>` for each external dependency below and use whichever exists.
> This doc uses `~/<name>` for brevity, but either location works.

| Component | Path | Purpose |
|-----------|------|---------|
| **SITL_Models** | `~/SITL_Models/Gazebo/models/ourboat` | Boat model (mesh, SDF, plugins) |
| **ardupilot_gazebo** | `~/ardupilot_gazebo/build/` | ArduPilotPlugin (connects SITL to Gazebo) |
| **ardupilot** | `~/ardupilot/` | ArduPilot SITL binary (simulated autopilot) |
| **bridge_ws** | `~/bridge_ws/` | ros_gz_bridge (Gazebo ↔ ROS 2 for Jazzy) |
| **bridge_deps_ws** | `~/bridge_deps_ws/` | Optional; message deps for ros_gz_bridge (actuator_msgs, gps_msgs, vision_msgs) if not from apt |
| **MAVProxy** | `~/.local/bin/mavproxy.py` | MAVProxy (command-line ground station) |

### 1. SITL_Models (Boat Model)

**What:** The `ourboat` Gazebo model (mesh, SDF, sensors, thrusters, ArduPilotPlugin config).

**Where it should be:** `~/SITL_Models/Gazebo/models/ourboat/`

**Structure:**
```
~/SITL_Models/
└── Gazebo/
    ├── models/
    │   └── ourboat/
    │       ├── model.sdf       # Main model definition
    │       ├── model.config    # Gazebo model metadata
    │       └── meshes/         # 3D meshes (DAE, STL, etc.)
    └── worlds/
        └── (optional world files)
```

**Check if you have it:**
```bash
ls -la ~/SITL_Models/Gazebo/models/ourboat/

```
**If `ourboat` is missing:** `SITL_Models` (per ArduPilot's own repo) only
ships stock models like `blueboat`. `ourboat` is aQuatonomous's own
customized BlueBoat hull (renamed model + fixed mesh URIs — see Issue #4
below), and a pre-built copy already lives inside this repo. Copy it into
place rather than rebuilding it from scratch:

```bash
mkdir -p ~/SITL_Models/Gazebo/models
cp -r ~/Repos/School/autonomy-ws-25-26/src/asv_wave_sim/gz-waves-models/models/ourboat \
      ~/SITL_Models/Gazebo/models/ourboat
```


### 2. ardupilot_gazebo (ArduPilotPlugin)

**What:** The ArduPilot Gazebo plugin that connects ArduPilot SITL to Gazebo physics.

**Where it should be:** `~/ardupilot_gazebo/build/libArduPilotPlugin.so`

**How to get it:**
```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

**Important:** The plugin must be built under **your user** (not root, not another user).

### 3. ardupilot (SITL)

**What:** ArduPilot Software-In-The-Loop (SITL) – the simulated autopilot.

**Where it should be:** `~/ardupilot/` (can be a symlink)

**How to get it:**
```bash
cd ~
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
./waf configure --board sitl
./waf rover
```

**If the repo is under another user's directory:**
1. Symlink it to your home: `ln -s /home/other_user/ardupilot ~/ardupilot`
2. Add it as a Git safe directory: `git config --global --add safe.directory /home/other_user/ardupilot`

### 4. bridge_ws (ros_gz_bridge for Jazzy)

**What:** The `ros_gz_bridge` package that bridges Gazebo topics to ROS 2.

**Where it should be:** `~/bridge_ws/install/`

**Dependencies:** `ros_gz_bridge` needs `actuator_msgs`, `gps_msgs`, and `vision_msgs`. If `colcon build` fails to find them, build a deps workspace first:

```bash
# Optional: only if bridge_ws build fails with missing actuator_msgs/gps_msgs/vision_msgs
mkdir -p ~/bridge_deps_ws/src && cd ~/bridge_deps_ws/src
git clone https://github.com/rudislabs/actuator_msgs.git
git clone https://github.com/swri-robotics/gps_umd.git -b ros2-devel
git clone https://github.com/ros-perception/vision_msgs.git -b jazzy

cd ~/bridge_deps_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select gps_msgs actuator_msgs vision_msgs
```

**How to get it:**
```bash
mkdir -p ~/bridge_ws/src
cd ~/bridge_ws/src
git clone https://github.com/gazebosim/ros_gz.git -b jazzy   # NOT -b ros2 — that tracks ROS Rolling and fails to build against Jazzy which is what we r using

cd ~/bridge_ws
source /opt/ros/jazzy/setup.bash
[ -f ~/bridge_deps_ws/install/setup.bash ] && source ~/bridge_deps_ws/install/setup.bash
colcon build --symlink-install
```

**Verify:**
```bash
source /opt/ros/jazzy/setup.bash
[ -f ~/bridge_deps_ws/install/setup.bash ] && source ~/bridge_deps_ws/install/setup.bash
source ~/bridge_ws/install/setup.bash
ros2 pkg list | grep ros_gz_bridge
```

### 5. MAVProxy

**What:** Command-line ground station for ArduPilot.

**Where it should be:** `~/.local/bin/mavproxy.py`

**How to get it:**
```bash
pip install --user MAVProxy
```

**Verify:**
```bash
ls -la ~/.local/bin/mavproxy.py
mavproxy.py --version
```

**Ensure `~/.local/bin` is in your PATH:**
```bash
export PATH="$HOME/.local/bin:$PATH"
```

---

## System Build Dependencies

Before building any of the external dependencies above, install these
system packages. None of them are covered by the ROS 2 Jazzy or Gazebo
Harmonic installs on their own.

```bash
sudo apt install -y \
  libcgal-dev libfftw3-dev libogre-next-2.3-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  libportaudio19-dev \
  ros-jazzy-mavros ros-jazzy-mavros-extras \
  ros-jazzy-ament-cmake-clang-format \
  tmux
```

MAVROS also needs the GeographicLib datasets:
```bash
sudo bash /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
```

**Only one Gazebo Harmonic install should be present on the machine.**
Having both the system `gz-harmonic` apt metapackage and ROS 2's own
`ros-jazzy-gz-sim-vendor` installed at once causes a version mismatch between
Gazebo's GUI and server processes, which breaks resource-path resolution in
confusing ways. If you hit this, remove the standalone system package and
keep only the ROS-vendored copy:
```bash
sudo apt remove --purge gz-harmonic gz-sim8-cli libgz-sim8 libgz-sim8-dev \
  libgz-sim8-plugins python3-gz-sim8
sudo apt autoremove
```

---

> **Prerequisite:** the full stack requires the **whole workspace** to be
> built, not just `src/asv_wave_sim` (which is enough for the Stage 1 basic
> standalone test in [README.md](README.md)). From the workspace root:
> ```bash
> cd ~/Repos/School/autonomy-ws-25-26
> rm -rf build install log   # if taking over an existing/stale checkout
> export GZ_VERSION=harmonic
> ./build.sh
> ```
> **If this repo was previously checked out and built by someone else on
> this machine,** always wipe `build/`, `install/`, and `log/` first. CMake
> caches absolute paths from the original build machine and fails with a
> confusing "CMakeCache.txt directory is different" error otherwise.

## How to Run

### Option A: Single Command with tmux (Recommended)

The easiest way to start everything:

```bash
cd ~/Repos/School/autonomy-ws-25-26
bash ./src/asv_wave_sim/run_full_simulation_tmux.bash
```

This starts a tmux session with three windows:
- **Window 0:** Gazebo + 4 bridge panes (lidar, camera1, camera2, camera3)
- **Window 1:** MAVROS + shell
- **Window 2:** ArduPilot SITL + MAVProxy

To Switch windows select termina then use `Ctrl+b` then `0`, `1`, or `2`.

### Option B: Manual Launch in Separate Terminals

For more control and easier debugging, run each component in its own terminal. **Start them in this order and wait between steps:**

#### Terminal 1 – Gazebo (wait for "Loaded plugins" message)

```bash
cd ~/Repos/School/autonomy-ws-25-26
source /opt/ros/jazzy/setup.bash
source install/setup.bash

export GZ_VERSION=harmonic
export LD_LIBRARY_PATH="$HOME/Repos/School/autonomy-ws-25-26/install/gz-waves1/lib:$LD_LIBRARY_PATH"export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="\
$HOME/Repos/School/autonomy-ws-25-26/src/asv_wave_sim/gz-waves-models/models:\
$HOME/Repos/School/autonomy-ws-25-26/src/asv_wave_sim/gz-waves-models/world_models:\
$HOME/SITL_Models/Gazebo/models:\
$HOME/SITL_Models/Gazebo/worlds:\
$HOME/ardupilot_gazebo/models:\
$HOME/ardupilot_gazebo/worlds:\
${GZ_SIM_RESOURCE_PATH:-}"

cd ~/Repos/School/autonomy-ws-25-26/src/asv_wave_sim/gz-waves-models/worlds
gz sim -v 4 -r aquatonomous_world.sdf
```

**Wait for Gazebo to fully load and show entities before proceeding.**

#### Terminal 2 – ros_gz_bridge (Lidar) [Start ~10s after Gazebo]

```bash
source /opt/ros/jazzy/setup.bash
[ -f ~/bridge_deps_ws/install/setup.bash ] && source ~/bridge_deps_ws/install/setup.bash
source ~/bridge_ws/install/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  /lidar_topic/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked \
  --ros-args -r /lidar_topic/points:=/laser_points
```

#### Terminal 3 – ros_gz_bridge (Camera 1)

```bash
source /opt/ros/jazzy/setup.bash
[ -f ~/bridge_deps_ws/install/setup.bash ] && source ~/bridge_deps_ws/install/setup.bash
source ~/bridge_ws/install/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  /camera1_topic@sensor_msgs/msg/Image@gz.msgs.Image \
  --ros-args -r /camera1_topic:=/camera0/image_raw
```

#### Terminal 4 – ros_gz_bridge (Camera 2)

```bash
source /opt/ros/jazzy/setup.bash
[ -f ~/bridge_deps_ws/install/setup.bash ] && source ~/bridge_deps_ws/install/setup.bash
source ~/bridge_ws/install/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  /camera2_topic@sensor_msgs/msg/Image@gz.msgs.Image \
  --ros-args -r /camera2_topic:=/camera1/image_raw
```

#### Terminal 5 – ros_gz_bridge (Camera 3)

```bash
source /opt/ros/jazzy/setup.bash
[ -f ~/bridge_deps_ws/install/setup.bash ] && source ~/bridge_deps_ws/install/setup.bash
source ~/bridge_ws/install/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  /camera3_topic@sensor_msgs/msg/Image@gz.msgs.Image \
  --ros-args -r /camera3_topic:=/camera2/image_raw
```

#### Terminal 6 – MAVROS [Start ~35s after Gazebo]

```bash
source /opt/ros/jazzy/setup.bash

until (echo >/dev/tcp/localhost/5760) 2>/dev/null; do 
  echo "Waiting for SITL on port 5760..."; 
  sleep 2; 
done

ros2 launch mavros apm.launch fcu_url:=tcp://localhost:5760
```

#### Terminal 7 – ArduPilot SITL + MAVProxy [Start ~40s after Gazebo]

```bash
export PATH="$HOME/.local/bin:$PATH"
export PYTHONPATH="$HOME/.local/lib/python3.10/site-packages:$PYTHONPATH"

cd ~/ardupilot/Tools/autotest
python3 ./sim_vehicle.py -v Rover -f rover-skid --model JSON \
  --console --map \
  --custom-location="51.566151,-4.034345,10.0,-135"
```

---

## How to Control the Boat

### In MAVProxy (after SITL is up)

For simulation, you must set the arming parameter **first**, then switch mode and arm:

```text
param set ARMING_REQUIRE 0
mode GUIDED
arm throttle
```

Then drive the boat:

```text
position 10 0 0     # 10 m north (NED: North, East, Down)
velocity 2 0 0      # 2 m/s north
position 5 5 0      # 5 m north, 5 m east
```

**Do NOT use `guided lat lon` in MAVProxy for Rover**—use `position` / `velocity` or send global setpoints via ROS 2/MAVROS.

### From ROS 2

Global position setpoint example:
```bash
ros2 topic pub /mavros/setpoint_position/global geographic_msgs/msg/GeoPoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {latitude: 51.5662, longitude: -4.0343, altitude: 0.0}}}" --once
```

Use MAVROS services for mode, arm, etc., as needed.

---

## Running with Computer Vision

The sim bridges publish **`/camera0/image_raw`**, **`/camera1/image_raw`**, **`/camera2/image_raw`** from Gazebo. You can run the computer vision pipeline on these topics without real cameras.

> **Note on sensor rates:** Camera/lidar topic rates depend on Gazebo's
> real-time factor (RTF) on your machine, which is often well under 100% for
> the full stack (physics + rendering + 4 bridges + SITL running together).
> Expect update rates in the low single-digit Hz with some jitter on typical
> development hardware, not a clean fixed rate. This isn't a bug, check
> `gz sim`'s stats panel for the current RTF if rates seem unexpectedly low.

**Order:** Start the **simulation first**, then in a **second terminal** start the CV launch.

**Launch CV (sim-only):**
```bash
source /opt/ros/jazzy/setup.bash
source ~/Repos/School/autonomy-ws-25-26/computer_vision/install/setup.bash
ros2 launch cv_ros_nodes launch_cv_sim.py
```

**On Jetson — use one camera to avoid GPU OOM:**
```bash
ros2 launch cv_ros_nodes launch_cv_sim.py single_camera:=true
```

This runs **camera1 (center)** only. Combine with lower confidence to detect more sim buoys:
```bash
ros2 launch cv_ros_nodes launch_cv_sim.py single_camera:=true conf_threshold:=0.15
```

**Viewing detections:** In RViz, add an **Image** display and set the topic to **`/camera1/detections`**.

For more details, see the computer vision documentation.

---

## Ports and Communication

| Connection | Port | Protocol | Description |
|------------|------|----------|-------------|
| MAVROS ↔ SITL | 5760 | TCP | MAVLink; MAVROS uses `fcu_url:=tcp://localhost:5760` |
| SITL ↔ ArduPilotPlugin | 9002 | UDP | JSON FDM: pose/IMU from Gazebo, motor commands from SITL |
| MAVProxy ↔ SITL | 5760 | TCP | MAVProxy's `--master tcp:127.0.0.1:5760` |

---

## Issues We Hit and How We Fixed Them

### 1. Boat fell through the water

**Symptom:** Boat spawned then fell straight down through the water.

**Cause:** WavesModel and Hydrodynamics plugins did not load.

**Fix:**
- Add the waves install lib path to `LD_LIBRARY_PATH`:
  ```bash
  export LD_LIBRARY_PATH=$HOME/Repos/School/autonomy-ws-25-26/install/gz-waves1/lib:$LD_LIBRARY_PATH
  ```
- In `~/SITL_Models/Gazebo/models/ourboat/model.sdf`, ensure Hydrodynamics plugin has:
  ```xml
  <enable>ourboat::base_link</enable>
  ```

### 2. Boat did not move in Gazebo

**Symptom:** MAVProxy and SITL worked, but the 3D boat in Gazebo stayed still.

**Cause:** ArduPilotPlugin was not loaded or not built for the current user.

**Fix:**
- Ensure `GZ_SIM_SYSTEM_PLUGIN_PATH` includes `$HOME/ardupilot_gazebo/build`.
- Clean rebuild of the plugin under your user:
  ```bash
  cd ~/ardupilot_gazebo && rm -rf build && mkdir build && cd build
  cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && make -j4
  ```
- Verify `libArduPilotPlugin.so` exists in `~/ardupilot_gazebo/build/`.

### 3. MAVProxy not found

**Symptom:** SITL pane showed "No such file or directory: 'mavproxy.py'".

**Cause:** When tmux runs the SITL command, `PATH` did not include `~/.local/bin`.

**Fix:** Set in the script:
```bash
export PATH="$HOME/.local/bin:$PATH"
export PYTHONPATH="$HOME/.local/lib/python3.10/site-packages:$PYTHONPATH"
```

### 4. Model name mismatch (ourboat vs blueboat)

**Symptom:** Boat doesn't move or plugins don't apply.

**Cause:** SDF had wrong model name or mesh URIs.

**Fix:** In `~/SITL_Models/Gazebo/models/ourboat/model.sdf`:
- Set `<model name="ourboat">`.
- Replace all `models://blueboat/` with `models://ourboat/`.
- Set Hydrodynamics `<enable>ourboat::base_link</enable>`.

### 5. Pre-arm: "AP ARM gyros inconsistent", "need position estimate"

**Symptom:** `arm throttle` refused.

**Cause:** Rover arming checks were enabled.

**Fix:** In MAVProxy:
```text
param set ARMING_REQUIRE 0
mode GUIDED
arm throttle
```

### 6. Wrong MAVROS port (connection refused)

**Symptom:** MAVROS failed to connect.

**Cause:** Script or docs used `tcp://localhost:5763`; SITL listens on **5760**.

**Fix:** Use `fcu_url:=tcp://localhost:5760` everywhere.

### 7. SITL build fails: Permission denied on `.wafpickle` (Jetson)

**Symptom:** `PermissionError: [Errno 13] Permission denied: '.../ardupilot/build/sitl/.wafpickle-...'`

**Cause:** `ardupilot` was built with `sudo` or owned by another user.

**Fix:**
```bash
cd ~/ardupilot
sudo chown -R $USER:$USER .
rm -rf build/sitl
./waf configure --board sitl
./waf rover
```

### 8. Gazebo black or white screen / "could not connect to display" (Jetson)

**Symptom:** Gazebo window doesn't open or stays black/white.

**Cause:** No display, or EGL/GPU/ogre2 issues on Jetson.

**Fix:**
- **Run on the Jetson from a local graphical session** with a monitor attached.
- If you're at the Jetson but shell has no display: `export DISPLAY=:0`
- For black/white: try adding `--render-engine-gui ogre` to the `gz sim` line; ensure NVIDIA drivers are installed.

### 9. Gazebo shows "0 entities" (empty world)

**Symptom:** Gazebo window opens but shows 0 entities.

**Cause:** The world or its included models failed to load. `GZ_SIM_RESOURCE_PATH` is missing required model dirs.

**Fix:**
1. Ensure `GZ_SIM_RESOURCE_PATH` includes:
   - `.../gz-waves-models/models` and `.../gz-waves-models/world_models` (for `model://waves`, etc.)
   - `~/SITL_Models/Gazebo/models` (for `model://ourboat`)
   - `~/ardupilot_gazebo/models` and `~/ardupilot_gazebo/worlds`
2. Ensure directories exist on your machine.
3. Check Gazebo output for "could not find model" errors.

### 10. "DO SET MODE failed: ap: flight mode change failed" (GUIDED / arm)

**Symptom:** In MAVProxy, `mode GUIDED` and/or `arm throttle` fail.

**Cause:** ArduPilot is refusing the mode change, usually because it has no position estimate or the world failed to load.

**Fix:**
1. Fix "0 entities" first (see above).
2. In MAVProxy run: `param set ARMING_REQUIRE 0`, then `mode GUIDED` and `arm throttle`.

### 11. Gazebo "0 entities" + libEGL / Mesa / nvidia-drm warnings (Jetson)

**Symptom:** Gazebo shows 0 entities and libEGL / MESA warnings.

**Cause:** Display/rendering stack using Mesa instead of NVIDIA driver.

**Fix:**
1. Force NVIDIA DRM mode: `sudo modprobe nvidia-drm modeset=1`
2. Ensure `LD_LIBRARY_PATH` starts with `/usr/lib/aarch64-linux-gnu/nvidia`
3. Try: `export __GLX_VENDOR_LIBRARY_NAME=nvidia`

### 12. SITL / MAVProxy: NumPy 2.x / matplotlib crash

**Symptom:** SITL pane shows `A module that was compiled using NumPy 1.x cannot be run in NumPy 2.x` and MAVProxy (or sim_vehicle.py) exits.

**Cause:** MAVProxy imports matplotlib, which can conflict with NumPy 2.x.

**Fix:**
- Run the tmux script without MAVProxy GUI: `SITL_NO_GUI=1 bash run_full_simulation_tmux.bash` (SITL runs without `--console --map`).
- Or install NumPy 1.x in the environment used for SITL: `pip install 'numpy<2'` (in a venv or user install that MAVProxy uses).

### 13. OGRE2 wave-shader plugin fails to load (cosmetic, not blocking — do not "fix" via LD_LIBRARY_PATH)

**Symptom:** Gazebo logs `Failed to load plugin [gz-waves1-rendering-ogre2]` /
`libOgreNextMain.so.2.3.1: cannot open shared object file`.

**Cause:** This is a rendering-only plugin (fancier wave shading) — the
simulation runs correctly without it. The library it wants
(`/usr/lib/x86_64-linux-gnu/OGRE-2.3/`, v2.3.1) is a **different build** of
Ogre-Next than the one `gz-rendering8` already uses internally (the
ROS-vendored copy, v2.3.3).

**Do NOT add `/usr/lib/x86_64-linux-gnu/OGRE-2.3` to `LD_LIBRARY_PATH`** —
this was tried and makes things worse: it forces two separate Ogre-Next
instances into the same process, and the plugin hard-crashes both the
Gazebo server and GUI with `Ogre::MeshManager::getSingleton(): Assertion
'msSingleton' failed`, rather than just failing to load.

**Current status:** unresolved, low priority. The plugin failing to load
silently (original behavior) is the safer outcome — leave it alone. A real
fix would need `gz-waves1-rendering-ogre2` rebuilt against the same
Ogre-Next 2.3.3 build the rest of Gazebo uses, not a `LD_LIBRARY_PATH`
workaround.
---

## Troubleshooting

| Symptom | What to check |
|---------|----------------|
| Boat falls through water | `LD_LIBRARY_PATH` includes `.../install/gz-waves1/lib` (and `~/ardupilot_gazebo/build`); Hydrodynamics `<enable>` is `ourboat::base_link` |
| Boat doesn't move in Gazebo | `GZ_SIM_SYSTEM_PLUGIN_PATH` includes `~/ardupilot_gazebo/build`; plugin rebuilt for current user; SITL starts after Gazebo (40s delay) |
| MAVProxy not found | Script exports `PATH` and `PYTHONPATH` for SITL; `~/.local/bin/mavproxy.py` exists |
| MAVROS connection refused | SITL listening on 5760; use `fcu_url:=tcp://localhost:5760` |
| Can't arm: position / gyros | For sim: `param set ARMING_REQUIRE 0` then `mode GUIDED` and `arm throttle` |
| Gazebo black or white screen | `LD_LIBRARY_PATH` for NVIDIA libs (Jetson); ensure NVIDIA drivers installed |
| SITL build: Permission denied .wafpickle | `cd ~/ardupilot && sudo chown -R $USER:$USER . && rm -rf build/sitl && ./waf configure --board sitl && ./waf rover` |
| Gazebo shows 0 entities | `GZ_SIM_RESOURCE_PATH` must include all model dirs; check Gazebo pane for "could not find model" errors |
| Gazebo 0 entities + libEGL / Mesa / nvidia-drm | Run `sudo modprobe nvidia-drm modeset=1`; ensure `LD_LIBRARY_PATH` has NVIDIA libs first |
| DO SET MODE failed / stuck on MANUAL | **Set the param first:** `param set ARMING_REQUIRE 0`, then `mode GUIDED`, then `arm throttle` |
| SITL crashes: NumPy / matplotlib | Run with `SITL_NO_GUI=1` or use `numpy<2` for MAVProxy |

---

## Summary

The full boat simulation is ready to run. Choose one of these two approaches:

1. **Quick start (tmux):** `cd ~/Repos/School/autonomy-ws-25-26 && bash ./src/asv_wave_sim/run_full_simulation_tmux.bash`
2. **Manual (separate terminals):** Follow the "How to Run" section above.

With everything working, the boat floats, arms, and moves in Gazebo under MAVProxy and ROS 2 control.
