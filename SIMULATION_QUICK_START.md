# Boat Simulation Quick Start

Use this guide for the simulator. Ignore the repo-level `README.md` if your goal is just to launch the boat sim.

## Run Everything

From the repo root:

```bash
cd ~/Repos/School/autonomy-ws-25-26
bash ./src/asv_wave_sim/run_full_simulation_tmux.bash
```

This is the main launcher. It starts:

- Gazebo with the water, map/course, and boat
- ROS-Gazebo bridges for lidar and cameras
- ArduPilot SITL with MAVProxy
- MAVROS

## What Opens

- `tmux` window `0`: Gazebo and bridge panes
- `tmux` window `1`: SITL and MAVProxy
- `tmux` window `2`: MAVROS

Useful tmux keys:

- `Ctrl+b 0` for Gazebo
- `Ctrl+b 1` for SITL
- `Ctrl+b 2` for MAVROS
- `Ctrl+b o` to move between panes
- `Ctrl+b d` to detach and leave it running
- `Ctrl+b &` to kill the session

## What You Should See

- Gazebo opens with water, waves, the course objects, and the `ourboat` model
- The boat should now show a visible deck / bridge structure, not just the twin hulls
- Camera topics and lidar bridge into ROS 2
- MAVProxy starts printing vehicle status
- MAVROS connects to SITL on `localhost:5760`

Quick checks:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Repos/School/autonomy-ws-25-26/install/setup.bash

ros2 topic list | grep -E "camera|laser|mavros"
```

Expected topics include:

- `/camera0/image_raw`
- `/camera1/image_raw`
- `/camera2/image_raw`
- `/laser_points`

## If You Only Want One Piece

These scripts are for debugging single parts of the stack:

- `src/asv_wave_sim/test_gazebo_gui.sh`: Gazebo only
- `src/asv_wave_sim/test_bridges.sh`: ROS-Gazebo bridges only
- `src/asv_wave_sim/test_sitl.sh`: ArduPilot SITL only
- `src/asv_wave_sim/test_mavros.sh`: MAVROS only

If you just want the full simulator, use `run_full_simulation_tmux.bash` and ignore the `test_*.sh` scripts.

## Common Fixes

If `gz sim` is missing after sourcing ROS 2:

```bash
export GZ_CONFIG_PATH="/usr/share/gz:${GZ_CONFIG_PATH:-}"
```

If SITL does not come up right away:

- wait at least 35 to 40 seconds after Gazebo starts
- make sure `~/Repos/School/ardupilot` or `~/ardupilot` exists

If Gazebo opens but the boat is missing:

- make sure `~/Repos/School/SITL_Models/Gazebo/models/ourboat` exists

## More Detail

- Full sim docs: `src/asv_wave_sim/SIMULATION.md`
- Simulator package docs: `src/asv_wave_sim/README.md`
