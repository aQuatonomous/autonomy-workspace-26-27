# Wave Sim – Gazebo Waves and Surface Vessels

[![Ubuntu Jammy CI](https://github.com/srmainwaring/asv_wave_sim/actions/workflows/ubuntu-jammy-ci.yml/badge.svg)](https://github.com/srmainwaring/asv_wave_sim/actions/workflows/ubuntu-jammy-ci.yml)
[![macOS Ventura CI](https://github.com/srmainwaring/asv_wave_sim/actions/workflows/macos13-ventura-ci.yml/badge.svg)](https://github.com/srmainwaring/asv_wave_sim/actions/workflows/macos13-ventura-ci.yml)
[![Cpplint](https://github.com/srmainwaring/asv_wave_sim/actions/workflows/ccplint.yml/badge.svg)](https://github.com/srmainwaring/asv_wave_sim/actions/workflows/ccplint.yml)
[![Cppcheck](https://github.com/srmainwaring/asv_wave_sim/actions/workflows/ccpcheck.yml/badge.svg)](https://github.com/srmainwaring/asv_wave_sim/actions/workflows/ccpcheck.yml)

This package contains plugins that support the simulation of waves and surface vessels in [Gazebo](https://gazebosim.org/home).

The main branch targets [Gazebo Harmonic](https://gazebosim.org/docs/harmonic) and is compatible with [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/).

There are features including FFT wave generation methods, ocean tiling, and support for the [Ogre2](https://github.com/OGRECave/ogre-next) render engine.

---

## Quick Start (ROS 2 Jazzy)

### 1. Install Dependencies

```bash
sudo apt update
sudo apt install -y libcgal-dev libfftw3-dev
```

### 2. Build asv_wave_sim

```bash
cd ~/Repos/School/autonomy-ws-25-26/src/asv_wave_sim
colcon build --symlink-install --merge-install --cmake-args \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DBUILD_TESTING=ON \
  -DCMAKE_CXX_STANDARD=17

source install/setup.bash
```

### 3. Test Gazebo with Waves (Minimal)

Run Gazebo with water + course models (no boat, no SITL):

```bash
bash ~/Repos/School/autonomy-ws-25-26/src/asv_wave_sim/run_gazebo_standalone.sh
```

**Expected result:**
- Gazebo opens with water surface (waves)
- Course models load: buoys, gates, rope holds
- Message: "Unable to find uri[model://ourboat]" (expected; boat model not set up yet)

This confirms the wave sim works!

### 4. Full Boat Simulation (Optional)

To run the complete simulation with the boat, ArduPilot SITL, and MAVROS, see [SIMULATION.md](SIMULATION.md).

---

## Key Paths and Files

|| Item | Path |
||------|------|
|| **World (Aquatonomous simulation map)** | `~/Repos/School/autonomy-ws-25-26/src/asv_wave_sim/gz-waves-models/worlds/aquatonomous_world.sdf` |
|| **Wave/course models** | `gz-waves-models/models/` and `gz-waves-models/world_models/` (waves, buoys, gates, etc.) |
|| **Boat model** | `src/asv_wave_sim/gz-waves-models/models/ourboat/` (repo-local visual override) plus `~/SITL_Models/Gazebo/models/ourboat/` (external source assets) |
|| **ArduPilot plugin** | `~/ardupilot_gazebo/build/libArduPilotPlugin.so` (external; see SIMULATION.md) |
|| **Waves/hydro libs** | `~/Repos/School/autonomy-ws-25-26/install/gz-waves1/lib/` (libgz-waves1.so, Hydrodynamics) |

---

## Scripts

This package includes two ready-to-use scripts:

|| Script | Purpose |
||--------|---------|
|| **run_gazebo_standalone.sh** | Run Gazebo with waves and course models only (no boat, no SITL). Single terminal; useful for testing. |
|| **run_full_simulation_tmux.bash** | Run the complete boat simulation (Gazebo + bridges + MAVROS + SITL) in a tmux session. All-in-one command. |

---

## Troubleshooting

|| Issue | Solution |
||-------|----------|
|| `libcgal-dev` or `libfftw3-dev` not found | `sudo apt update && sudo apt install -y libcgal-dev libfftw3-dev` |
|| Build fails | Check you're using C++17: `colcon build ... -DCMAKE_CXX_STANDARD=17` |
|| Gazebo won't start | Ensure ROS 2 Jazzy is sourced: `source /opt/ros/jazzy/setup.bash` |
|| "Unable to find uri[model://waves]" | Check that `GZ_SIM_RESOURCE_PATH` includes `gz-waves-models/models` (the standalone script does this) |
|| "Unable to find uri[model://ourboat]" | Expected without `~/SITL_Models` set up; see [SIMULATION.md](SIMULATION.md) |
|| Gazebo "0 entities" (empty world) | Verify `GZ_SIM_RESOURCE_PATH` includes all required model dirs; see [SIMULATION.md](SIMULATION.md) §9 |
|| Boat looks like only two hulls | The repo now ships a local `ourboat` override with safe primitive bridge / deck visuals under `src/asv_wave_sim/gz-waves-models/models/ourboat/`. If you still only see bare hulls, make sure `gz-waves-models/models` appears before `~/SITL_Models/Gazebo/models` in `GZ_SIM_RESOURCE_PATH`. |
|| "Missing COLLADA tag" + segmentation fault | The original external `ourboat` model disables many COLLADA / DAE visuals to avoid Gazebo loader crashes. The repo-local override keeps the sim stable by using the STL hull plus simple bridge / deck visuals instead of re-enabling the crash-prone DAE meshes. |

---

## Full Boat Simulation

For running the **complete simulation stack** (boat + ArduPilot SITL + MAVROS + ROS 2 bridges), see [SIMULATION.md](SIMULATION.md).

That document covers:
- External dependencies setup (SITL_Models, ardupilot_gazebo, bridge_ws, and optional bridge_deps_ws for ros_gz_bridge message deps)
- Full system architecture and how components connect
- How to run (tmux or separate terminals)
- How to control the boat
- Running with computer vision
- Troubleshooting the full stack

---

## Upstream Documentation

This package is the [`asv_wave_sim`](https://github.com/srmainwaring/asv_wave_sim) project by Stephen Mainwaring. For upstream docs, build instructions, and advanced features, see the [original repository](https://github.com/srmainwaring/asv_wave_sim).

---

## Dependencies

### Ubuntu 22.04 (Jammy)

- **ROS 2 Jazzy** (`apt install ros-jazzy-desktop`)
- **Gazebo Sim 8.x (Harmonic)** (installed as ROS dependency)
- **CGAL** (`libcgal-dev`)
- **FFTW** (`libfftw3-dev`)

### macOS 12.6 (Monterey)

- **ROS 2 Jazzy** (via homebrew or binary)
- **Gazebo Sim 8.x (Harmonic)**
- **CGAL** (`brew install cgal`)
- **FFTW** (`brew install fftw`)

---

## License

This project is licensed under the GPL-3.0 License (due to CGAL and FFTW dependencies).
