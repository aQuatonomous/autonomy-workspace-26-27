#!/bin/bash
# Run full boat simulation in tmux: Gazebo + bridges + ArduPilot SITL + MAVROS
# Total: 3 windows, 9 panes. Switch windows: Ctrl+b 0 / 1 / 2
#   Window 0: Gazebo (pane 0.0) + 4 bridge panes (0.1–0.4: lidar, camera1, camera2, camera3)
#   Window 1: ArduPilot SITL (left) + ROS shell (right)
#   Window 2: MAVROS (left) + ROS shell (right)
# If SITL fails with NumPy/matplotlib (MAVProxy), use SITL_NO_GUI=1 to skip --console --map.
# See SIMULATION.md for full documentation.

set -e

SESSION=simfull

# Compute absolute paths so we don't depend on $HOME being correct inside tmux.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
WS_ROOT="$(cd -- "${SCRIPT_DIR}/../.." >/dev/null 2>&1 && pwd)"
USER_HOME="${HOME}"
ASV_INSTALL="${WS_ROOT}/install"
# gz-waves and hydro libs live under install/gz-waves1/lib (no top-level install/lib)
GZ_WAVES_LIB="${WS_ROOT}/install/gz-waves1/lib"

first_existing_dir() {
  local candidate
  for candidate in "$@"; do
    if [ -d "${candidate}" ]; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done
  return 1
}

SITL_MODELS_ROOT="$(first_existing_dir \
  "${USER_HOME}/Repos/School/SITL_Models/Gazebo" \
  "${USER_HOME}/SITL_Models/Gazebo")"
ARDUPILOT_GAZEBO_ROOT="$(first_existing_dir \
  "${USER_HOME}/Repos/School/ardupilot_gazebo" \
  "${USER_HOME}/ardupilot_gazebo")"
ARDUPILOT_ROOT="$(first_existing_dir \
  "${USER_HOME}/Repos/School/ardupilot" \
  "${USER_HOME}/ardupilot")"
BOAT_MODEL_DIR="${SITL_MODELS_ROOT}/models/ourboat"

# Start fresh: kill old session so Gazebo actually starts
tmux kill-session -t "$SESSION" 2>/dev/null || true

if [ -z "${SITL_MODELS_ROOT}" ] || [ ! -d "${BOAT_MODEL_DIR}" ]; then
  echo "ERROR: Missing boat model at ${BOAT_MODEL_DIR}"
  echo "The full tmux simulation needs the external 'ourboat' model."
  echo
  echo "Options:"
  echo "  1. Add SITL_Models/Gazebo/models/ourboat under ~/ or ~/Repos/School"
  echo "  2. For Gazebo-only testing, run:"
  echo "     bash ${WS_ROOT}/src/asv_wave_sim/run_gazebo_standalone.sh"
  exit 1
fi

if [ -z "${ARDUPILOT_GAZEBO_ROOT}" ] || [ ! -d "${ARDUPILOT_GAZEBO_ROOT}/build" ]; then
  echo "ERROR: Missing ardupilot_gazebo build directory."
  echo "Expected one of:"
  echo "  ${USER_HOME}/Repos/School/ardupilot_gazebo/build"
  echo "  ${USER_HOME}/ardupilot_gazebo/build"
  exit 1
fi

if [ -z "${ARDUPILOT_ROOT}" ] || [ ! -d "${ARDUPILOT_ROOT}/Tools/autotest" ]; then
  echo "ERROR: Missing ArduPilot checkout."
  echo "Expected one of:"
  echo "  ${USER_HOME}/Repos/School/ardupilot"
  echo "  ${USER_HOME}/ardupilot"
  exit 1
fi

# Helper: send a command string to a pane and press Enter
send() { tmux send-keys -t "$1" "$2" Enter; }

# ── Create session and all windows/panes with plain bash ─────────────────────
tmux new-session  -d -s "$SESSION" -x 220 -y 50
# Window 0: already exists (pane 0.0 = Gazebo). Add 4 bridge panes on the right.
tmux split-window -t "${SESSION}:0" -h
tmux split-window -t "${SESSION}:0.1" -v
tmux split-window -t "${SESSION}:0.2" -v
tmux split-window -t "${SESSION}:0.3" -v

# Window 1: SITL (left) + ROS shell (right)
tmux new-window -t "${SESSION}" -n sitl
tmux split-window -t "${SESSION}:1" -h

# Window 2: MAVROS (left) + ROS shell (right)
tmux new-window -t "${SESSION}" -n mavros
tmux split-window -t "${SESSION}:2" -h

# ── Window 0: Gazebo + Bridges ───────────────────────────────────────────────
send "${SESSION}:0.0" "source /opt/ros/jazzy/setup.bash && source ${ASV_INSTALL}/setup.bash && export GZ_VERSION=harmonic && export GZ_CONFIG_PATH=/usr/share/gz:\${GZ_CONFIG_PATH:-} && export LD_LIBRARY_PATH="$HOME/Repos/School/autonomy-ws-25-26/install/gz-waves1/lib:$LD_LIBRARY_PATH" && export GZ_SIM_SYSTEM_PLUGIN_PATH=${GZ_WAVES_LIB}:${ARDUPILOT_GAZEBO_ROOT}/build && export GZ_SIM_RESOURCE_PATH=${WS_ROOT}/src/asv_wave_sim/gz-waves-models/models:${WS_ROOT}/src/asv_wave_sim/gz-waves-models/world_models:${SITL_MODELS_ROOT}/models:${SITL_MODELS_ROOT}/worlds:${ARDUPILOT_GAZEBO_ROOT}/models:${ARDUPILOT_GAZEBO_ROOT}/worlds && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && cd ${WS_ROOT}/src/asv_wave_sim/gz-waves-models/worlds && gz sim -v 4 -r aquatonomous_world.sdf"

# Bridge panes – sleep 20s to let Gazebo fully initialize
BRIDGE_SRC="source /opt/ros/jazzy/setup.bash; [ -f ~/bridge_deps_ws/install/setup.bash ] && source ~/bridge_deps_ws/install/setup.bash; [ -f ~/bridge_ws/install/setup.bash ] && source ~/bridge_ws/install/setup.bash; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
send "${SESSION}:0.1" "$BRIDGE_SRC && sleep 20 && ros2 run ros_gz_bridge parameter_bridge /lidar_topic/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked --ros-args -r /lidar_topic/points:=/laser_points"
send "${SESSION}:0.2" "$BRIDGE_SRC && sleep 20 && ros2 run ros_gz_bridge parameter_bridge /camera1_topic@sensor_msgs/msg/Image@gz.msgs.Image --ros-args -r /camera1_topic:=/camera0/image_raw"
send "${SESSION}:0.3" "$BRIDGE_SRC && sleep 20 && ros2 run ros_gz_bridge parameter_bridge /camera2_topic@sensor_msgs/msg/Image@gz.msgs.Image --ros-args -r /camera2_topic:=/camera1/image_raw"
send "${SESSION}:0.4" "$BRIDGE_SRC && sleep 20 && ros2 run ros_gz_bridge parameter_bridge /camera3_topic@sensor_msgs/msg/Image@gz.msgs.Image --ros-args -r /camera3_topic:=/camera2/image_raw"

# ── Window 1: ArduPilot SITL ─────────────────────────────────────────────────
# MAVProxy runs here. With --console --map you get Qt windows; if NumPy/matplotlib fails, use SITL_NO_GUI=1.
# Waits 35s for Gazebo/ArduPilotPlugin to be ready on port 9002 before SITL connects.
SITL_EXTRA="--console --map"; [ -n "${SITL_NO_GUI:-}" ] && SITL_EXTRA=""
send "${SESSION}:1.0" "export PATH=${USER_HOME}/.local/bin:\$PATH && export PYTHONPATH=${USER_HOME}/.local/lib/python3.12/site-packages:\$PYTHONPATH && sleep 35 && echo 'Starting SITL...' && cd ${ARDUPILOT_ROOT}/Tools/autotest && python3 ./sim_vehicle.py -v Rover -f rover-skid --model JSON ${SITL_EXTRA} --custom-location=51.566151,-4.034345,10.0,-135"
send "${SESSION}:1.1" "source /opt/ros/jazzy/setup.bash && source ${ASV_INSTALL}/setup.bash && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && echo 'ROS shell - try: ros2 topic list'"

# ── Window 2: MAVROS ─────────────────────────────────────────────────────────
# Waits for SITL TCP port 5760 before launching MAVROS.
send "${SESSION}:2.0" "source /opt/ros/jazzy/setup.bash && source ${ASV_INSTALL}/setup.bash && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && echo 'Waiting for SITL port 5760...' && until (echo >/dev/tcp/localhost/5760) 2>/dev/null; do echo '  waiting...'; sleep 3; done && echo 'SITL ready! Starting MAVROS...' && ros2 launch mavros apm.launch fcu_url:=tcp://localhost:5760"
send "${SESSION}:2.1" "source /opt/ros/jazzy/setup.bash && source ${ASV_INSTALL}/setup.bash && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && echo 'ROS shell - try: ros2 topic list | grep mavros'"

# Attach at Window 0 (Gazebo view)
tmux select-window -t "${SESSION}:0"
tmux attach-session -t "$SESSION"
