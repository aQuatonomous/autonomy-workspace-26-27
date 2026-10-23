# Shared environment for the simulator scripts. Source it (". env.sh"); every numbered script does.
# Paths are derived from this file's location, so the repo can live anywhere.

SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ASV_DIR="$(cd "$SIM_DIR/.." && pwd)"
WS="$(cd "$ASV_DIR/../.." && pwd)"
export SIM_DIR ASV_DIR WS
export AUTONOMY_WS="$WS"
export COURSE_DIR="$ASV_DIR/course"
export SIM_STATE_DIR="$COURSE_DIR/generated"
export SIM_RUNS_DIR="${SIM_RUNS_DIR:-$SIM_DIR/runs}"

[ -f "$SIM_DIR/config.local.env" ] && . "$SIM_DIR/config.local.env"
. "$SIM_DIR/config.env"
export ARDUPILOT_DIR ARDUPILOT_VENV ARDUPILOT_GAZEBO_DIR SITL_MODELS_DIR ROBOCOMMAND_HOST ROBOCOMMAND_PORT \
  CV_ENGINE_CACHE MESSAGE_NODE_PYDEPS

# An active Python venv (e.g. ArduPilot's) hides the ROS Python packages.
deactivate 2>/dev/null || true
if [ ! -f "$ROS_SETUP" ]; then
  echo "ROS 2 Jazzy not found at $ROS_SETUP. Run $SIM_DIR/setup_sim.sh first." >&2
  return 1 2>/dev/null || exit 1
fi
. "$ROS_SETUP"
if [ -f "$WS/install/setup.bash" ]; then
  . "$WS/install/setup.bash"
else
  echo "warning: $WS/install/setup.bash is missing; build the workspace (./build.sh or setup_sim.sh)." >&2
fi

export GZ_VERSION=harmonic
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
case ":${GZ_CONFIG_PATH:-}:" in *:/usr/share/gz:*) ;; *) export GZ_CONFIG_PATH="/usr/share/gz${GZ_CONFIG_PATH:+:$GZ_CONFIG_PATH}" ;; esac

WAVES_LIB="$WS/install/gz-waves1/lib"
case ":${LD_LIBRARY_PATH:-}:" in *":$WAVES_LIB:"*) ;; *) export LD_LIBRARY_PATH="$WAVES_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" ;; esac
export GZ_SIM_SYSTEM_PLUGIN_PATH="$WAVES_LIB:$ARDUPILOT_GAZEBO_DIR/build"
export GZ_SIM_RESOURCE_PATH="$ASV_DIR/gz-waves-models/models:$ASV_DIR/gz-waves-models/world_models:$ARDUPILOT_GAZEBO_DIR/models"
[ -d "$SITL_MODELS_DIR/Gazebo/models" ] && GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$SITL_MODELS_DIR/Gazebo/models"

# WSL2: WSLg's Mesa defaults to software rendering; d3d12 routes OpenGL to the Windows GPU, and the
# Gazebo GUI is stable on X11 (xcb) rather than WSLg's Wayland.
if grep -qi microsoft /proc/version 2>/dev/null; then
  export SIM_IS_WSL=1
  export GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}"
  export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
  # CUDA's WSL driver libraries (libcuda) live here.
  case ":${LD_LIBRARY_PATH}:" in *:/usr/lib/wsl/lib:*) ;; *) export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/usr/lib/wsl/lib" ;; esac
fi
[ -d /usr/local/cuda/lib64 ] && case ":${LD_LIBRARY_PATH}:" in *:/usr/local/cuda/lib64:*) ;; *) export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/usr/local/cuda/lib64" ;; esac

# Prints a warning when OpenGL falls back to the CPU renderer (llvmpipe); Gazebo then runs at a
# few percent of real time and the cameras are unusable.
sim_check_gpu() {
  command -v glxinfo >/dev/null 2>&1 || return 0
  local r
  r="$(glxinfo -B 2>/dev/null | sed -n 's/^OpenGL renderer string: //p')"
  case "$r" in
    *llvmpipe*|*softpipe*|*SWR*)
      echo "WARNING: OpenGL is using the software renderer ($r)." >&2
      if [ -n "${SIM_IS_WSL:-}" ]; then
        echo "  On WSL2: update the Windows GPU driver and WSL (wsl --update), then wsl --shutdown." >&2
      else
        echo "  Install the vendor GPU driver (e.g. sudo ubuntu-drivers install) and reboot." >&2
      fi ;;
    "") ;;
    *) echo "OpenGL renderer: $r" ;;
  esac
}

# Course selected by 1_gazebo.sh / select_course.sh: SIM_WORLD, SIM_ANSWER_KEY, SIM_SITL_HOME, ...
sim_load_course() {
  if [ -f "$SIM_STATE_DIR/current.env" ]; then
    . "$SIM_STATE_DIR/current.env"
    export SIM_WORLD SIM_WORLD_NAME SIM_ANSWER_KEY SIM_SITL_HOME SIM_WATER_DENSITY SIM_SEED
    return 0
  fi
  return 1
}

# message_node sends to the competition address (10.10.10.1). Adding it to the loopback interface
# lets the sim judge, listening on all addresses, receive those reports on this machine.
# Needs root once per boot. A run without the alias still works, it just cannot reach the judge.
sim_robocommand_address() {
  case "$ROBOCOMMAND_HOST" in 127.*|localhost) return 0 ;; esac
  ip -4 addr show dev lo 2>/dev/null | grep -q "inet $ROBOCOMMAND_HOST/" && return 0
  local sudo=sudo; [ "$(id -u)" -eq 0 ] && sudo=""
  # Never prompt. 7_stack.sh calls this part-way through starting the stack and its tmux windows
  # do have a tty, so the interactive branch blocked the script forever on a password prompt --
  # silently costing every node started after this line (message_node and the truth scorer).
  $sudo ${sudo:+-n} ip addr add "$ROBOCOMMAND_HOST/32" dev lo 2>/dev/null && return 0
  echo "WARNING: $ROBOCOMMAND_HOST is not on this machine; reports will not reach the judge." >&2
  echo "  Run once: sudo ip addr add $ROBOCOMMAND_HOST/32 dev lo" >&2
  return 1
}

# Blocks until Gazebo publishes the world clock.
sim_wait_for_gazebo() {
  local world="${SIM_WORLD_NAME:-}"
  echo "Waiting for Gazebo${world:+ (world $world)}..."
  until gz topic -l 2>/dev/null | grep -q "^/world/${world:-[^/]*}/clock$"; do sleep 2; done
}
