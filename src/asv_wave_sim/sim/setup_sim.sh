#!/bin/bash
# One-shot, re-runnable setup of the RoboBoat simulator on Ubuntu 24.04 (native or WSL2).
#
#   setup_sim.sh               install/build whatever is missing (asks for sudo when needed)
#   setup_sim.sh --with-cv     also CUDA runtime + TensorRT 10 + per-GPU engines (NVIDIA GPU)
#   setup_sim.sh --check       report what is installed; change nothing
#   setup_sim.sh --system-only only the apt steps (e.g. run once as root: sudo setup_sim.sh --system-only)
#   setup_sim.sh --rebuild     rebuild the colcon workspace even when it is already built
#
# Locations come from config.env (override in config.local.env). Every step checks first and
# skips work that is already done, so running it again is safe.
set -u
SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$SIM_DIR/../../.." && pwd)"
[ -f "$SIM_DIR/config.local.env" ] && . "$SIM_DIR/config.local.env"
. "$SIM_DIR/config.env"

CHECK=0 WITH_CV=0 SYSTEM_ONLY=0 REBUILD=0
for a in "$@"; do
  case "$a" in
    --check) CHECK=1 ;;
    --with-cv) WITH_CV=1 ;;
    --system-only) SYSTEM_ONLY=1 ;;
    --rebuild) REBUILD=1 ;;
    -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
    *) echo "unknown option $a" >&2; exit 2 ;;
  esac
done

TRT_VERSION="10.16.1.11-1+cuda12.9"   # TensorRT 10 on CUDA 12 like the Jetson; supports Blackwell (RTX 50xx)
CUDART_PKG="cuda-cudart-12-9"
TRT_PKGS=(libnvinfer10 libnvinfer-plugin10 libnvinfer-vc-plugin10 libnvonnxparsers10 libnvinfer-lean10
          libnvinfer-dispatch10 python3-libnvinfer libnvinfer-bin)

APT_PKGS=(ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-mavros ros-jazzy-mavros-extras
          ros-jazzy-ament-cmake-clang-format python3-colcon-common-extensions python3-rosdep
          build-essential cmake git curl tmux mesa-utils python3-yaml python3-pil python3-numpy python3-pytest
          python3-protobuf python3-sklearn python3-scipy rapidjson-dev libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
          gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl portaudio19-dev)

IS_WSL=0; grep -qi microsoft /proc/version 2>/dev/null && IS_WSL=1
SUDO="sudo"; [ "$(id -u)" -eq 0 ] && SUDO=""
declare -a REPORT=()
FAILED=0

ok()   { REPORT+=("  [ok]      $1"); }
miss() { REPORT+=("  [missing] $1"); FAILED=1; }
did()  { REPORT+=("  [done]    $1"); }
bad()  { REPORT+=("  [FAILED]  $1"); FAILED=1; }
step() { echo; echo "=== $1"; }
have_pkg() { dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -q "install ok installed"; }

# ---------------------------------------------------------------- system (apt)
system_steps() {
  step "Operating system"
  . /etc/os-release
  if [ "${VERSION_ID:-}" = "24.04" ]; then ok "Ubuntu 24.04$( [ $IS_WSL = 1 ] && echo ' (WSL2)')"
  else REPORT+=("  [warn]    $PRETTY_NAME: only Ubuntu 24.04 is tested"); fi

  step "ROS 2 Jazzy apt source"
  if [ -f /etc/apt/sources.list.d/ros2.list ] || ls /etc/apt/sources.list.d/ 2>/dev/null | grep -q ros2; then
    ok "ROS 2 apt source"
  elif [ $CHECK = 1 ]; then miss "ROS 2 apt source"
  else
    $SUDO apt-get update && $SUDO apt-get install -y curl gnupg lsb-release software-properties-common &&
    $SUDO add-apt-repository -y universe &&
    $SUDO curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg &&
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" |
      $SUDO tee /etc/apt/sources.list.d/ros2.list >/dev/null && did "ROS 2 apt source" || bad "ROS 2 apt source"
  fi

  step "apt packages (ROS 2 Jazzy, Gazebo Harmonic via ros_gz, MAVROS, build deps)"
  local missing=()
  for p in "${APT_PKGS[@]}"; do have_pkg "$p" || missing+=("$p"); done
  if [ ${#missing[@]} -eq 0 ]; then ok "apt packages"
  elif [ $CHECK = 1 ]; then miss "apt packages: ${missing[*]}"
  else
    # Gazebo comes from the ROS vendor packages; a second copy from packages.osrfoundation.org
    # (gz-harmonic, libgz-sim8-dev) conflicts with it, so it is deliberately absent here.
    $SUDO apt-get update && $SUDO apt-get install -y "${missing[@]}" && did "apt packages: ${missing[*]}" ||
      bad "apt packages: ${missing[*]}"
  fi

  step "GeographicLib datasets (MAVROS)"
  if [ -d /usr/share/GeographicLib/geoids ]; then ok "GeographicLib datasets"
  elif [ $CHECK = 1 ]; then miss "GeographicLib datasets"
  else $SUDO bash /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh && did "GeographicLib datasets" ||
       bad "GeographicLib datasets"; fi

  if [ $WITH_CV = 1 ]; then cv_system_steps; fi
}

cv_system_steps() {
  step "NVIDIA GPU"
  if command -v nvidia-smi >/dev/null && nvidia-smi -L >/dev/null 2>&1; then ok "GPU: $(nvidia-smi --query-gpu=name,driver_version --format=csv,noheader | head -n 1)"
  else
    if [ $IS_WSL = 1 ]; then bad "nvidia-smi: install/update the NVIDIA driver on Windows (WSL uses it; never install a Linux driver inside WSL)"
    else bad "nvidia-smi: install the NVIDIA driver (sudo ubuntu-drivers install) and reboot"; fi
    return
  fi

  step "CUDA runtime + TensorRT ${TRT_VERSION%%-*}"
  local need=0
  have_pkg "$CUDART_PKG" || need=1
  for p in "${TRT_PKGS[@]}"; do
    [ "$(dpkg-query -W -f='${Version}' "$p" 2>/dev/null)" = "$TRT_VERSION" ] || need=1
  done
  if [ $need = 0 ]; then ok "TensorRT $TRT_VERSION + $CUDART_PKG"; return; fi
  if [ $CHECK = 1 ]; then miss "TensorRT $TRT_VERSION / $CUDART_PKG"; return; fi
  if ! have_pkg cuda-keyring; then
    curl -fsSL -o /tmp/cuda-keyring.deb \
      https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb &&
      $SUDO dpkg -i /tmp/cuda-keyring.deb || { bad "NVIDIA CUDA apt repository"; return; }
  fi
  if [ $IS_WSL = 1 ]; then
    # WSL gets the GPU driver from Windows; a Linux driver from this repo would break it.
    printf 'Package: cuda-drivers* nvidia-driver* nvidia-dkms* nvidia-kernel* xserver-xorg-video-nvidia*\nPin: release *\nPin-Priority: -1\n' |
      $SUDO tee /etc/apt/preferences.d/aquatonomous-wsl-no-nvidia-driver >/dev/null
  fi
  local pins=()
  for p in "${TRT_PKGS[@]}"; do pins+=("$p=$TRT_VERSION"); done
  $SUDO apt-mark unhold "${TRT_PKGS[@]}" >/dev/null 2>&1
  if $SUDO apt-get update && $SUDO apt-get install -y --allow-downgrades --allow-change-held-packages "$CUDART_PKG" "${pins[@]}"; then
    # Keep apt upgrade from replacing TensorRT 10 with 11 (different API and engine format).
    $SUDO apt-mark hold "${TRT_PKGS[@]}" >/dev/null
    $SUDO ldconfig
    did "TensorRT $TRT_VERSION + $CUDART_PKG (held)"
  else bad "TensorRT $TRT_VERSION / $CUDART_PKG"; fi
}

# ---------------------------------------------------------------- user (clone + build)
user_steps() {
  # ROS setup scripts read unset variables, which set -u turns into an exit
  set +u
  # shellcheck disable=SC1091
  . /opt/ros/jazzy/setup.bash 2>/dev/null || { set -u; miss "ROS 2 Jazzy (/opt/ros/jazzy)"; return; }
  set -u
  export GZ_VERSION=harmonic

  step "ArduPilot SITL ($ARDUPILOT_DIR)"
  if [ -x "$ARDUPILOT_DIR/build/sitl/bin/ardurover" ] && [ -f "$ARDUPILOT_VENV/bin/activate" ]; then ok "ArduPilot Rover SITL + MAVProxy venv"
  elif [ $CHECK = 1 ]; then miss "ArduPilot Rover SITL ($ARDUPILOT_DIR/build/sitl/bin/ardurover, $ARDUPILOT_VENV)"
  else
    ( set -e
      [ -d "$ARDUPILOT_DIR/.git" ] || git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git "$ARDUPILOT_DIR"
      cd "$ARDUPILOT_DIR"
      git submodule update --init --recursive
      # Installs build deps and MAVProxy into a venv ($HOME/venv-ardupilot); prompts for sudo.
      [ -f "$ARDUPILOT_VENV/bin/activate" ] || Tools/environment_install/install-prereqs-ubuntu.sh -y
      deactivate 2>/dev/null || true
      . "$ARDUPILOT_VENV/bin/activate"
      ./waf configure --board sitl
      ./waf rover
    ) && did "ArduPilot Rover SITL" || bad "ArduPilot Rover SITL"
  fi

  step "ArduPilot Gazebo plugin ($ARDUPILOT_GAZEBO_DIR)"
  if [ -f "$ARDUPILOT_GAZEBO_DIR/build/libArduPilotPlugin.so" ]; then ok "ArduPilotPlugin"
  elif [ $CHECK = 1 ]; then miss "ArduPilotPlugin ($ARDUPILOT_GAZEBO_DIR/build/libArduPilotPlugin.so)"
  else
    ( set -e
      deactivate 2>/dev/null || true
      [ -d "$ARDUPILOT_GAZEBO_DIR/.git" ] || git clone https://github.com/ArduPilot/ardupilot_gazebo.git "$ARDUPILOT_GAZEBO_DIR"
      mkdir -p "$ARDUPILOT_GAZEBO_DIR/build" && cd "$ARDUPILOT_GAZEBO_DIR/build"
      cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
      make -j"$(nproc)"
    ) && did "ArduPilotPlugin" || bad "ArduPilotPlugin"
  fi

  step "Workspace build ($WS)"
  if [ -f "$WS/install/setup.bash" ] && [ -d "$WS/install/sim_judge" ] && [ -f "$WS/install/gz-waves1/lib/libgz-waves1-hydrodynamics-system.so" ] && [ $REBUILD = 0 ]; then
    ok "colcon workspace (pass --rebuild to rebuild)"
  elif [ $CHECK = 1 ]; then miss "colcon workspace build ($WS/install)"
  else
    ( set -e
      deactivate 2>/dev/null || true
      cd "$WS"
      rosdep update >/dev/null 2>&1 || true
      rosdep install --from-paths src --ignore-src -y -r --rosdistro jazzy >/dev/null 2>&1 || true
      ./build.sh
    ) && did "colcon workspace" || bad "colcon workspace (see the build output above)"
  fi

  step "protobuf for message_node ($MESSAGE_NODE_PYDEPS)"
  # report_pb2.py is protobuf 6.32 generated code; Ubuntu's python3-protobuf (4.21) cannot load it
  # and the Gazebo Python bindings need the system one, so this copy is only on message_node's path.
  local pbv
  pbv="$(PYTHONPATH="$MESSAGE_NODE_PYDEPS" python3 -c 'import google.protobuf as p; print(p.__version__)' 2>/dev/null)"
  if [[ "$pbv" == 6.* ]] && [ -d "$MESSAGE_NODE_PYDEPS/google/protobuf" ]; then ok "protobuf $pbv for message_node"
  elif [ $CHECK = 1 ]; then miss "protobuf 6 for message_node ($MESSAGE_NODE_PYDEPS)"
  elif [ -x "$ARDUPILOT_VENV/bin/pip" ] &&
       "$ARDUPILOT_VENV/bin/pip" install -q --upgrade --target "$MESSAGE_NODE_PYDEPS" 'protobuf>=6.32,<7'; then
    did "protobuf for message_node"
  else bad "protobuf for message_node (needs $ARDUPILOT_VENV/bin/pip)"; fi

  step "Course world"
  if python3 "$WS/src/asv_wave_sim/tools/generate_course.py" --config "$WS/src/asv_wave_sim/course/roboboat2026.yaml" \
       --seed 0 --out /tmp/roboboat_setup_check >/dev/null 2>&1; then ok "course generator (seed 0)"
  else bad "course generator: python3 $WS/src/asv_wave_sim/tools/generate_course.py --seed 0"; fi

  if [ $WITH_CV = 1 ]; then
    step "TensorRT engines for this GPU"
    if bash "$SIM_DIR/build_engines.sh" --check >/dev/null 2>&1; then ok "TensorRT engines ($(bash "$SIM_DIR/build_engines.sh" --print-dir))"
    elif [ $CHECK = 1 ]; then miss "TensorRT engines (build_engines.sh)"
    else bash "$SIM_DIR/build_engines.sh" && did "TensorRT engines" || bad "TensorRT engines"; fi
  fi

  step "OpenGL renderer"
  local r
  r="$(glxinfo -B 2>/dev/null | sed -n 's/^OpenGL renderer string: //p')"
  if [ $IS_WSL = 1 ] && [ -z "$r" ] || [[ "$r" == *llvmpipe* ]]; then
    r="$(GALLIUM_DRIVER=d3d12 glxinfo -B 2>/dev/null | sed -n 's/^OpenGL renderer string: //p')"
  fi
  case "$r" in
    *llvmpipe*|*softpipe*|"") REPORT+=("  [warn]    OpenGL renderer '${r:-none}': Gazebo will be very slow without GPU rendering") ;;
    *) ok "OpenGL renderer: $r" ;;
  esac
}

if [ $SYSTEM_ONLY = 1 ]; then
  system_steps
elif [ "$(id -u)" -eq 0 ]; then
  echo "Run this as your normal user (it calls sudo for the apt steps), or as root with --system-only." >&2
  exit 2
else
  system_steps
  user_steps
fi

echo
echo "=== Summary"
printf '%s\n' "${REPORT[@]}"
if [ $FAILED = 0 ]; then
  echo
  [ $CHECK = 1 ] && echo "Everything is in place." || echo "Setup complete."
  echo "Start a run with: $SIM_DIR/run_competition.sh --seed 0 --task 1"
fi
exit $FAILED
