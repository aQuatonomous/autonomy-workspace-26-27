#!/bin/bash
# Test Gazebo GUI launch separately
# Run this first to verify Gazebo window opens

set -e

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
WS_ROOT="$(cd -- "${SCRIPT_DIR}/../.." >/dev/null 2>&1 && pwd)"
WS_INSTALL="${WS_ROOT}/install"
LEGACY_INSTALL="${WS_ROOT}/src/asv_wave_sim/install"
WORLD_DIR="${WS_ROOT}/src/asv_wave_sim/gz-waves-models/worlds"
WORLD_FILE="${WORLD_DIR}/aquatonomous_world.sdf"

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
  "$HOME/Repos/School/SITL_Models/Gazebo" \
  "$HOME/SITL_Models/Gazebo")"
ARDUPILOT_GAZEBO_ROOT="$(first_existing_dir \
  "$HOME/Repos/School/ardupilot_gazebo" \
  "$HOME/ardupilot_gazebo")"
BOAT_MODEL_DIR="${SITL_MODELS_ROOT}/models/ourboat"

if [ -f "${WS_INSTALL}/setup.bash" ]; then
  ASV_INSTALL="${WS_INSTALL}"
elif [ -f "${LEGACY_INSTALL}/setup.bash" ]; then
  ASV_INSTALL="${LEGACY_INSTALL}"
else
  ASV_INSTALL="${WS_INSTALL}"
fi

if [ -d "${WS_INSTALL}/gz-waves1/lib" ]; then
  GZ_WAVES_LIB="${WS_INSTALL}/gz-waves1/lib"
else
  GZ_WAVES_LIB="${ASV_INSTALL}/lib"
fi

resolve_world_file() {
  if [ -d "${BOAT_MODEL_DIR}" ]; then
    printf '%s\n' "${WORLD_FILE}"
    return
  fi

  local temp_world
  temp_world="$(mktemp /tmp/aquatonomous_world_no_boat.XXXXXX.sdf)"
  awk '
    /<include>/ { block = $0 ORS; in_block = 1; next }
    in_block {
      block = block $0 ORS
      if ($0 ~ /model:\/\/ourboat/) {
        skip_block = 1
      }
      if ($0 ~ /<\/include>/) {
        if (!skip_block) {
          printf "%s", block
        }
        block = ""
        in_block = 0
        skip_block = 0
      }
      next
    }
    { print }
  ' "${WORLD_FILE}" > "${temp_world}"
  printf '%s\n' "${temp_world}"
}

echo "========================================="
echo " Test 1: Gazebo GUI Launch"
echo "========================================="

# Source ROS and asv_wave_sim
source /opt/ros/jazzy/setup.bash
export COLCON_CURRENT_PREFIX="${ASV_INSTALL}"
source "${ASV_INSTALL}/setup.bash"

# Set environment
export GZ_VERSION=harmonic
export LD_LIBRARY_PATH="${GZ_WAVES_LIB}:$LD_LIBRARY_PATH"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_WAVES_LIB}${ARDUPILOT_GAZEBO_ROOT:+:${ARDUPILOT_GAZEBO_ROOT}/build}"

GZ_SIM_RESOURCE_PATH="${WS_ROOT}/src/asv_wave_sim/gz-waves-models/models:${WS_ROOT}/src/asv_wave_sim/gz-waves-models/world_models"
[ -n "${SITL_MODELS_ROOT}" ] && GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:${SITL_MODELS_ROOT}/models:${SITL_MODELS_ROOT}/worlds"
[ -n "${ARDUPILOT_GAZEBO_ROOT}" ] && GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:${ARDUPILOT_GAZEBO_ROOT}/models:${ARDUPILOT_GAZEBO_ROOT}/worlds"
export GZ_SIM_RESOURCE_PATH

echo "Starting Gazebo GUI..."
echo "Window should open showing boat, water, and course..."
WORLD_TO_RUN="$(resolve_world_file)"
if [ "${WORLD_TO_RUN}" != "${WORLD_FILE}" ]; then
  trap 'rm -f "${WORLD_TO_RUN}"' EXIT
  echo "Boat model not found; launching generated course-only world."
fi
echo ""

cd "${WORLD_DIR}"

# Fix GZ_CONFIG_PATH so 'gz sim' command is available
# ROS 2 sets GZ_CONFIG_PATH to only ROS vendor directories, we need system paths too
export GZ_CONFIG_PATH="/usr/share/gz:${GZ_CONFIG_PATH:-}"

gz sim -v 2 -r "${WORLD_TO_RUN}"
