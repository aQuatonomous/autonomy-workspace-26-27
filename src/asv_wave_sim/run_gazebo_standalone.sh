#!/bin/bash
# Run Gazebo with waves and course models (no boat, no SITL)
# Single terminal; useful for testing the wave sim before full boat simulation.
# See README.md for quick start and SIMULATION.md for full boat setup.

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
    return 0
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
echo " Gazebo Standalone Test (asv_wave_sim)"
echo "========================================="
echo

# Check if asv_wave_sim is built
if [ ! -f "${ASV_INSTALL}/setup.bash" ]; then
    echo "ERROR: asv_wave_sim not built yet!"
    echo
    echo "Build it first:"
    echo "  cd ${WS_ROOT}"
    echo "  sudo apt install -y libcgal-dev libfftw3-dev"
    echo "  ./build.sh --packages-select gz-waves1"
    echo
    exit 1
fi

# Check external dependencies (optional for course models test)
echo "Checking external dependencies..."
MISSING_DEPS=0

if [ -z "${SITL_MODELS_ROOT}" ] || [ ! -d "${SITL_MODELS_ROOT}/models" ]; then
    echo "  WARNING: SITL_Models/Gazebo/models not found"
    echo "           (Boat model 'ourboat' will not load; this is OK for testing course models)"
    MISSING_DEPS=1
elif [ ! -d "${BOAT_MODEL_DIR}" ]; then
    echo "  WARNING: ${BOAT_MODEL_DIR} not found"
    echo "           (Launching a course-only world without the boat model)"
    MISSING_DEPS=1
fi

if [ -z "${ARDUPILOT_GAZEBO_ROOT}" ] || [ ! -d "${ARDUPILOT_GAZEBO_ROOT}/build" ]; then
    echo "  WARNING: ardupilot_gazebo/build not found"
    echo "           (ArduPilotPlugin will not load; this is OK for testing course models)"
    MISSING_DEPS=1
fi

if [ $MISSING_DEPS -eq 1 ]; then
    echo
    echo "  You can still test Gazebo with water + course models (buoys, gates, etc.)"
    echo "  See EXTERNAL_SETUP.md for how to set up the full boat simulation."
    echo
fi

# Source ROS 2 Jazzy
echo "Sourcing ROS 2 Jazzy..."
if [ ! -f "/opt/ros/jazzy/setup.bash" ]; then
    echo "ERROR: ROS 2 Jazzy not installed at /opt/ros/jazzy/"
    echo "Install it first: sudo apt install ros-jazzy-desktop"
    exit 1
fi
source /opt/ros/jazzy/setup.bash

# Source asv_wave_sim
echo "Sourcing asv_wave_sim install..."
export COLCON_CURRENT_PREFIX="${ASV_INSTALL}"
source "${ASV_INSTALL}/setup.bash"

# Set environment variables
echo "Setting Gazebo environment variables..."
export GZ_VERSION=harmonic

# LD_LIBRARY_PATH: Add asv_wave_sim libs (waves/hydro plugins)
# On x86 skip /usr/lib/aarch64-linux-gnu/nvidia (Jetson only)
if [ -d "/usr/lib/aarch64-linux-gnu/nvidia" ]; then
    export LD_LIBRARY_PATH="/usr/lib/aarch64-linux-gnu/nvidia:${GZ_WAVES_LIB}:$LD_LIBRARY_PATH"
else
    export LD_LIBRARY_PATH="${GZ_WAVES_LIB}:$LD_LIBRARY_PATH"
fi

# GZ_SIM_SYSTEM_PLUGIN_PATH: asv_wave_sim plugins (waves, hydro) first, then ArduPilotPlugin if present
_SAVED_GZ_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_WAVES_LIB}"
if [ -n "${ARDUPILOT_GAZEBO_ROOT}" ] && [ -d "${ARDUPILOT_GAZEBO_ROOT}/build" ]; then
    GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH}:${ARDUPILOT_GAZEBO_ROOT}/build"
fi
[ -n "$_SAVED_GZ_PLUGIN_PATH" ] && GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH}:$_SAVED_GZ_PLUGIN_PATH"
export GZ_SIM_SYSTEM_PLUGIN_PATH

# GZ_SIM_RESOURCE_PATH: Add model paths
GZ_SIM_RESOURCE_PATH="${WS_ROOT}/src/asv_wave_sim/gz-waves-models/models"
GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:${WS_ROOT}/src/asv_wave_sim/gz-waves-models/world_models"

# Add external paths if they exist
if [ -n "${SITL_MODELS_ROOT}" ] && [ -d "${SITL_MODELS_ROOT}/models" ]; then
    GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:${SITL_MODELS_ROOT}/models"
fi
if [ -n "${SITL_MODELS_ROOT}" ] && [ -d "${SITL_MODELS_ROOT}/worlds" ]; then
    GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:${SITL_MODELS_ROOT}/worlds"
fi
if [ -n "${ARDUPILOT_GAZEBO_ROOT}" ] && [ -d "${ARDUPILOT_GAZEBO_ROOT}/models" ]; then
    GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:${ARDUPILOT_GAZEBO_ROOT}/models"
fi
if [ -n "${ARDUPILOT_GAZEBO_ROOT}" ] && [ -d "${ARDUPILOT_GAZEBO_ROOT}/worlds" ]; then
    GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:${ARDUPILOT_GAZEBO_ROOT}/worlds"
fi

# Always append any existing GZ_SIM_RESOURCE_PATH
GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_RESOURCE_PATH

# Fix GZ_CONFIG_PATH so 'gz sim' command is available
# ROS 2 sets GZ_CONFIG_PATH to only ROS vendor directories, we need system paths too
export GZ_CONFIG_PATH="/usr/share/gz:${GZ_CONFIG_PATH:-}"

echo
echo "Environment ready. Starting Gazebo..."
WORLD_TO_RUN="$(resolve_world_file)"
if [ "${WORLD_TO_RUN}" != "${WORLD_FILE}" ]; then
    trap 'rm -f "${WORLD_TO_RUN}"' EXIT
    echo "World: $(basename "${WORLD_TO_RUN}") (generated without model://ourboat)"
else
    echo "World: aquatonomous_world.sdf"
fi
echo
echo "Expected results:"
echo "  - Water surface with waves (from WavesModel plugin)"
echo "  - Buoys, gates, rope holds (from gz-waves-models)"
echo "  - If ~/SITL_Models/Gazebo/models/ourboat exists: 'ourboat' model loads"
echo "  - Otherwise: a boat-free course world launches cleanly"
echo
echo "Press Ctrl+C to stop Gazebo"
echo "========================================="
echo

# Change to worlds directory and run Gazebo
cd "${WORLD_DIR}"
exec gz sim -v 4 -r "${WORLD_TO_RUN}"
