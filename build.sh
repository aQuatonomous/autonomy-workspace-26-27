#!/bin/bash
# Top-level build script for the repo.
# Run from the repository root.
#
# This builds:
#   - Root ROS2 workspace (./src)
#   - Computer vision workspace (./computer_vision/src)
#
# Examples (extra args are passed to the **root** build):
#   ./build.sh
#   ./build.sh --packages-select message_node task_sequence_coordinator
#   ./build.sh --packages-up-to task_sequence_coordinator

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# If this repository was moved, old colcon CMake caches can still point to
# the previous path and make every package fail immediately.
has_stale_cmake_cache=0
if [ -d "${SCRIPT_DIR}/build" ]; then
  while IFS= read -r cache_file; do
    old_src_dir="$(sed -n 's|^CMAKE_HOME_DIRECTORY:INTERNAL=||p' "${cache_file}")"
    if [ -n "${old_src_dir}" ] && [[ "${old_src_dir}" != "${SCRIPT_DIR}"/* ]]; then
      has_stale_cmake_cache=1
      break
    fi
  done < <(find "${SCRIPT_DIR}/build" -name CMakeCache.txt -type f 2>/dev/null)
fi

if [ "${has_stale_cmake_cache}" -eq 1 ]; then
  echo "Detected stale CMake cache from a previous workspace path."
  echo "Cleaning generated colcon artifacts (build/install/log) for a fresh rebuild..."
  rm -rf "${SCRIPT_DIR}/build" "${SCRIPT_DIR}/install" "${SCRIPT_DIR}/log"
fi

echo "=== Building root ROS2 workspace (./src) ==="
echo

# Source ROS2 from a clean environment
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/jazzy/setup.bash

# This workspace targets Gazebo Harmonic on ROS 2 Jazzy. Without this, gz-waves1
# falls back to Garden and looks for the wrong dependency versions.
export GZ_VERSION="${GZ_VERSION:-harmonic}"

# Build from the workspace root; only consider packages under ./src
cd "${SCRIPT_DIR}"
colcon build --symlink-install --base-paths src "$@"

echo
echo "✓ Root workspace build finished."

# Source the freshly built root workspace as an underlay for downstream workspaces
source "${SCRIPT_DIR}/install/setup.bash"

# Build computer_vision workspace if present
if [ -d "${SCRIPT_DIR}/computer_vision/src" ]; then
  echo
  echo "=== Building computer_vision workspace (./computer_vision/src) ==="
  echo
  cd "${SCRIPT_DIR}/computer_vision"
  colcon build --symlink-install --base-paths src
  echo
  echo "✓ computer_vision workspace build finished."
else
  echo
  echo "Skipping computer_vision build (no computer_vision/src directory found)."
fi

echo
echo "✓ All builds finished."
echo "Next:"
echo "  source install/setup.bash"
echo "  (and for CV nodes, also: cd computer_vision && source install/setup.bash)"
