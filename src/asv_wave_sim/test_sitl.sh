#!/bin/bash
# Test ArduPilot SITL
# Run AFTER Gazebo is running (40+ seconds after Gazebo starts)

set -e

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

ARDUPILOT_ROOT="$(first_existing_dir \
  "$HOME/Repos/School/ardupilot" \
  "$HOME/ardupilot")"

echo "========================================="
echo " Test 3: ArduPilot SITL"
echo "========================================="
echo ""
echo "Prerequisites:"
echo "  - Gazebo must be running"
echo "  - Wait 40+ seconds after Gazebo starts"
echo ""

# Ensure MAVProxy is in PATH
export PATH="$HOME/.local/bin:$PATH"
export PYTHONPATH="$HOME/.local/lib/python3.12/site-packages:$PYTHONPATH"

if [ -z "${ARDUPILOT_ROOT}" ] || [ ! -d "${ARDUPILOT_ROOT}/Tools/autotest" ]; then
  echo "ERROR: Could not find ArduPilot checkout."
  echo "Expected one of:"
  echo "  $HOME/Repos/School/ardupilot"
  echo "  $HOME/ardupilot"
  exit 1
fi

cd "${ARDUPILOT_ROOT}/Tools/autotest"

echo "Starting ArduPilot SITL with MAVProxy..."
echo ""
echo "In MAVProxy console, type:"
echo "  param set ARMING_REQUIRE 0"
echo "  mode GUIDED"
echo "  arm throttle"
echo "  position 10 0 0"
echo ""

python3 ./sim_vehicle.py -v Rover -f rover-skid --model JSON \
  --console --map \
  --custom-location="51.566151,-4.034345,10.0,-135"
