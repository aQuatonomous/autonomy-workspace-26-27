#!/bin/bash
# Puts the SITL boat in GUIDED mode and arms it over MAVROS (what the safety operator does with
# the RC transmitter at competition). Retries until both succeed.
. "$(dirname "${BASH_SOURCE[0]}")/env.sh"
until ros2 topic echo --once /mavros/state 2>/dev/null | grep -q 'connected: true'; do
  echo "Waiting for MAVROS to connect to SITL..."
  sleep 3
done
for _ in $(seq 1 30); do
  if ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}" 2>/dev/null | grep -q 'mode_sent=True' &&
     ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}" 2>/dev/null | grep -q 'success=True'; then
    echo "GUIDED and armed."
    exit 0
  fi
  echo "Not armed yet (SITL may still be waiting for a GPS fix); retrying..."
  sleep 3
done
echo "Could not arm. In the SITL terminal try: mode GUIDED, then arm throttle." >&2
exit 1
