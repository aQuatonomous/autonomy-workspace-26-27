#!/bin/bash
# Terminal 4: MAVROS. Waits for SITL's second MAVLink port (TCP 5762; MAVProxy holds 5760).
. "$(dirname "${BASH_SOURCE[0]}")/env.sh"
until (echo >/dev/tcp/127.0.0.1/5762) 2>/dev/null; do
  echo "Waiting for SITL on port 5762..."
  sleep 3
done
exec ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5762 "$@"
