#!/bin/bash
# Echo important topics in a single updating frame (boat_pose, cmd_vel, fused buoys).
# Run from the repository root. Refreshes in place like monitor_camera_move.sh.
#
# If you see "RuntimeError: !rclpy.ok()" when running ros2 topic list/echo in another
# terminal, use a fresh terminal and only source ROS + workspace (no nodes that call
# rclpy.shutdown), or run:  ros2 daemon stop  then  ros2 daemon start

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "${SCRIPT_DIR}/install/setup.bash"

INTERVAL="${INTERVAL:-0.5}"
# WAIT must allow time for ROS2 discovery + one message (1.5–2s); 0.8s often too short
WAIT="${WAIT:-1.8}"
# Planner and our nodes publish RELIABLE; use default (reliable) for boat_pose, cmd_vel, fused_buoys
# (Only MAVROS sensor topics like global_position use best_effort.)

# MAVROS base: try /mavros first, else /uas1/mavros
M="/mavros"
raw=$(timeout 2 ros2 topic echo --once --qos-reliability best_effort "/mavros/global_position/global" 2>/dev/null) || true
if [[ -z "$raw" ]] || ! echo "$raw" | grep -q "latitude:"; then
  raw=$(timeout 2 ros2 topic echo --once --qos-reliability best_effort "/uas1/mavros/global_position/global" 2>/dev/null) || true
  [[ -n "$raw" ]] && echo "$raw" | grep -q "latitude:" && M="/uas1/mavros"
fi

TMP="${TMPDIR:-/tmp}"
trap 'rm -f "${TMP}"/echo_topics_*.$$ 2>/dev/null; exit' EXIT INT TERM

while true; do
  # Fetch all topics in parallel (default QoS = reliable to match boat_state, planner, fusion)
  # Cmd_vel: planner may publish to /uas1/mavros/... while GPS is on /mavros/...; try both
  timeout "$WAIT" ros2 topic echo --once /boat_pose 2>/dev/null > "${TMP}/echo_topics_bp.$$" &
  timeout "$WAIT" ros2 topic echo --once "/mavros/setpoint_velocity/cmd_vel_unstamped" 2>/dev/null > "${TMP}/echo_topics_cv_m.$$" &
  timeout "$WAIT" ros2 topic echo --once "/uas1/mavros/setpoint_velocity/cmd_vel_unstamped" 2>/dev/null > "${TMP}/echo_topics_cv_u.$$" &
  timeout "$WAIT" ros2 topic echo --once /fused_buoys 2>/dev/null > "${TMP}/echo_topics_fb.$$" &
  wait
  # Use whichever cmd_vel topic had data (planner usually publishes to /uas1/mavros when MAVROS uses that prefix)
  if echo "$(cat "${TMP}/echo_topics_cv_u.$$" 2>/dev/null)" | grep -q "linear:"; then
    cp "${TMP}/echo_topics_cv_u.$$" "${TMP}/echo_topics_cv.$$"
  else
    cp "${TMP}/echo_topics_cv_m.$$" "${TMP}/echo_topics_cv.$$"
  fi

  clear
  echo "=== Topics — refresh ${INTERVAL}s — $(date '+%H:%M:%S') — Ctrl+C to exit ==="
  echo ""

  # ---- Boat pose ----
  raw=$(cat "${TMP}/echo_topics_bp.$$" 2>/dev/null) || true
  if echo "$raw" | grep -qE "east|north|heading_rad"; then
    east=$(echo "$raw" | awk '/east:/{print $2;exit}')
    north=$(echo "$raw" | awk '/north:/{print $2;exit}')
    hrad=$(echo "$raw" | awk '/heading_rad:/{print $2;exit}')
    hdeg=$(awk -v r="${hrad:-0}" 'BEGIN{printf "%.1f", r*57.295779513}')
    printf "  Boat pose    east %.1f m   north %.1f m   heading %.1f°\n" "${east:-0}" "${north:-0}" "${hdeg:-0}"
  else
    echo "  Boat pose    (no message)"
  fi
  echo ""

  # ---- Velocity setpoint ----
  raw=$(cat "${TMP}/echo_topics_cv.$$" 2>/dev/null) || true
  if echo "$raw" | grep -q "linear:"; then
    lx=$(echo "$raw" | awk '/linear:/{l=1;next} l&&/x:/{print $2;exit}')
    ly=$(echo "$raw" | awk '/linear:/{l=1;next} l&&/y:/{print $2;exit}')
    printf "  Cmd velocity   north %.1f   east %.1f  (m/s)\n" "${lx:-0}" "${ly:-0}"
  else
    echo "  Cmd velocity   (no message)"
  fi
  echo ""

  # ---- Fused detections ----
  raw=$(cat "${TMP}/echo_topics_fb.$$" 2>/dev/null) || true
  if echo "$raw" | grep -q "buoys:"; then
    n=$(echo "$raw" | grep -cE "^[[:space:]]+id:" 2>/dev/null) || n=0
    echo "  Fused buoys   ${n} track(s)"
    echo "$raw" | awk '
      /^[[:space:]]*id:[[:space:]]/ { id=$2; r=""; name="" }
      /^[[:space:]]*range:[[:space:]]/ && r=="" { r=$2 }
      /^[[:space:]]*class_name:[[:space:]]/ { name=$2; gsub(/^["'\''"]|["'\''"]$/,"",name); if(name=="") name="?"; if(id!="" && r!="") printf "    #%s  %s  %.1fm\n", id, name, r+0 }
    ' 2>/dev/null | head -8
  else
    echo "  Fused buoys   (no message)"
  fi

  rm -f "${TMP}"/echo_topics_bp.$$ "${TMP}"/echo_topics_cv.$$ "${TMP}"/echo_topics_cv_m.$$ "${TMP}"/echo_topics_cv_u.$$ "${TMP}"/echo_topics_fb.$$ 2>/dev/null || true
  sleep "$INTERVAL"
done
