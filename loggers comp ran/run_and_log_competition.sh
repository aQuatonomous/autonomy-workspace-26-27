#!/usr/bin/env bash
# run_and_log_competition.sh — wrapper to capture all ROS2 launch output and parse it by node.
#
# Usage:
#   ./run_and_log_competition.sh <launch_command_args...>
#
# Examples:
#   ./run_and_log_competition.sh ./task_test_comp.sh
#   ./run_and_log_competition.sh ./comp_task1.sh
#   SOUND=1 ./run_and_log_competition.sh ./comp_task3.sh      # also launch sound pipeline
#
# Environment variables:
#   SOUND=1       Also launch the sound pipeline (audio_capturer, sound_signal, message_node)
#   TASK_ID=N     Override task ID for coverage reporting (auto-detected from launch args)
#
# On Ctrl-C (or launch exit), the raw log is parsed into per-node files
# inside "loggers comp ran/run_<timestamp>/".

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
TIMESTAMP="$(date +%Y-%m-%d_%H-%M-%S)"
RUN_DIR="${SCRIPT_DIR}/run_${TIMESTAMP}"
mkdir -p "${RUN_DIR}"
echo "${RUN_DIR}" > "${SCRIPT_DIR}/LATEST_RUN.txt"

RAW_LOG="${RUN_DIR}/raw_log.txt"
SOUND_PID=""

# Try to auto-detect task_id from the command line
TASK_ID="${TASK_ID:-}"
if [ -z "$TASK_ID" ]; then
    for arg in "$@"; do
        case "$arg" in
            *task_id:=*) TASK_ID="${arg#*task_id:=}"; TASK_ID="${TASK_ID%% *}" ;;
            *task_test*|*TaskTest*) TASK_ID=0 ;;
            *task1*|*Task1*)  TASK_ID=1 ;;
            *task2*|*Task2*)  TASK_ID=2 ;;
            *task3*|*Task3*)  TASK_ID=3 ;;
            *task4*|*Task4*)  TASK_ID=4 ;;
        esac
    done
fi

echo "========================================="
echo " Logging to: ${RUN_DIR}"
echo " Command:    $*"
[ -n "${TASK_ID}" ] && echo " Task ID:    ${TASK_ID}"
[ "${SOUND:-0}" = "1" ] && echo " Sound:      enabled"
echo "========================================="
echo ""

cleanup() {
    echo ""
    echo "========================================="
    echo " Launch ended. Parsing logs by node..."
    echo "========================================="

    # Kill sound pipeline if we started it
    if [ -n "$SOUND_PID" ]; then
        echo "Stopping sound pipeline (pid $SOUND_PID)..."
        kill -TERM "$SOUND_PID" 2>/dev/null || true
        wait "$SOUND_PID" 2>/dev/null || true
    fi

    PARSE_ARGS=("${RAW_LOG}" "${RUN_DIR}")
    [ -n "${TASK_ID}" ] && PARSE_ARGS+=("${TASK_ID}")
    python3 "${SCRIPT_DIR}/parse_logs.py" "${PARSE_ARGS[@]}"

    echo ""
    echo "Done. Logs in: ${RUN_DIR}"
}
trap cleanup EXIT

# Optionally launch sound pipeline (merged into same output stream)
if [ "${SOUND:-0}" = "1" ]; then
    echo "=== Starting sound pipeline (audio_capturer, sound_signal, message_node) ==="
    source /opt/ros/jazzy/setup.bash
    if [ -f "${WS_ROOT}/install/setup.bash" ]; then
        source "${WS_ROOT}/install/setup.bash"
    fi
    ros2 launch sound_pipeline_launch sound_pipeline.launch.py 2>&1 &
    SOUND_PID=$!
fi

# Run the command, tee stdout+stderr to both terminal and raw log file.
"$@" 2>&1 | tee "${RAW_LOG}"
