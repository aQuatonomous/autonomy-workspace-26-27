#!/usr/bin/env bash
# logging_lib.sh — source this from comp scripts to auto-log all output.
#
# Usage (at top of comp script, after set -e):
#   TASK_ID=0  # set before sourcing
#   source "$(dirname "${BASH_SOURCE[0]}")/loggers comp ran/logging_lib.sh"
#
# Environment:
#   NOLOG=1   Disable auto-logging
#   SOUND=1   Also launch the sound pipeline
#
# After sourcing, all stdout/stderr is teed to a timestamped log directory.
# On exit, parse_logs.py splits the raw log into per-node files.

_LOGGER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_WS_ROOT="$(cd "${_LOGGER_DIR}/.." && pwd)"

if [ "${NOLOG:-0}" = "1" ]; then
    _LOG_ENABLED=0
else
    _LOG_ENABLED=1
    _LOG_TIMESTAMP="$(date +%Y-%m-%d_%H-%M-%S)"
    _LOG_RUN_DIR="${_LOGGER_DIR}/run_${_LOG_TIMESTAMP}"
    mkdir -p "${_LOG_RUN_DIR}"
    echo "${_LOG_RUN_DIR}" > "${_LOGGER_DIR}/LATEST_RUN.txt"
    _LOG_RAW="${_LOG_RUN_DIR}/raw_log.txt"

    # Redirect all output through tee
    exec > >(tee "${_LOG_RAW}") 2>&1

    echo "=== Logging enabled: ${_LOG_RUN_DIR} ==="
fi

_SOUND_PID=""

start_sound_pipeline() {
    if [ "${SOUND:-0}" != "1" ]; then return; fi
    echo "=== Starting sound pipeline (audio_capturer, sound_signal, message_node) ==="
    ros2 launch sound_pipeline_launch sound_pipeline.launch.py &
    _SOUND_PID=$!
}

stop_sound_pipeline() {
    if [ -n "$_SOUND_PID" ]; then
        echo "=== Stopping sound pipeline (pid $_SOUND_PID) ==="
        kill -TERM "$_SOUND_PID" 2>/dev/null || true
        wait "$_SOUND_PID" 2>/dev/null || true
        _SOUND_PID=""
    fi
}

parse_and_summarize() {
    if [ "$_LOG_ENABLED" = "1" ]; then
        echo ""
        echo "========================================="
        echo " Parsing logs by node..."
        echo "========================================="
        _PARSE_ARGS=("${_LOG_RAW}" "${_LOG_RUN_DIR}")
        [ -n "${TASK_ID:-}" ] && _PARSE_ARGS+=("${TASK_ID}")
        python3 "${_LOGGER_DIR}/parse_logs.py" "${_PARSE_ARGS[@]}" || true
        echo "Logs in: ${_LOG_RUN_DIR}"
    fi
}
