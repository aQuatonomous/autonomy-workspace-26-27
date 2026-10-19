#!/usr/bin/env bash
# Open the most recent logger run folder.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "${SCRIPT_DIR}/LATEST_RUN.txt" ]]; then
    LATEST=$(cat "${SCRIPT_DIR}/LATEST_RUN.txt")
    if [[ -d "${LATEST}" ]]; then
        echo "Opening: ${LATEST}"
        xdg-open "${LATEST}" 2>/dev/null || echo "Path: ${LATEST}"
    else
        echo "LATEST_RUN points to missing dir: ${LATEST}"
    fi
else
    # Find most recent run_* folder
    LATEST=$(ls -td "${SCRIPT_DIR}"/run_* 2>/dev/null | head -1)
    if [[ -n "${LATEST}" ]]; then
        echo "Opening: ${LATEST}"
        xdg-open "${LATEST}" 2>/dev/null || echo "Path: ${LATEST}"
    else
        echo "No run folders found. Run the logger first:"
        echo "  ./run_and_log_competition.sh ros2 launch global_planner global_planner.launch.py task_id:=0"
    fi
fi
