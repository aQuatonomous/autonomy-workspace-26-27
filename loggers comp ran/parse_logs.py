#!/usr/bin/env python3
"""
Parse ROS2 launch log output and split it by node name.

Usage:
  # From a saved log file:
  python3 parse_logs.py /path/to/raw_log.txt [output_dir]

  # Piped from ros2 launch:
  ros2 launch global_planner global_planner.launch.py 2>&1 | python3 parse_logs.py -

Output:
  Creates a timestamped directory inside 'loggers comp ran/' with:
    - raw_log.txt           (full unmodified log)
    - <node_name>.log       (one file per detected node)
    - _launch.log           (ros2 launch infrastructure messages)
    - _shell.log            (shell banners / script output)
    - _unmatched.log        (lines that couldn't be attributed)
    - summary.txt           (line counts per node + coverage check)
"""

import os
import re
import sys
from collections import defaultdict
from datetime import datetime

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# --- Regexes ---
# [node_name-N] rest_of_line  (ros2 launch bracket prefix)
NODE_BRACKET_RE = re.compile(r"^\[([A-Za-z0-9_./-]+-\d+)\]\s*(.*)")
# [LEVEL] [timestamp] [node_name]: message  (rclpy/rclcpp logger, e.g. ros2 run)
NODE_LOGGER_RE = re.compile(
    r"^\[(INFO|WARN|ERROR|DEBUG|FATAL)\]\s*\[\d+\.\d+\]\s*\[([A-Za-z0-9_./-]+)\]:\s*(.*)"
)
# [INFO] [launch]: ...  (launch infrastructure)
LAUNCH_RE = re.compile(r"^\[(INFO|WARN|ERROR|DEBUG|FATAL)\]\s*\[\S+\]\s*\[launch\]")
# Shell banners: === ... ===
SHELL_BANNER_RE = re.compile(r"^===\s.*===\s*$")
# Other script output (lines from set_camera_fps, sourcing, etc.)
SCRIPT_OUTPUT_RE = re.compile(
    r"^(Setting cameras|Camera \d|Frame rate|Verifying|Width/Height|Pixel Format|"
    r"Streaming Parameters|Capabilities|Frames per|Read buffers|Done!|Press Ctrl|"
    r"^\s*✓|^\t)"
)

# All nodes that exist across all tasks. Used for coverage reporting.
ALL_KNOWN_NODES = {
    # MAVROS
    "mavros_node",
    # Global frame
    "boat_state_node",
    "detection_to_global_node",
    # LiDAR pipeline
    "unitree_lidar_ros2_node",
    "lidar_range_filter",
    "buoy_detector",
    "buoy_tracker",
    "buoy_visualizer",
    "tracked_buoy_visualizer",
    # CV pipeline (single camera)
    "v4l2_camera_node",
    "camera1_node",
    "vision_preprocessing",
    "preprocess_camera1",
    "vision_inference",
    "inference_camera1",
    "vision_combiner",
    "detection_combiner",
    "maritime_distance_estimator",
    # CV pipeline (3 camera extras)
    "camera0_node",
    "camera2_node",
    "preprocess_camera0",
    "preprocess_camera2",
    "inference_camera0",
    "inference_camera2",
    # Conditional CV nodes
    "indicator_buoy_processor",
    "task4_supply_processor",
    # Fusion
    "vision_lidar_fusion",
    # Planner
    "global_planner_node",
    # Sound pipeline
    "audio_capturer_node",
    "sound_signal",
    "message_node",
}

TASK_EXPECTED_NODES = {
    0: {  # Task Test (single camera, no indicator/task4)
        "mavros_node", "boat_state_node", "detection_to_global_node",
        "unitree_lidar_ros2_node", "lidar_range_filter",
        "buoy_detector", "buoy_tracker", "buoy_visualizer", "tracked_buoy_visualizer",
        "v4l2_camera_node", "vision_preprocessing", "vision_inference",
        "vision_combiner", "maritime_distance_estimator",
        "vision_lidar_fusion", "global_planner_node",
    },
    1: {  # Task 1 (same as test)
        "mavros_node", "boat_state_node", "detection_to_global_node",
        "unitree_lidar_ros2_node", "lidar_range_filter",
        "buoy_detector", "buoy_tracker", "buoy_visualizer", "tracked_buoy_visualizer",
        "v4l2_camera_node", "vision_preprocessing", "vision_inference",
        "vision_combiner", "maritime_distance_estimator",
        "vision_lidar_fusion", "global_planner_node",
    },
    2: {  # Task 2 (+ indicator buoy)
        "mavros_node", "boat_state_node", "detection_to_global_node",
        "unitree_lidar_ros2_node", "lidar_range_filter",
        "buoy_detector", "buoy_tracker", "buoy_visualizer", "tracked_buoy_visualizer",
        "v4l2_camera_node", "vision_preprocessing", "vision_inference",
        "vision_combiner", "maritime_distance_estimator",
        "indicator_buoy_processor",
        "vision_lidar_fusion", "global_planner_node",
    },
    3: {  # Task 3 (+ indicator buoy)
        "mavros_node", "boat_state_node", "detection_to_global_node",
        "unitree_lidar_ros2_node", "lidar_range_filter",
        "buoy_detector", "buoy_tracker", "buoy_visualizer", "tracked_buoy_visualizer",
        "v4l2_camera_node", "vision_preprocessing", "vision_inference",
        "vision_combiner", "maritime_distance_estimator",
        "indicator_buoy_processor",
        "vision_lidar_fusion", "global_planner_node",
    },
    4: {  # Task 4 (+ indicator buoy + task4 supply)
        "mavros_node", "boat_state_node", "detection_to_global_node",
        "unitree_lidar_ros2_node", "lidar_range_filter",
        "buoy_detector", "buoy_tracker", "buoy_visualizer", "tracked_buoy_visualizer",
        "v4l2_camera_node", "vision_preprocessing", "vision_inference",
        "vision_combiner", "maritime_distance_estimator",
        "indicator_buoy_processor", "task4_supply_processor",
        "vision_lidar_fusion", "global_planner_node",
    },
}

# Node name aliases: some launch files use different names for same executable
NODE_ALIASES = {
    "camera1_node": "v4l2_camera_node",
    "camera0_node": "v4l2_camera_node",
    "camera2_node": "v4l2_camera_node",
    "preprocess_camera0": "vision_preprocessing",
    "preprocess_camera1": "vision_preprocessing",
    "preprocess_camera2": "vision_preprocessing",
    "inference_camera0": "vision_inference",
    "inference_camera1": "vision_inference",
    "inference_camera2": "vision_inference",
    "detection_combiner": "vision_combiner",
}


def node_key(raw_name: str) -> str:
    """Normalize 'node_name-1' to 'node_name'."""
    parts = raw_name.rsplit("-", 1)
    if len(parts) == 2 and parts[1].isdigit():
        return parts[0]
    return raw_name


def parse_lines(lines):
    """Return dict of {node_name: [lines]}, launch lines, shell lines, and unmatched."""
    by_node = defaultdict(list)
    launch_lines = []
    shell_lines = []
    unmatched = []
    current_node = None

    for line in lines:
        # Launch infrastructure: [INFO] [timestamp] [launch]: ...
        if LAUNCH_RE.match(line):
            launch_lines.append(line)
            current_node = None
            continue

        # Shell banners: === ... ===
        if SHELL_BANNER_RE.match(line):
            shell_lines.append(line)
            current_node = None
            continue

        # Script-level output (camera config, sourcing, etc.)
        if SCRIPT_OUTPUT_RE.match(line):
            shell_lines.append(line)
            current_node = None
            continue

        # ROS2 launch bracket prefix: [node_name-N] ...
        m = NODE_BRACKET_RE.match(line)
        if m:
            current_node = node_key(m.group(1))
            by_node[current_node].append(line)
            continue

        # ROS2 logger format: [LEVEL] [timestamp] [node_name]: ...
        m2 = NODE_LOGGER_RE.match(line)
        if m2:
            current_node = node_key(m2.group(2))
            by_node[current_node].append(line)
            continue

        # Continuation line: attribute to current node if we have one
        if current_node is not None and line.strip():
            by_node[current_node].append(line)
        elif line.strip():
            unmatched.append(line)
        # Blank lines: don't attribute, just skip

    return dict(by_node), launch_lines, shell_lines, unmatched


def resolve_alias(name):
    """Return the canonical name if this is an alias."""
    return NODE_ALIASES.get(name, name)


def check_coverage(found_nodes, task_id=None):
    """Return (expected, found, missing, unexpected) sets for reporting."""
    if task_id is not None and task_id in TASK_EXPECTED_NODES:
        expected = set()
        for n in TASK_EXPECTED_NODES[task_id]:
            expected.add(resolve_alias(n))
    else:
        expected = None

    canonical_found = set()
    for n in found_nodes:
        canonical_found.add(resolve_alias(n))

    if expected is None:
        return None, canonical_found, None, None

    missing = expected - canonical_found
    unexpected = canonical_found - expected - {"_launch", "_shell"}
    return expected, canonical_found, missing, unexpected


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    source = sys.argv[1]
    out_dir = sys.argv[2] if len(sys.argv) > 2 else None
    task_id_str = sys.argv[3] if len(sys.argv) > 3 else None
    task_id = int(task_id_str) if task_id_str is not None else None

    if source == "-":
        lines = sys.stdin.readlines()
    else:
        with open(source, "r") as f:
            lines = f.readlines()

    if not lines:
        print("No log lines to parse.")
        sys.exit(0)

    if out_dir is None:
        ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        out_dir = os.path.join(SCRIPT_DIR, f"run_{ts}")
    else:
        ts = os.path.basename(out_dir.rstrip("/"))
    os.makedirs(out_dir, exist_ok=True)

    if source != "-":
        raw_dest = os.path.join(out_dir, "raw_log.txt")
        if not os.path.exists(raw_dest) or os.path.abspath(source) != os.path.abspath(raw_dest):
            with open(raw_dest, "w") as f:
                f.writelines(lines)

    by_node, launch_lines, shell_lines, unmatched = parse_lines(lines)

    for node_name, node_lines in sorted(by_node.items()):
        safe_name = node_name.replace("/", "_").replace(" ", "_")
        with open(os.path.join(out_dir, f"{safe_name}.log"), "w") as f:
            f.writelines(node_lines)

    if launch_lines:
        with open(os.path.join(out_dir, "_launch.log"), "w") as f:
            f.writelines(launch_lines)

    if shell_lines:
        with open(os.path.join(out_dir, "_shell.log"), "w") as f:
            f.writelines(shell_lines)

    if unmatched:
        with open(os.path.join(out_dir, "_unmatched.log"), "w") as f:
            f.writelines(unmatched)

    # Auto-detect task_id from raw log if not provided
    if task_id is None:
        for line in lines[:30]:
            if "task_id:=" in line:
                try:
                    tid = line.split("task_id:=")[1].split()[0].strip()
                    task_id = int(tid)
                except (ValueError, IndexError):
                    pass
                break

    expected, found, missing, unexpected = check_coverage(by_node.keys(), task_id)

    node_line_count = sum(len(v) for v in by_node.values())
    total_lines = len(lines)

    with open(os.path.join(out_dir, "summary.txt"), "w") as f:
        f.write(f"Log run: {ts}\n")
        if task_id is not None:
            f.write(f"Task ID: {task_id}\n")
        f.write(f"Total lines: {total_lines}\n")
        f.write(f"Nodes detected: {len(by_node)}\n\n")

        f.write("--- Per-node line counts ---\n")
        for node_name in sorted(by_node.keys()):
            f.write(f"  {node_name:40s} {len(by_node[node_name]):6d} lines\n")

        f.write(f"\n--- Infrastructure ---\n")
        f.write(f"  {'_launch (ros2 launch msgs)':40s} {len(launch_lines):6d} lines\n")
        f.write(f"  {'_shell (script banners/output)':40s} {len(shell_lines):6d} lines\n")
        if unmatched:
            f.write(f"  {'_unmatched':40s} {len(unmatched):6d} lines\n")

        if expected is not None:
            f.write(f"\n--- Coverage (task_id={task_id}) ---\n")
            f.write(f"  Expected nodes: {len(expected)}\n")
            f.write(f"  Found nodes:    {len(found)}\n")
            if missing:
                f.write(f"\n  MISSING nodes ({len(missing)}):\n")
                for n in sorted(missing):
                    f.write(f"    ✗ {n}\n")
            else:
                f.write(f"\n  ✓ All expected nodes present.\n")
            if unexpected:
                f.write(f"\n  Extra nodes (not in expected set):\n")
                for n in sorted(unexpected):
                    f.write(f"    + {n}\n")

    # Terminal output
    print(f"Logs saved to: {out_dir}")
    print(f"  Nodes: {', '.join(sorted(by_node.keys())) or 'none'}")
    print(f"  Total: {total_lines} lines ({node_line_count} node, "
          f"{len(launch_lines)} launch, {len(shell_lines)} shell, "
          f"{len(unmatched)} unmatched)")
    if task_id is not None and missing:
        print(f"\n  ⚠ MISSING NODES for task {task_id}:")
        for n in sorted(missing):
            print(f"    ✗ {n}")
    elif task_id is not None:
        print(f"  ✓ All expected nodes for task {task_id} present.")


if __name__ == "__main__":
    main()
