#!/bin/bash
# Ends a run_competition.sh session: stops the judge first (it writes the run report), then the
# rest of the session, then any simulator processes that outlived their window.
SESSION=roboboat
if tmux has-session -t "$SESSION" 2>/dev/null; then
  if tmux list-windows -t "$SESSION" -F '#W' | grep -qx judge; then
    tmux send-keys -t "$SESSION:judge" C-c
    for _ in $(seq 1 10); do pgrep -f 'sim_judge/judge' >/dev/null || break; sleep 0.5; done
  fi
  tmux list-windows -t "$SESSION" -F '#W' | grep -qx stack && tmux send-keys -t "$SESSION:stack" C-c && sleep 3
  tmux kill-session -t "$SESSION"
fi
# Processes started by the sim scripts (patterns are specific to them).
for pat in '[g]z sim' 'sim_vehicle.py -v Rover' '[a]rdurover' 'mavproxy.py' '[r]os_gz_bridge/parameter_bridge' \
           'mavros_node' 'sim_judge/judge' 'message_node/message_node' 'global_planner_node' 'boat_state_node' \
           'detection_to_global_node' 'lidar_range_filter' 'buoy_detector' 'buoy_tracker' 'buoy_visualizer' \
           'vision_preprocessing' 'vision_inference' 'vision_combiner' 'maritime_distance_estimator' \
           'indicator_buoy_processor' 'vision_lidar_fusion' 'arm_guided.sh'; do
  pkill -INT -f "$pat" 2>/dev/null
done
sleep 2
for pat in '[g]z sim' '[a]rdurover' 'mavproxy.py' 'mavros_node' 'vision_inference'; do pkill -KILL -f "$pat" 2>/dev/null; done
echo "Simulation stopped. Run reports: $(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/runs/"
