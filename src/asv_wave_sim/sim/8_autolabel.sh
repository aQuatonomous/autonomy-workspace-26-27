#!/bin/bash
# Terminal 8: records labelled CV training frames (sim_autolabel) into sim/datasets/<time>_seed<N>.
# Needs the course selected with --labels (select_course.sh / 1_gazebo.sh / run_competition.sh).
# Extra arguments go to the recorder (e.g. --max-frames 2000, --cameras camera1, --empty-every 0).
. "$(dirname "${BASH_SOURCE[0]}")/env.sh"
if ! sim_load_course || [ -z "$SIM_ANSWER_KEY" ]; then
  echo "No answer key for the selected course. Run select_course.sh --labels first." >&2
  exit 1
fi
if [ "${SIM_LABELS:-0}" != 1 ]; then
  echo "The selected course has no autolabel cameras. Select it again with --labels, e.g." >&2
  echo "  select_course.sh --seed ${SIM_SEED:-0} --labels   (then restart Gazebo)" >&2
  exit 1
fi
exec ros2 run sim_autolabel record --answer-key "$SIM_ANSWER_KEY" "$@"
