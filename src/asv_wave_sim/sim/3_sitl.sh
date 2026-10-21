#!/bin/bash
# Terminal 3: ArduPilot Rover SITL + MAVProxy, homed at the course origin so GPS matches the
# answer key. Type MAVProxy commands here (mode GUIDED, arm throttle, ...).
# SITL_GUI=1 adds the MAVProxy console and map windows.
. "$(dirname "${BASH_SOURCE[0]}")/env.sh"
sim_load_course || true
HOME_LOC="${SIM_SITL_HOME:-51.566151,-4.034345,10.0,-135}"

if [ ! -x "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" ]; then
  echo "ArduPilot not found at $ARDUPILOT_DIR (set ARDUPILOT_DIR in config.local.env or run setup_sim.sh)." >&2
  exit 1
fi
[ -f "$ARDUPILOT_VENV/bin/activate" ] && . "$ARDUPILOT_VENV/bin/activate"
EXTRA=()
[ -n "${SITL_GUI:-}" ] && EXTRA+=(--console --map)

# A per-course directory keeps SITL's eeprom/logs out of the ArduPilot checkout and makes runs
# start from the same parameters every time.
RUN_DIR="$SIM_STATE_DIR/sitl"
mkdir -p "$RUN_DIR"
cd "$RUN_DIR"

# The real boat's vehicle tuning (frame, steering/speed controllers, cruise, navigation) from its
# Pixhawk parameter dump, so SITL steers like the boat. Hardware-specific entries (serial, RC,
# sensors, failsafes, PWM) stay at SITL defaults; sitl.parm loads last and wins.
# TEAM_PARAMS=none skips this; TEAM_PARAMS=/path/file.parm uses another dump.
TEAM_PARAMS="${TEAM_PARAMS:-$AUTONOMY_WS/src/task_sequence_coordinator/mav.parm}"
PARAM_FILES=()
if [ "$TEAM_PARAMS" != none ] && [ -f "$TEAM_PARAMS" ]; then
  grep -E '^(FRAME_CLASS|FRAME_TYPE|ATC_|CRUISE_|TURN_|WP_|PIVOT_|NAVL1_|PSC_|MOT_|SPEED_MAX|ACRO_|GUID_|LOIT_)' "$TEAM_PARAMS" |
    grep -v -E '^MOT_PWM_' >"$RUN_DIR/team_tuning.parm"
  PARAM_FILES+=(--add-param-file="$RUN_DIR/team_tuning.parm")
  echo "Team vehicle tuning: $(wc -l <"$RUN_DIR/team_tuning.parm") parameters from $TEAM_PARAMS"
fi
PARAM_FILES+=(--add-param-file="$SIM_DIR/sitl.parm")

echo "SITL home: $HOME_LOC"
exec python3 "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" -v Rover -f rover-skid --model JSON \
  --no-rebuild -w --use-dir="$RUN_DIR" "${PARAM_FILES[@]}" \
  --custom-location="$HOME_LOC" "${EXTRA[@]}" "$@"
