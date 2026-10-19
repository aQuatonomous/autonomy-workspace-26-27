# RoboBoat 2026 simulator

Gazebo Harmonic course built from the RoboBoat 2026 handbook (Tasks 1, 2, 3 and 5), the boat with
its competition sensors (RoboSense Airy 96 + three cameras), ArduPilot Rover SITL with the real
boat's tuning, MAVROS, the team's autonomy stack, and a judge that scores runs the way the
RoboCommand server and the course judges do.

Runs on Ubuntu 24.04, natively or under WSL2 on Windows 11. Everything runs on one desktop; the
Jetson AGX Orin stays on the boat.

## New computer

```bash
git clone <repo> ~/repos/autonomy-ws-25-26
cd ~/repos/autonomy-ws-25-26/src/asv_wave_sim/sim
./setup_sim.sh --with-cv      # ROS 2 Jazzy, Gazebo, ArduPilot, workspace build, TensorRT engines
./setup_sim.sh --check        # what is installed; changes nothing
```

`setup_sim.sh` asks for sudo for the apt steps, checks each step first, and is safe to re-run.
Leave out `--with-cv` on a machine without an NVIDIA GPU (the stack then runs LiDAR-only).
Paths (ArduPilot checkout, venv, engine cache, RoboCommand port) are in `config.env`; put
overrides in `config.local.env` (not committed).

WSL2: Gazebo renders through the Windows GPU driver (`GALLIUM_DRIVER=d3d12`, set by `env.sh`).
Install the current NVIDIA Windows driver; do not install a Linux NVIDIA driver inside WSL.
`setup_sim.sh` warns when rendering falls back to the software renderer (llvmpipe).

## Run

```bash
./run_competition.sh --seed 0 --task 1     # handbook layout, planner Task 1
./run_competition.sh --seed 7 --task 3     # randomized layout within the rulebook ranges
./run_competition.sh --seed 3 --no-stack   # everything except the autonomy stack
./stop_competition.sh                      # ends the judge first so the run report is written
```

One tmux session (`roboboat`) with a window per component: `gazebo`, `bridges`, `sitl`,
`mavros`, `judge`, `stack`, `shell` (and `rviz` with `--rviz`). `Ctrl+b w` lists them.
Other options: `--waves calm_lake|breezy_lake`, `--no-cv`, `--no-arm`, `--no-attach`,
`--world FILE`, `--legacy` (the original world, no judge), `--cmd-mode` (below).

The numbered scripts start the same pieces one per terminal: `1_gazebo.sh` ... `7_stack.sh`, plus
`5_view_sensors.sh` for RViz (LiDAR cloud, filtered cloud, buoy markers, three cameras).
Run `select_course.sh --seed N` first; every script reads the course it writes.

## Course

`course/roboboat2026.yaml` holds the dimensions taken from the handbook diagrams
(`course/reference/`). `tools/generate_course.py` turns it into a world plus an answer key
(`course/generated/seed_N/`): every element's world position, lat/lon, gate lines, slips and the
correct answers. Seed 0 is the layout as drawn; other seeds move things within the limits listed
under `randomize:` in the YAML. The SITL home is the world origin, so GPS from MAVROS and the
answer key share one datum.

## Judge

`6_judge.sh` (package `sim_judge`) listens where `message_node` sends RoboCommand frames
(`$R` + length + protobuf + `!!`, port 50000) and reads the boat's true pose from Gazebo.

- Scores Tasks 1, 2, 3, 5 per the handbook: gate entry/exit, indicator circling, debris reports
  within 3 m, the Task 3 loop direction and time, slip choice and docking report.
- Strikes for buoy/vessel contact; leaving the course boundary ends the run.
- Heartbeat checks (rate, gaps, position error, current task) and report validity.
- Writes `runs/<time>_seed<N>.json` and `.md` when stopped.

`ros2 run sim_judge send_test_reports` sends a correct set of reports to check the judge alone.

## Sensors

- Airy 96: 900 x 96 points, 360 x 90 deg (horizon to straight down), 10 Hz, on
  `/rslidar_points` (frame `rslidar`) plus its IMU on `/rslidar_imu_data`. Minimum range is
  1.2 m, standing in for the driver's `min_distance`: from this mount the deck and stern fill
  ~85 % of the scan otherwise. Set the same cut in `rslidar_sdk` on the Jetson.
- Cameras: `/camera0/image_raw` (70 deg left), `/camera1` (centre), `/camera2` (70 deg right),
  960 x 600 at 15 Hz, matching the CV `mounting_angles_deg [-70, 0, 70]`.
- Placeholders to replace with measured values: LiDAR and camera mounting poses, dock freeboard,
  venue datum.

## Autonomy stack in the sim

`7_stack.sh` runs the team's packages unmodified: global_frame, the LiDAR buoy pipeline, CV on
the three cameras, CV-LiDAR fusion, the global planner and `message_node`, then sets GUIDED and
arms. Only the wiring is sim-specific:

- LiDAR: `lidar_pipeline_sim.launch.py` runs the same nodes and tuning as
  `pointcloud_filters/launch/buoy_pipeline.launch.py` on `/rslidar_points`, without the Unitree
  driver and with the Unitree mount rotations at 0. Keep its tuning in step with that file.
- CV: TensorRT engines are GPU-specific, so `build_engines.sh` builds them from the ONNX models
  into `~/.cache/aquatonomous/engines/<gpu>_trt<version>/`; the Jetson engines are untouched.
- Planner: started with `ros2 run` and the `PLANNING_PATH` its launch file sets
  (`global_planner.launch.py` passes `map_bounds=[]`, which launch_ros on Jazzy rejects).
- Reports: `message_node` sends to the competition address, 10.10.10.1:50000. The sim adds that
  address to the loopback interface (`sudo ip addr add 10.10.10.1/32 dev lo`, once per boot;
  `run_competition.sh` asks for it) and the judge listens there. `report_pb2.py` is protobuf 6.32
  generated code, so `message_node` gets protobuf 6 on its own `PYTHONPATH`
  (`~/.cache/aquatonomous/message_node_pydeps`, installed by `setup_sim.sh`).
- SITL loads the vehicle tuning (frame, steering/speed controllers, cruise, navigation) from the
  boat's parameter dump `src/task_sequence_coordinator/mav.parm`; `TEAM_PARAMS=none` uses stock
  rover-skid values.

## Driving the boat over MAVROS

The boat is ArduPilot Rover in GUIDED mode behind MAVROS, as on the real boat.

- `/mavros/setpoint_raw/local` (`mavros_msgs/PositionTarget`, `FRAME_LOCAL_NED`, velocity in
  ENU, yaw and yaw rate masked): Rover steers to the velocity direction at that speed.
- `/mavros/setpoint_velocity/cmd_vel_unstamped` (`Twist`): MAVROS adds a yaw rate, and Rover then
  uses only the speed (sign from `linear.x`) and turns at `angular.z`; the velocity direction is
  ignored. With `angular.z = 0` the boat holds its heading.
- `/mavros/setpoint_position/global` or MAVProxy (`mode GUIDED`, `arm throttle`, map clicks) in
  the `sitl` window also work.

## Boat dynamics

`ourboat/model.sdf` uses gz-waves hydrodynamics calibrated to the boat's ArduPilot tuning:
1.18 m/s at 51 % throttle (`CRUISE_SPEED` / `CRUISE_THROTTLE`) and ~190 deg/s pivot at full
differential (1 / `ATC_STR_RAT_FF`). Replace with measured values (straight-line speed at a known
throttle, pivot rate, turning circle) when available.

## Performance

On an RTX 5060 Ti 8 GB with 12 cores the full stack runs at about 0.45 x real time (Gazebo and
SITL run lock-step; the judge scores in sim time). The autonomy nodes run on wall-clock time, so
to them the boat moves at about half speed. Close other GPU-heavy programs; `--no-cv` frees the
most.
