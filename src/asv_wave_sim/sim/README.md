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

Shared machine (several accounts on the sim Jetson): `env.sh` gives each user their own
`GZ_PARTITION` (the user name) and `ROS_DOMAIN_ID` (from the user id), so two people's sims do not
see each other's topics. In a plain shell, run `. sim/env.sh` before `ros2 topic` or `gz topic`.

The sim Jetson keeps one copy of ArduPilot, its venv, the ArduPilot Gazebo plugin, the TensorRT
engines and the repo's git history under `/opt/aqua` (group `aqua`); `config.env` uses it whenever
it exists. A per-user copy of those is about 6 GB, which the Jetson's 54 GB disk cannot spare.
`setup_shared.sh` made it once from a working `setup_sim.sh` install (`--check` shows the state).
New account on that machine:

```bash
git clone --reference /opt/aqua/autonomy-workspace-26-27.git <repo url> ~/autonomy-workspace-26-27
~/autonomy-workspace-26-27/src/asv_wave_sim/sim/setup_sim.sh   # builds only your workspace
```

An existing clone switches to the shared history (its `.git` drops from ~510 MB to a few MB;
local branches and uncommitted work are kept) with
`setup_shared.sh --link ~/autonomy-workspace-26-27`. Never delete
`/opt/aqua/autonomy-workspace-26-27.git`: those clones read their history from it.

WSL2: Gazebo renders through the Windows GPU driver (`GALLIUM_DRIVER=d3d12`, set by `env.sh`).
Install the current NVIDIA Windows driver; do not install a Linux NVIDIA driver inside WSL.
`setup_sim.sh` warns when rendering falls back to the software renderer (llvmpipe).

## Run

```bash
./run_competition.sh --seed 0 --task 1     # handbook layout, planner Task 1
./run_competition.sh --seed 7 --task 3     # randomized layout within the rulebook ranges
./run_competition.sh --seed 0 --task 2     # boat starts just before Task 2's channel
./run_competition.sh --seed 0 --task 123   # Task 1 -> 2 -> 3 in sequence, as at competition
./run_competition.sh --seed 3 --no-stack   # everything except the autonomy stack
./stop_competition.sh                      # ends the judge first so the run report is written
```

`--task` is the planner's `task_id`. Task 1 and the sequence (`123`, the planner's
TaskMaster running 1 -> 2 -> 3) start at the handbook start before Task 1; `--task 2`, `3` and `5`
put the boat `task_start_standoff` (3 m, `course/roboboat2026.yaml`) before that task's entrance,
facing in, so each task can be tested on its own. The answer key lists every task's start
(`starts`) and which one the world uses (`start_task`).

One tmux session (`roboboat`) with a window per component: `gazebo`, `bridges`, `sitl`,
`mavros`, `judge`, `stack`, `foxglove`, `shell` (and `rviz` with `--rviz`). `Ctrl+b w` lists them.
Other options: `--waves`, `--wind`, `--wind-from`, `--light`, `--time` (see Weather and light), `--full-res` or `--camera WxH@HZ`, `--no-cv`, `--no-arm`, `--no-attach`,
`--world FILE`, `--legacy` (the original world, no judge), `--labels` (see
Autolabel), `--headless` (below).

Over SSH (e.g. the sim Jetson from a laptop), pass `--headless`: the Gazebo server then renders
the sensors on the machine's GPU with no window. Watch the run in Foxglove (below). The Gazebo
GUI needs OpenGL 3.3, which `ssh -Y` to an X server on Windows (VcXsrv) does not provide; on a
Linux laptop or in a VNC desktop on the sim machine, `. sim/env.sh && sim_load_course && gz sim -g`
opens it next to a running server.

### Watching a run (Foxglove)

The `foxglove` window runs `foxglove_bridge` and the sim viewer (`9_foxglove.sh`). Each user on
a shared machine gets their own port, `FOXGLOVE_PORT` (8700 + user id % 100; `run_competition.sh`
prints it). On your laptop install [Lichtblick](https://github.com/lichtblick-suite/lichtblick/releases)
(free, open source) or [Foxglove](https://foxglove.dev/download), then "Open connection" ->
Foxglove WebSocket -> `ws://<sim machine>:<port>`, or `ws://localhost:<port>` after
`ssh -L <port>:localhost:<port> <user>@<sim machine>` when the port is not reachable directly.

Useful panels:
- 3D, display frame `map`: `/sim/course` (true course: buoys, gate lines, docks, boundary),
  `/sim/boat_truth` (true pose), `/boat_pose_stamped` (where the stack thinks it is),
  `/planned_path`, `/points_filtered` and `/buoy_markers` (LiDAR), the TF tree.
- Image: `/sim/view/camera0/compressed` (and camera1, camera2): JPEG at 5 Hz. Avoid the raw
  `/cameraN/image_raw` topics over Wi-Fi; each is about 25 MB/s.
- Log (`/rosout`) and Raw Messages (`/cur_task`, `/messages/gate_pass`) for the reports.

Save the layout from the app (Layouts -> Export) as `sim/foxglove_layout.json` and commit it so
everyone starts from the same one. The ground truth topics are for watching only; the stack never
subscribes to them.

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

Element models come from `tools/make_course_models.py` (rulebook dimensions, origin at the
waterline). The Polyform A-0 / A-2 buoys and the colour indicator buoy use the mechanical team's
CAD (`course/cad/*.step`): `tools/convert_cad.py` turns it into `roboboat_common/meshes/*.obj`, split
into pieces that take different colours (needs `pip install gmsh trimesh scipy shapely`, not part of
the sim runtime). To add another part, put its STEP export in `course/cad/`, add it to
`PARTS` in `convert_cad.py`, then use the mesh in `make_course_models.py`.

## Weather and light

Set per run with the same options on `run_competition.sh`, `1_gazebo.sh` or `select_course.sh`; the
answer key records what was used (`weather`).

- `--waves flat|calm_lake|breezy_lake|windy_lake` sets the wind and the waves together (1.5, 4 and
  7 m/s mean wind with gusts). The wind pushes the hull through Gazebo's WindEffects system, sized
  from the boat's windage (`boat.wind_cda` in the YAML), so expect drift and weather-vaning at low
  speed and during docking. `--wind M/S` and `--wind-from DEG` (compass, where it blows from)
  override it; the waves follow the same direction. Seeds other than 0 get a random wind bearing.
- `--light morning|midday|afternoon|low_sun|overcast` places the sun for that local time on the
  venue date in the YAML (`--time HH:MM` for any other time). Low sun is dimmer and warmer, the
  sky and ambient light follow it; `low_sun` puts glare in the cameras on westward legs and
  `overcast` gives flat light. `--shadows` adds sun shadows (costs GPU time).

## Judge

`6_judge.sh` (package `sim_judge`) listens where `message_node` sends RoboCommand frames
(`$R` + length + protobuf + `!!`, port 50000) and reads the boat's true pose from Gazebo.

- Scores Tasks 1, 2, 3, 5 per the handbook: gate entry/exit, indicator circling, debris reports
  within 3 m, the Task 3 loop direction and time, slip choice and docking report.
- Strikes for buoy/vessel contact; leaving the course boundary ends the run.
- Heartbeat checks (rate, gaps, position error, current task) and report validity.
- Writes `runs/<time>_seed<N>.json` and `.md` when stopped.

`ros2 run sim_judge send_test_reports` sends a correct set of reports to check the judge alone.

## Autolabel (CV training data)

The sim knows what every course element is and where it is, so it can label camera frames for
the CV team without anyone drawing boxes. Package `sim_autolabel`.

```bash
./run_competition.sh --seed 4 --task 2 --labels    # adds an `autolabel` window that records
./8_autolabel.sh --max-frames 2000                 # or on its own, with a course selected --labels
ros2 run sim_autolabel export datasets/ --out ~/datasets/sim_main --preview 30   # every session under it
export ROBOFLOW_API_KEY=...                        # Roboflow: Settings > API Keys
ros2 run sim_autolabel upload ~/datasets/sim_main --workspace WS --project PROJECT --limit 20   # trial
ros2 run sim_autolabel upload ~/datasets/sim_main --workspace WS --project PROJECT
```

The CV team trains YOLO on Roboflow, exports `.pt` -> ONNX -> TensorRT engine
(`computer_vision/models/TENSORRT.md`) and runs it with OpenCV preprocessing in `vision_inference`.
The sim frames go into the same Roboflow project as the real images, as their own upload batch.

- `--labels` (`select_course.sh`, `1_gazebo.sh`, `run_competition.sh`) gives each boat camera a
  Gazebo bounding-box camera twin with the same pose, lens and resolution, which publishes the
  boxes of the labelled elements in view on `/autolabel/camera{0,1,2}`, plus a plain camera twin
  for the clean image on `/autolabel/camera{0,1,2}_rgb` (the box camera draws its boxes into its
  own image). Both run at `SIM_LABEL_RATE` (2 Hz default, `config.env`; nearby frames are
  near-duplicates anyway). Each view renders three times, so expect the sim to run slower.
- Boxes cover only what the camera sees: an element hidden behind another gets no box and a
  partly hidden one gets a box around its visible part. There is no minimum size at recording;
  `export --min-box-px` drops the tiny ones (below).
- Labels: `generate_course.py` adds a gz Label plugin to each element (`tools/sim_labels.py`
  has the IDs; the answer key carries the table). The stationary vessels label only their sign
  symbol and the number banners only their numeral, inside the model (`make_course_models.py`),
  because that is what the CV models box. Boundary markers and the boat are not labelled.
- `record` writes `sim/datasets/<time>_seed<N>/`: `session.json` (course, weather, labels) and per
  camera `<stamp>.jpg` + `<stamp>.json` (boxes in pixels under sim label names, boat pose). It
  keeps 1 in 10 frames with nothing in view as background examples (`--empty-every`).
- `export` maps sim labels to a CV model's classes using `sim_autolabel/classes.yaml`: `main`
  (aqua_main, in the alphabetical class order of the model that ran at competition) or `numbers`
  (docking numbers, digit only). Labels a dataset does not list become background. Recordings keep the sim labels, so
  after changing the mapping you only export again. Frames are split into train/valid in 30 s
  blocks so near-duplicate frames don't land on both sides. `export` needs only Python (numpy,
  opencv, pyyaml): `python3 -m sim_autolabel.export` runs it from `src/sim_autolabel` without ROS.
- The export is in Roboflow's YOLO layout (`train/`, `valid/`, `data.yaml`): drag the folder into
  the project in the browser, or use `upload`, which also tags every image `sim` and
  `sim_seed<N>` so a dataset version can include or leave out the sim images. `upload` stops if
  the project's class names differ from `classes.yaml` (a different spelling would create new
  classes). It needs `pip install roboflow`. `yolo train data=.../data.yaml` also works directly.
- Boxes are kept or dropped by their size in the model input: `imgsz` in `classes.yaml` (640 for
  the main engine, 960 for the numbers engine, both letterboxed), `--min-box-px` 3 px by default.
- For variety, record several seeds with different `--waves`, `--light` and `--time`, and drive
  the boat yourself (`--no-stack` + MAVProxy) as well as with the planner. Sim images do not look
  like the real cameras: use them alongside the real dataset, not instead of it.

## Sensors

- Airy 96: 900 x 96 points, 360 x 90 deg (horizon to straight down), 10 Hz, on
  `/rslidar_points` (frame `rslidar`) plus its IMU on `/rslidar_imu_data`. Minimum range is
  1.2 m, standing in for the driver's `min_distance`: from this mount the deck and stern fill
  ~85 % of the scan otherwise. Set the same cut in `rslidar_sdk` on the Jetson.
- Cameras: `/camera0/image_raw` (70 deg left), `/camera1` (centre), `/camera2` (70 deg right),
  matching the CV `mounting_angles_deg [-70, 0, 70]`. Resolution and rate come from `SIM_CAMERA`
  in `config.env` (default `960x600@15`, so three cameras render on a mid-range GPU). The real
  cameras are 1920 x 1200 at 30 Hz: use `--full-res` (or `--camera WxH@HZ`) when testing CV
  detection range or confidence thresholds, which the lower resolution makes look worse than on
  the boat. `select_course.sh` writes the boat model with that setting to
  `course/generated/models/ourboat`, and `7_stack.sh` passes the same resolution to CV.
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

The global planner uses `setpoint_raw/local` (its `cmd_mode` parameter, default `raw`).
`cmd_mode:=twist` restores its old `Twist` output on `cmd_vel_topic`, with which the boat cannot
turn toward a goal (it drove backwards in the sim when the goal was off to the side).
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

On the sim Jetson (AGX Orin, MAXN power mode), measured 2026-11-09:

| Run | Real-time factor |
|---|---|
| `--no-stack` | ~0.37 |
| full stack with CV, default cameras (960x600@15) | ~0.3 (also with `--labels`) |
| full stack, `--full-res` (1920x1200@30) | ~0.18; the cameras then deliver ~1.3 frames per sim second, too few for CV |

Use the default camera setting on the Jetson; test detection range with `--full-res` on a desktop
GPU.
