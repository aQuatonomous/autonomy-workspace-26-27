# RoboBoat simulator (2027 and 2026 courses)

Gazebo Harmonic courses for RoboBoat 2027 (Core tiers of Tasks 1-5 and the Proof of Readiness
gates, from the 2027 primer) and RoboBoat 2026 (Tasks 1, 2, 3 and 5, from the handbook), the boat with
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
./run_competition.sh --seed 0 --task 1     # 2027 course as sketched, boat before Task 1, planner task 1
./run_competition.sh --seed 7 --task 3     # randomized layout, boat before the marina
./run_competition.sh --seed 0 --task 123   # no start of its own: the course start (a full run)
./run_competition.sh --course readiness --task 1   # 2027 Proof of Readiness: the two gates alone
./run_competition.sh --course 2026 --task 2        # the 2026 course
./run_competition.sh --seed 3 --no-stack   # everything except the autonomy stack
./stop_competition.sh                      # ends the judge first so the run report is written
```

`--course` picks the course: `2027` (default), `readiness` or `2026`. `--task` is the planner's
`task_id`, and the boat starts `task_start_standoff` (3 m, in the course YAML) before that task's
entrance, facing in, so each task can be tested on its own. A task without its own start (the
planner's sequence mode `123`, or no `--task`) uses the course's default start: the sketched start
on the 2027 course, before Task 1 on the 2026 course. The answer key lists every task's start
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

`tools/generate_course.py` turns a course YAML into a world plus an answer key
(`course/generated/<course>_seed_N/`): every element's world position, lat/lon, gate lines, slips
and the correct answers. Seed 0 is the layout as drawn; other seeds move things within the limits
listed under `randomize:` in the YAML. The SITL home is the world origin, so GPS from MAVROS and the
answer key share one datum.

### 2027 (`course/roboboat2027.yaml`, `tools/course2027.py`)

Built from the RoboBoat 2027 Primer & Task Ideas, the only 2027 document so far: positions are read
off its course sketch, sizes reuse the 2026 handbook for elements that carry over, and every guess
is tagged `PLACEHOLDER` in the YAML. Update those when the 2027 handbook is out. Core tiers only:

| Task | In the sim |
|---|---|
| 1 Safe Passage | Light buoys along a passage from the ENTRY to the EXIT buoy (both lit blue): GREEN on its port side, RED on starboard, BLACK (off) buoys as debris. Seeds > 0 generate a new path. |
| 2 Emergency Response Sprint | Polyform A-2 buoys in six colours and numbered beacons 1-3; one beacon is lit green (START). The target colour is sent on the tasking topic. |
| 3 Navigate the Marina | One dock, three slips with green/red indicators, number banners, a vessel in a red slip. |
| 4 Delivery Run | Five stationary vessels with cross / circle / square / triangle signs; RoboCommand assigns one by shape. |
| 5 Dynamic Response | A rally point RoboCommand sends during the run. |
| Proof of Readiness | `--course readiness`: two red/green gates, 6-10 ft wide, 25-100 ft apart, start > 6 ft before. |

Not modelled until the handbook specifies them: Task 1 colour changes (Advanced, Disruptive),
acoustic pingers (Task 2 Advanced, Disruptive), the fire / water delivery and the window light
sequence (Task 3 Advanced, Disruptive), multiple deliveries and two-boat tasks (Tasks 4, 5).

### 2026 (`course/roboboat2026.yaml`)

The dimensions are taken from the 2026 handbook diagrams (`course/reference/`).

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

- 2027: scores the Core tier of each task (`sim_judge/referee2027.py`; the rules it applies are
  listed in every run report). The primer has no scoring tables, so the points are PLACEHOLDERS that
  only make runs comparable. Task 1: circle ENTRY clockwise, pass GREEN to port and RED to
  starboard, circle EXIT counter-clockwise, report ENTRY/EXIT (GatePass) and the buoys; Task 2:
  start at the green beacon, loop the target, leave the field (timed); Task 3: dock in a green
  slip; Task 4: hold near the assigned vessel after the DELIVERY message; Task 5: reach the rally
  point after the RALLY message; readiness: both gates, no strikes.
- 2026: scores Tasks 1, 2, 3, 5 per the handbook: gate entry/exit, indicator circling, debris
  reports within 3 m, the Task 3 loop direction and time, slip choice and docking report.
- Strikes for buoy/vessel contact; leaving the course boundary ends the run.
- Heartbeat checks (rate, gaps, position error, current task) and report validity.
- Writes `runs/<time>_seed<N>.json` and `.md` when stopped.

`ros2 run sim_judge send_test_reports` sends a correct set of reports to check the judge alone.

### RoboCommand tasking (2027)

In 2027 RoboCommand also sends messages *to* the boat (Task 4 deliveries, Task 5 rally). The real
format is not published, so the judge stands in with JSON on `/sim/robocommand/tasking`
(`std_msgs/String` in ROS, bridged from gz), each message re-sent every 2 s with the same `seq`:

```json
{"type": "SPRINT_TARGET", "task": 2, "target_color": "blue", "seq": 1, "issued_sim_t": 0.4}
{"type": "DELIVERY", "task": 4, "targets": [{"shape": "circle"}], "seq": 2, "issued_sim_t": 120.0}
{"type": "RALLY", "task": 5, "lat": 27.37398, "lon": -82.45285, "seq": 3, "issued_sim_t": 240.0}
```

When they are sent (sim seconds after the judge starts) is in the answer key (`tasking`), from
`tasking_at` in the YAML: on a full run (course start) DELIVERY at 120 s and RALLY at 240 s; with
`--task 4` or `--task 5` the one message at 5 s. `6_judge.sh --no-tasking` sends none. Replace the
stand-in with the real message handling once RoboNation publishes it.

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
