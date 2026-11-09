# Simulator guide

The team's RoboBoat simulator: the competition course in Gazebo Harmonic, our boat with its
sensors (RoboSense Airy 96 LiDAR + three cameras), ArduPilot Rover SITL with the real boat's
tuning, MAVROS, our autonomy stack running unmodified, and a judge that scores the run the way
RoboCommand and the course judges do. Courses: **RoboBoat 2027** (default), the 2027 **Proof of
Readiness** gates, and **RoboBoat 2026**.

This is the only simulator guide. The scripts live in `src/asv_wave_sim/sim/`; every command
below is run from there unless it says otherwise.

1. [Quick start](#1-quick-start)
2. [Running the sim](#2-running-the-sim)
3. [Courses, starts, weather](#3-courses-starts-weather)
4. [Judge and RoboCommand](#4-judge-and-robocommand)
5. [CV training data (autolabel)](#5-cv-training-data-autolabel)
6. [How the sim matches the boat](#6-how-the-sim-matches-the-boat)
7. [Performance](#7-performance)
8. [The shared sim Jetson (admin)](#8-the-shared-sim-jetson-admin)
9. [Troubleshooting](#9-troubleshooting)
10. [Where things are](#10-where-things-are)

---

## 1. Quick start

### On the shared sim Jetson (most people)

The sim Jetson (`aquatonomous`, AGX Orin) is reached over Tailscale at `100.113.125.27`. speak to the years autonomy director to get an account,
then from your laptop: `ssh <your account name>@100.113.125.27`.

**First time only:**

```bash
git clone --reference /opt/aqua/autonomy-workspace-26-27.git \
  https://github.com/aQuatonomous/autonomy-workspace-26-27.git ~/autonomy-workspace-26-27
~/autonomy-workspace-26-27/src/asv_wave_sim/sim/setup_sim.sh   # builds your workspace; ArduPilot etc. are shared
```

Everyone works on `main`: run `git pull` before you start and before you push. Use a branch and a
pull request for big changes, and for anything that changes how the boat drives (planner, CV,
`message_node`).

`--reference` makes your clone borrow the shared git history, so it costs a few MB instead of
~510 MB. Do not clone without it on the Jetson; the disk is small (see section 8).

**Every run:**

```bash
cd ~/autonomy-workspace-26-27/src/asv_wave_sim/sim
./run_competition.sh --headless --seed 0 --task 1
```

- `--headless` is needed over SSH (there is no screen to open Gazebo on).
- After a reboot it asks for your sudo password once, to give this machine the competition
  RoboCommand address (`10.10.10.1`). Without it the judge never sees the boat's reports.
- It prints a Foxglove address; open it on your laptop to watch (section 2).
- `tmux attach -t roboboat` shows the windows; `Ctrl+b w` lists them, `Ctrl+b d` leaves them running.
- `./stop_competition.sh` ends the run and writes the judge's report to `runs/`.

### On your own machine

Ubuntu 24.04, natively or under WSL2 on Windows. An NVIDIA GPU is needed for the CV pipeline;
without one the stack runs LiDAR-only (`--no-cv`).

```bash
git clone https://github.com/aQuatonomous/autonomy-workspace-26-27.git ~/autonomy-workspace-26-27
cd ~/autonomy-workspace-26-27/src/asv_wave_sim/sim
./setup_sim.sh --with-cv      # ROS 2 Jazzy, Gazebo, ArduPilot, workspace build, TensorRT engines
./setup_sim.sh --check        # what is installed; changes nothing
./run_competition.sh --seed 0 --task 1
```

`setup_sim.sh` asks for sudo for the apt steps, checks every step first and is safe to re-run.
Leave out `--with-cv` without an NVIDIA GPU. On a Jetson it uses JetPack's CUDA/TensorRT and never
installs the desktop ones. Paths (ArduPilot, venv, engine cache, RoboCommand port) are in
`config.env`; put your own overrides in `config.local.env` (not committed).

WSL2: Gazebo renders through the Windows GPU driver (`GALLIUM_DRIVER=d3d12`, set by `env.sh`).
Install the current NVIDIA Windows driver and never a Linux NVIDIA driver inside WSL.

---

## 2. Running the sim

```bash
./run_competition.sh --seed 0 --task 1            # 2027 course, boat before Task 1, planner task 1
./run_competition.sh --seed 7 --task 3            # randomized layout, boat before the marina
./run_competition.sh --seed 0 --task 123          # planner sequence mode, from the course start
./run_competition.sh --course readiness --task 1  # 2027 Proof of Readiness: two gates
./run_competition.sh --course 2026 --task 2       # the 2026 course
./run_competition.sh --seed 3 --no-stack          # everything but the autonomy stack: drive it yourself
./stop_competition.sh                             # always stop this way (writes the run report)
```

| Option | Effect |
|---|---|
| `--course 2027\|readiness\|2026` | Which course (default 2027) |
| `--seed N` | 0 = the layout as drawn; other seeds randomize it within the rules |
| `--task N` | The planner's `task_id`, and the boat starts in front of that task (section 3) |
| `--headless` | No Gazebo window (needed over SSH); watch in Foxglove |
| `--no-stack`, `--no-cv`, `--no-arm` | Leave out the autonomy stack / CV / automatic GUIDED + arm |
| `--waves`, `--wind`, `--wind-from`, `--light`, `--time`, `--shadows` | Weather and light (section 3) |
| `--full-res`, `--camera WxH@HZ` | Camera resolution (default 960x600@15) |
| `--labels` | Record labelled CV training frames (section 5) |
| `--rviz`, `--no-attach` | RViz window; don't attach to tmux |

One tmux session, `roboboat`, with a window per part: `gazebo`, `bridges`, `sitl`, `mavros`,
`judge`, `stack`, `foxglove`, `shell` (plus `rviz` / `autolabel` when asked for). Every window's
output is also saved to `runs/logs/<time>/<window>.log` (last 5 runs kept). The `shell` window
has the ROS environment loaded; in any other terminal run `. env.sh` first, or you will not see the
sim's topics (each user gets their own ROS domain).

The numbered scripts start the same parts one per terminal (`1_gazebo.sh` ... `9_foxglove.sh`)
after `select_course.sh` has chosen a course; handy when debugging one part.

### Watching a run (Foxglove)

`run_competition.sh` prints your port (8700 + your user id % 100). Install
[Lichtblick](https://github.com/lichtblick-suite/lichtblick/releases) (free) or
[Foxglove](https://foxglove.dev/download), then Open connection -> Foxglove WebSocket ->
`ws://100.113.125.27:<port>` (or `ws://localhost:<port>` after
`ssh -L <port>:localhost:<port> <you>@<machine>` if the port is not reachable directly).

- 3D panel, frame `map`: `/sim/course` (true course), `/sim/boat_truth` (true pose),
  `/boat_pose_stamped` (where the stack thinks it is), `/planned_path`, `/points_filtered`,
  `/buoy_markers`.
- Image panels: `/sim/view/camera0/compressed` (and 1, 2), 5 Hz JPEG. Avoid the raw
  `/cameraN/image_raw` topics over Wi-Fi (~25 MB/s each).
- Log (`/rosout`), Raw Messages (`/cur_task`, `/messages/gate_pass`, `/sim/robocommand/tasking`).

The ground-truth topics are for watching only; the stack never uses them. To see Gazebo itself,
on a Linux desktop or VNC on the sim machine: `. env.sh && sim_load_course && gz sim -g` next to a
running server (`ssh -Y` to Windows does not provide the OpenGL it needs).

### Driving the boat yourself

With `--no-stack`, arm with `./arm_guided.sh`, then either type MAVProxy commands in the `sitl`
window (`mode GUIDED`, `arm throttle`) or publish over MAVROS like the planner does:

- `/mavros/setpoint_raw/local` (`mavros_msgs/PositionTarget`, `coordinate_frame: 1`,
  `type_mask: 3527`, velocity in ENU): the boat steers toward the velocity direction at that speed.
- `/mavros/setpoint_velocity/cmd_vel_unstamped` (`Twist`): only the speed and `angular.z` are used;
  the boat cannot turn toward a goal this way.

---

## 3. Courses, starts, weather

`tools/generate_course.py` turns a course YAML into a Gazebo world plus an **answer key**
(`course/generated/<course>_seed_N/`): every element's position (world and lat/lon), gates, slips,
the correct answers, the starts and the tasking schedule. `select_course.sh` (called by
`run_competition.sh`) picks it; every other script reads that choice.

### 2027 (`course/roboboat2027.yaml`)

Built from the **RoboBoat 2027 Primer & Task Ideas**, the only 2027 document so far: positions read
off its course sketch, sizes reused from the 2026 handbook where elements carry over, and every
guess tagged `PLACEHOLDER` in the YAML. Update those when the 2027 handbook comes out. Core tiers:

| Task | In the sim |
|---|---|
| 1 Safe Passage | Light buoys along a passage from the ENTRY to the EXIT buoy (both lit blue): green on its port side, red on starboard, unlit black buoys as debris. Other seeds generate a new passage. |
| 2 Emergency Response Sprint | Polyform buoys in six colours and numbered beacons 1-3, one lit green (START). |
| 3 Navigate the Marina | One dock, three slips with green/red indicators, number banners, a vessel in a red slip. |
| 4 Delivery Run | Five vessels with cross / circle / square / triangle signs; RoboCommand assigns one. |
| 5 Dynamic Response | A rally point RoboCommand sends during the run. |
| Proof of Readiness | `--course readiness`: two red/green gates (6-10 ft wide, 25-100 ft apart). |

Not modelled until the handbook specifies them: colour changes (Task 1 Advanced/Disruptive),
acoustic pingers (Task 2), the fire and water delivery (Task 3), multiple deliveries and two-boat
tiers (Tasks 4, 5).

### 2026 (`course/roboboat2026.yaml`)

From the 2026 handbook diagrams (`course/reference/`): Task 1 gates, Task 2 channel and debris
field, Task 3 speed challenge, Task 5 two-dock marina. Kept so the planner's current behaviour can
still be checked against the course it was written for.

### Where the boat starts

`--task N` puts the boat 3 m (`task_start_standoff`) in front of that task's entrance, facing in,
so each task can be tested on its own. A `--task` without its own start (e.g. the planner's
sequence mode `123`), or no `--task`, uses the course start: the sketched start on 2027, before
Task 1 on 2026. On 2027, `--task 4` and `--task 5` start at the course start and get their
RoboCommand message after 5 s.

### Weather and light

- `--waves flat|calm_lake|breezy_lake|windy_lake`: wind and waves together (1.5 / 4 / 7 m/s with
  gusts). The wind pushes the hull, so expect drift during docking. `--wind M/S` and
  `--wind-from DEG` (compass, where it blows from) override it. Seeds other than 0 get a random
  wind bearing.
- `--light morning|midday|afternoon|low_sun|overcast`, or `--time HH:MM`: the sun for that time on
  the competition date. `low_sun` puts glare in the cameras; `--shadows` adds shadows (costs GPU).

### Course models

`tools/make_course_models.py` writes the element models (rulebook sizes, origin at the waterline)
into `gz-waves-models/models`; run it again after changing a dimension. Polyform and indicator
buoys use the mechanical team's CAD (`course/cad/*.step`, converted by `tools/convert_cad.py`).

---

## 4. Judge and RoboCommand

The `judge` window (`6_judge.sh`, package `sim_judge`) is the RoboCommand server: it listens where
`message_node` sends reports (10.10.10.1:50000, as at competition), reads the boat's true pose from
Gazebo and shows a live scoreboard.

- **2027:** scores the Core tier of each task. The primer has no scoring tables, so the points are
  placeholders that only make runs comparable. Task 1: circle ENTRY clockwise, pass green to port
  and red to starboard, circle EXIT counter-clockwise, report ENTRY/EXIT and the buoys. Task 2:
  start at the green beacon, loop the target, leave the field (timed). Task 3: dock in a green slip.
  Task 4: hold next to the assigned vessel after the DELIVERY message. Task 5: reach the rally point
  after the RALLY message. Readiness: both gates, no strikes.
- **2026:** the handbook scoring for Tasks 1, 2, 3 and 5.
- Both: a strike for every buoy/vessel contact; leaving the course boundary ends the run;
  heartbeat and report checks.
- `./stop_competition.sh` writes `runs/<time>_seed<N>.md` (readable summary) and `.json` (every
  event and report). Each report lists exactly how the judge interpreted the rules.

`ros2 run sim_judge send_test_reports --answer-key "$SIM_ANSWER_KEY"` (after `. env.sh &&
sim_load_course`) sends a correct set of reports, to check the judge on its own.

### RoboCommand tasking (2027)

In 2027 RoboCommand also sends messages *to* the boat. The real format is not published, so the
judge sends a stand-in: JSON on `/sim/robocommand/tasking` (`std_msgs/String`), each message
repeated every 2 s with the same `seq`.

```json
{"type": "SPRINT_TARGET", "task": 2, "target_color": "blue", "seq": 1, "issued_sim_t": 0.4}
{"type": "DELIVERY", "task": 4, "targets": [{"shape": "circle"}], "seq": 2, "issued_sim_t": 120.0}
{"type": "RALLY", "task": 5, "lat": 27.37398, "lon": -82.45285, "seq": 3, "issued_sim_t": 240.0}
```

On a full run DELIVERY comes at 120 s and RALLY at 240 s (sim time); `6_judge.sh --no-tasking`
sends none. Nothing in the autonomy stack listens to this yet.

---

## 5. CV training data (autolabel)

The sim knows what every object is and where it is, so it labels camera frames automatically.

```bash
./run_competition.sh --seed 4 --task 2 --labels    # records while the run goes (autolabel window)
ros2 run sim_autolabel export datasets/ --out ~/datasets/sim_main --preview 30
export ROBOFLOW_API_KEY=...                        # Roboflow: Settings > API Keys
ros2 run sim_autolabel upload ~/datasets/sim_main --workspace WS --project PROJECT --limit 20   # trial
ros2 run sim_autolabel upload ~/datasets/sim_main --workspace WS --project PROJECT
```

- Recordings go to `sim/datasets/<time>_seed<N>/` with the boxes in sim label names. `export`
  maps them to a CV model's classes with `src/sim_autolabel/sim_autolabel/classes.yaml` (`main` or
  `numbers`) and writes Roboflow's YOLO layout with a train/valid split. Labels the dataset does not
  list are dropped; after changing the mapping, just export again.
- Boxes cover only what the camera sees. Export also drops tiny boxes (`--min-box-px`, 3 px) and
  boxes spanning the whole frame width (a known water-rendering artefact, ~1 % of frames).
- `--preview N` draws the boxes on N images: look at a few before uploading.
- The new 2027 objects (light buoys, blue/white Polyforms, circle/square signs) are recorded but
  not in the main model's classes yet; the CV team adds them to `classes.yaml`.
- Record several seeds, weathers and times of day. Sim images do not look like the real cameras:
  use them alongside the real dataset, not instead of it. Recording slows the sim down.

---

## 6. How the sim matches the boat

- **LiDAR:** Airy 96, 900 x 96 points, 10 Hz, on `/rslidar_points` (frame `rslidar`), IMU on
  `/rslidar_imu_data`, minimum range 1.2 m. The LiDAR pipeline is the real one
  (`lidar_pipeline_sim.launch.py`, same tuning as `buoy_pipeline.launch.py`; keep them in step).
- **Cameras:** `/camera0/image_raw` (70 deg left), `/camera1` (centre), `/camera2` (70 deg right),
  as in the CV config. The real cameras are 1920x1200@30; the sim defaults to 960x600@15 so three
  of them render, and `7_stack.sh` gives CV the sim resolution.
- **CV:** TensorRT engines only work on the GPU they were built for, so `build_engines.sh` builds
  the sim's own from the ONNX models into the engine cache; the boat's engines are untouched.
- **Planner and reports:** `7_stack.sh` runs global_frame, LiDAR, CV, fusion, the global planner
  and `message_node` unmodified, then sets GUIDED and arms. `message_node` gets protobuf 6 on its
  own path (the system one is too old for `report_pb2.py`).
- **ArduPilot:** SITL loads the boat's real tuning from `src/task_sequence_coordinator/mav.parm`
  (`TEAM_PARAMS=none` for stock values). The hull is tuned to the same numbers: 1.18 m/s at 51 %
  throttle, ~190 deg/s pivot.
- **Still placeholders:** LiDAR and camera mounting poses, dock freeboard, venue datum, measured
  boat speed and turning.

---

## 7. Performance

Gazebo and SITL run in lock-step, so the sim runs slower than real time; the judge scores in sim
time, but the autonomy nodes run on wall-clock time, so to them the boat looks slower.

| Machine and run | Real-time factor |
|---|---|
| Sim Jetson (AGX Orin, MAXN), `--no-stack` | ~0.37 |
| Sim Jetson, full stack with CV | ~0.3 (also with `--labels`) |
| Sim Jetson, `--full-res` | ~0.18, and only ~1.3 camera frames per sim second: too few for CV |
| RTX 5060 Ti desktop, full stack | ~0.45 |

Use the default cameras on the Jetson; test CV detection range with `--full-res` on a desktop GPU.
`--no-cv` frees the most.

---

## 8. The shared sim Jetson (admin)

- **Accounts share one install.** ArduPilot, its venv, the Gazebo plugin, the TensorRT engines and
  the repo's git history live once in `/opt/aqua` (group `aqua`); `config.env` uses it whenever it
  exists. A per-user install would be ~6 GB each. `setup_shared.sh --check` shows the state;
  `setup_shared.sh --link <clone>` moves an existing clone onto the shared history (keeps branches
  and uncommitted work). **Never delete `/opt/aqua/autonomy-workspace-26-27.git`**: the clones read
  their history from it.
- **Users don't collide.** Each account gets its own Gazebo partition, ROS domain and Foxglove
  port. One sim per account at a time.
- **Disk (54 GB) is the limit.** Watch `df -h /`. Things that grow: `~/.vscode-server` (clear it
  every few months; VS Code reinstalls it), `runs/`, `datasets/`, `~/.ros/log`.
- **Never `apt purge` NVIDIA tools.** Removing Nsight also removes the JetPack meta-packages, and a
  later `apt autoremove` then deletes CUDA and TensorRT. Nsight and the TensorRT static libraries
  were removed by deleting the files; `/etc/dpkg/dpkg.cfg.d/01-aqua-slim` keeps updates from
  bringing them back. Always run `apt-get -s autoremove` (simulate) first.

---

## 9. Troubleshooting

Start with `./setup_sim.sh --check` and the window's log in `runs/logs/<time>/`.

| Symptom | Fix |
|---|---|
| Can't see the sim's topics with `ros2 topic list` | Run `. env.sh` in that terminal (each user has their own ROS domain) |
| Judge shows no reports / heartbeats | `ip addr show lo` must list `10.10.10.1`; else `sudo ip addr add 10.10.10.1/32 dev lo` (once per boot) |
| Judge: "no odometry yet" | Gazebo isn't running the course: check the `gazebo` window / log |
| "Not armed yet" keeps repeating | SITL is waiting for a GPS fix (normal for ~30 s). Still stuck: in the `sitl` window `mode GUIDED`, `arm throttle`; for testing `param set ARMING_REQUIRE 0` |
| Boat sinks or doesn't move in Gazebo | The wave plugin or ArduPilot plugin is missing: `./setup_sim.sh --check`, then `./setup_sim.sh --rebuild` |
| Gazebo world empty / "Unable to find uri" | Start it through the scripts (they set the model paths), not with a bare `gz sim` |
| Gazebo window black, or "could not connect to display" | Over SSH use `--headless` and Foxglove |
| `Failed to load plugin [gz-waves1-rendering-ogre2]` | Harmless (fancier wave shading only). Do **not** add an OGRE path to `LD_LIBRARY_PATH`: that crashes Gazebo |
| "numpy.dtype size changed" in a ROS node | A Python venv is active in that shell; `env.sh` removes it, so start things through the scripts |
| MAVProxy crashes on NumPy 2 | Don't set `SITL_GUI=1` (the console/map windows need matplotlib) |
| No CV / "No TensorRT engines for this GPU" | `./build_engines.sh` (takes ~10 min on the Jetson) |
| Engine build: "Error when reading engine file" | Usually a full disk: `df -h /` |
| Permission denied under `/opt/aqua` | You're not in group `aqua` yet: log out and back in (or `newgrp aqua`) |
| Sim very slow | See section 7; close other GPU programs, try `--no-cv` |
| `run_competition.sh` says a previous run is open | It stops it for you; or run `./stop_competition.sh` |

---

## 10. Where things are

| Path | What |
|---|---|
| `src/asv_wave_sim/sim/` | All the run scripts, `config.env`, `runs/` (reports, logs), `datasets/` |
| `src/asv_wave_sim/course/` | Course YAMLs (2027, 2026), reference drawings, CAD; `generated/` = selected worlds |
| `src/asv_wave_sim/tools/` | Course generator (`generate_course.py`, `course2027.py`), model generator, CAD converter |
| `src/asv_wave_sim/gz-waves-models/` | Boat model (`models/ourboat`), course element models, committed seed-0 worlds |
| `src/sim_judge/` | Judge: referees for 2026 and 2027, RoboCommand server, Foxglove course viewer |
| `src/sim_autolabel/` | Autolabel recorder, export, Roboflow upload, `classes.yaml` |
| `src/asv_wave_sim/gz-waves/` | The upstream wave and hydrodynamics plugins (`src/asv_wave_sim/README.md`) |

Ports: RoboCommand 50000 (judge), SITL MAVLink TCP 5760 (MAVProxy) and 5762 (MAVROS),
SITL <-> Gazebo UDP 9002, Foxglove 8700 + user id % 100.

**Old launchers:** `src/asv_wave_sim/run_full_simulation_tmux.bash`, `run_gazebo_standalone.sh`
and `test_*.sh` are from before this setup (they expect `~/Repos/School/...` checkouts and the old
world). Don't use them; `run_competition.sh` and the numbered scripts replace them.
