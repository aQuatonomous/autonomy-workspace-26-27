## Pump control via MAVLink (Task 4 quick guide)

The **only supported pump path** in this repo is through the flight controller, driven by the
`TaskSequenceCoordinator` ROS2 node over MAVLink. There is no official Jetson‑GPIO path anymore.

At a high level:

- `task_sequence_coordinator_node.py` uses MAVROS `CommandInt` service calls to toggle a relay output.
- Task 4 in the state machine turns the pump **ON** near the start of Task 4 and **OFF** ~70 seconds later.

---

### 1. What the coordinator does in Task 4

In `task_sequence_coordinator/task_sequence_coordinator_node.py`:

- When entering Task 4 (`TaskState.TASK4_START`):
  - It publishes `/cur_task = 4` once at the beginning.
- About 2 seconds after Task 4 starts:
  - Switches mode to **GUIDED**.
  - Calls `_start_pumping()`, which sends MAVLink **CommandInt 182** to the flight controller:
    - `command = 182`
    - `param3 = 180.0` → how long to pump for on the FC side (seconds).
  - Immediately switches back to **AUTO**.
  - At the same moment, it:
    - Publishes a hard‑coded yellow vessel detection on `/messages/object_detected`.
    - Publishes `/messages/object_delivered` once.
- Around 70 seconds after entering Task 4:
  - Switches to **GUIDED** again.
  - Calls `_end_pumping()`, which sends MAVLink **CommandInt 181** to stop the relay:
    - `command = 181`
  - Switches back to **AUTO** and transitions to Task 5 (docking).

On the autopilot side, you must have the relevant relay/aux output configured so that
commands 182/181 control the physical pump relay.

---

### 2. How to run the pump (full sequence)

The simplest way to exercise the pump via MAVLink is to run the **task sequence script**, which
brings up MAVROS, the sound pipeline, CV, and the coordinator for Tasks 1→3→4:

```bash
cd ~/Repos/School/autonomy-ws-25-26
source /opt/ros/jazzy/setup.bash
./build_comp_messageing.sh         # or ./build.sh if you prefer
./run_task_sequence.sh
```

What happens:

- `run_task_sequence.sh`:
  - Sources ROS2 and the workspace.
  - Launches CV and the patrol‑boat detector.
  - Launches **MAVROS** (talking to your Pixhawk/ArduPilot via `FCU_URL`, default `/dev/ttyACM0:57600`).
  - Launches the sound pipeline and **message_node**.
  - Launches `task_sequence_coordinator`.
- The coordinator:
  - Runs Task 1 and Task 3.
  - Enters Task 4 and automatically commands the pump ON/OFF via MAVLink as described above.

You can monitor status by watching the coordinator logs and/or these topics:

- `/cur_task` (std_msgs/Int32) — current task number (4 during pump phase).
- `/messages/object_detected` and `/messages/object_delivered` — RoboCommand reports.

If the autopilot and wiring are set up correctly, the pump will:

1. Turn **ON** shortly after Task 4 begins (you’ll see log lines like “Task 4: starting pump for 60 seconds”).
2. Turn **OFF** roughly 70 seconds after entering Task 4, when the coordinator switches to Task 5.

---

### 3. Manual testing via MAVROS (optional)

If you want to test just the MAVLink relay commands without running the full task sequence, you can
call the MAVROS service directly once MAVROS is running:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Repos/School/autonomy-ws-25-26/install/setup.bash

# Turn pump ON via CommandInt 182
ros2 service call /mavros/cmd/command_int mavros_msgs/srv/CommandInt "{
  broadcast: false,
  frame: 0,
  command: 182,
  current: 0,
  autocontinue: 0,
  param1: 1.0,
  param2: 1.0,
  param3: 180.0,
  param4: 0.0,
  x: 0,
  y: 0,
  z: 0.0
}"

# Turn pump OFF via CommandInt 181
ros2 service call /mavros/cmd/command_int mavros_msgs/srv/CommandInt "{
  broadcast: false,
  frame: 0,
  command: 181,
  current: 0,
  autocontinue: 0,
  param1: 1.0,
  param2: 0.0,
  param3: 0.0,
  param4: 0.0,
  x: 0,
  y: 0,
  z: 0.0
}"
```

Again, the exact effect depends on how your flight controller is configured to map these custom
commands (182/181) to a physical relay output wired to the pump.

