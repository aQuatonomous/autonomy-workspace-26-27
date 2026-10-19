# MAVROS — Quick Reference

## Run MAVROS

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:57600
```

Other port: `fcu_url:=/dev/ttyUSB0:57600`. SITL: `fcu_url:=tcp://localhost:5760`.

---

## Set FC mode (service calls)

List mode service:

```bash
ros2 service list | grep set_mode
```

Set to GUIDED:

```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'GUIDED'}"
```

Set to MANUAL:

```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'MANUAL'}"
```

With UAS namespace (e.g. `/uas1/mavros`): use the path from `ros2 service list` (e.g. `/uas1/mavros/set_mode`).

---

## Echo useful topics

```bash
ros2 topic echo /mavros/global_position/global
ros2 topic echo /mavros/global_position/compass_hdg
ros2 topic echo /mavros/state
ros2 topic list | grep mavros
```

Clear mission / set MANUAL from repo: `./fcu_clear_mission.sh --manual` (or `MAVROS_NS=uas1 ./fcu_clear_mission.sh --manual`).

---

## Troubleshooting

**`ros2 topic list` (or echo) fails with `RuntimeError: !rclpy.ok()`**  
Use a **new terminal** and only `source` ROS + workspace (don’t run a node that exits with `rclpy.shutdown()` in that shell). Or run `ros2 daemon stop` then `ros2 daemon start` and try again.
