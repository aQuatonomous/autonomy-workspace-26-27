#!/usr/bin/env python3
"""
Global planner ROS2 node.

Subscribes to:
  - /global_detections (global_frame/GlobalDetectionArray) from detection_to_global
  - /mavros/global_position/global (sensor_msgs/NavSatFix) for position (lat/lon -> local x,y)
  - /mavros/global_position/compass_hdg (std_msgs/Float64) for heading (deg -> rad)
  - /mavros/global_position/gp_vel (geometry_msgs/Twist) for velocity
  - /sound_signal_interupt (std_msgs/Int32): 1 = stop boat, 2 = ignore

Maintains a persistent EntityList, runs TaskMaster (Task1/2/3/4) at 10 Hz,
publishes /planned_path (nav_msgs/Path), /curr_task (std_msgs/Int32), and
/mavros/setpoint_velocity/cmd_vel_unstamped (geometry_msgs/Twist) for MAVROS.

All planning library code (entities, Task1/2/3, TaskMaster, potential_fields_planner)
remains ROS-free; this node is the only place with rclpy.
"""

from __future__ import annotations

import os
import subprocess
import sys
from typing import Optional

# Ensure planning library is on path (sibling of global_planner package in workspace)
def _add_planning_path():
    planning_path = os.environ.get("PLANNING_PATH")
    if not planning_path or not os.path.isdir(planning_path):
        # Try relative to this file: .../install/global_planner/lib/python3.x/site-packages/global_planner
        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory("global_planner")
            # Try standard layout: workspace/src/planning
            planning_path = os.path.abspath(os.path.join(share, "..", "..", "..", "src", "planning"))
            if not os.path.isdir(planning_path):
                # Fallback: planning at workspace root (e.g. autonomy-ws-25-26/planning)
                planning_path = os.path.abspath(os.path.join(share, "..", "..", "..", "..", "planning"))
        except Exception:
            planning_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "planning"))
    if os.path.isdir(planning_path) and planning_path not in sys.path:
        sys.path.insert(0, planning_path)


_add_planning_path()

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String as StringMsg, Int32, Float64
from global_frame.msg import GlobalDetectionArray
import math
import traceback

# MAVROS publishes sensor data with BEST_EFFORT QoS; subscribers must match
_QOS_SENSOR = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

try:
    from mavros_msgs.msg import State as MavrosState
    from mavros_msgs.srv import SetMavFrame
    _MAVROS_STATE_AVAILABLE = True
except ImportError:
    _MAVROS_STATE_AVAILABLE = False
    MavrosState = None
    SetMavFrame = None

# Planning library (no ROS)
from Global.entities import EntityList, apply_tracked_buoys
from TaskMaster import TaskMaster

from .watchdogs import tick as watchdog_tick, initial_state as watchdog_initial_state


def _log_prefix(clock, pose) -> str:
    """Prefix for all boat logs: timestamp and boat position (x, y) in m."""
    t = clock.now()
    sec = t.nanoseconds * 1e-9
    x, y = float(pose[0]), float(pose[1])
    return f"[t={sec:.1f}s] x={x:.1f} y={y:.1f} (m) | "


class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__("global_planner_node")

        self.declare_parameter("task_id", 1)
        self.declare_parameter("planning_hz", 10.0)
        self.declare_parameter("cmd_vel_topic", "/mavros/setpoint_velocity/cmd_vel_unstamped")
        self.declare_parameter("heading_topic", "/mavros/global_position/compass_hdg")
        self.declare_parameter("gp_vel_topic", "/mavros/global_position/gp_vel")
        self.declare_parameter("sound_signal_topic", "/sound_signal_interupt")
        self.declare_parameter("pump_hold_script", "")  # Full path to pump_hold.py for Task4; empty = disabled
        # Map bounds: (min_x, min_y, max_x, max_y) in meters. Empty list = use dynamic from gates/entities
        self.declare_parameter("map_bounds", [])

        task_id = int(self.get_parameter("task_id").value)
        self._task_id = task_id
        self._planning_hz = float(self.get_parameter("planning_hz").value)
        self._cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self._heading_topic = self.get_parameter("heading_topic").value
        self._gp_vel_topic = self.get_parameter("gp_vel_topic").value
        self._sound_signal_topic = self.get_parameter("sound_signal_topic").value
        self._pump_hold_script = (self.get_parameter("pump_hold_script").value or "").strip()
        self._pump_hold_proc: Optional[subprocess.Popen] = None
        
        # Parse static map bounds if provided
        map_bounds_param = self.get_parameter("map_bounds").value
        if map_bounds_param and len(map_bounds_param) == 4:
            self._static_map_bounds = tuple(float(v) for v in map_bounds_param)
            self.get_logger().info(f"Static map bounds: {self._static_map_bounds}")
        else:
            self._static_map_bounds = None
            self.get_logger().info("Using dynamic map bounds from gates/entities")

        self._entity_list = EntityList(start_position=(0.0, 0.0))
        self._latest_pose = (0.0, 0.0, 0.0)  # x, y, heading (m, m, rad) in map frame
        self._latest_velocity_m_s = (0.0, 0.0)  # world frame (ENU), m/s
        self._latest_heading_rad: Optional[float] = None  # from compass
        self._origin_lat_lon: Optional[tuple] = None  # (lat0, lon0) for lat/lon -> x,y
        self._buoy_count = 0
        self._start_set = False
        self._heading_set = False
        self._sound_stop = False  # True when /sound_signal_interupt == 1

        # Guided-mode gate: only run planning when Pixhawk is in GUIDED
        self._guided_mode_active = not _MAVROS_STATE_AVAILABLE  # allow planning if no mavros_msgs
        if _MAVROS_STATE_AVAILABLE:
            self._mavros_state_sub = self.create_subscription(
                MavrosState,
                "/mavros/state",
                self._mavros_state_callback,
                _QOS_SENSOR,
            )
        else:
            self.get_logger().warn("mavros_msgs not available; guided-mode gate disabled, planning allowed")

        # Global reference for lat/long (Task 2 debris report). Set when origin is set.
        self._global_ref = None  # (lat_ref, lon_ref, x_ref_m, y_ref_m)
        # Task 2 debris report throttle (avoid publishing every tick)
        self._last_debris_report_time: Optional[float] = None
        self._debris_report_interval_sec = 2.0
        # Task 3 indicator color report throttle (same topic /gs_message_send)
        self._last_indicator_report_time: Optional[float] = None
        self._indicator_report_interval_sec = 2.0
        self._last_supply_report_time: Optional[float] = None
        self._supply_report_interval_sec = 2.0

        self._task_master = TaskMaster(
            entities=self._entity_list,
            start=(0.0, 0.0),
            task_id=task_id,
            map_bounds=self._static_map_bounds,  # Use static bounds if provided, else None (dynamic)
        )

        self._global_detections_sub = self.create_subscription(
            GlobalDetectionArray,
            "/global_detections",
            self._global_detections_callback,
            10,
        )
        self._global_position_sub = self.create_subscription(
            NavSatFix,
            "/mavros/global_position/global",
            self._global_position_callback,
            _QOS_SENSOR,
        )
        self._heading_sub = self.create_subscription(
            Float64,
            self._heading_topic,
            self._heading_callback,
            _QOS_SENSOR,
        )
        self._gp_vel_sub = self.create_subscription(
            TwistStamped,
            self._gp_vel_topic,
            self._gp_vel_callback,
            _QOS_SENSOR,
        )
        self._sound_sub = self.create_subscription(
            Int32,
            self._sound_signal_topic,
            self._sound_signal_callback,
            10,
        )

        self._path_pub = self.create_publisher(Path, "/planned_path", 10)
        self._cmd_vel_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._curr_task_pub = self.create_publisher(Int32, "/curr_task", 10)
        self._gs_message_pub = self.create_publisher(StringMsg, "/gs_message_send", 10)
        # Task 1: gate_pass for entry/exit reporting (message_node sends to competition server)
        self._gate_pass_pub = self.create_publisher(StringMsg, "/messages/gate_pass", 10)
        self._task1_entry_reported = False
        self._task1_exit_reported = False

        if SetMavFrame is not None:
            self._set_mav_frame_client = self.create_client(
                SetMavFrame, "/mavros/setpoint_velocity/mav_frame"
            )
            self._set_mav_frame_timer = self.create_timer(2.0, self._set_mav_frame_once)
        else:
            self.get_logger().warn("SetMavFrame service not available; cannot set LOCAL_NED")

        self._watchdog_state = watchdog_initial_state()
        self._timer = self.create_timer(1.0 / self._planning_hz, self._planning_tick)

        self.get_logger().info(
            f"global_planner_node started: task_id={task_id}, hz={self._planning_hz}, "
            f"cmd_vel={self._cmd_vel_topic}"
        )

    def _set_mav_frame_once(self) -> None:
        """Set MAVROS setpoint_velocity frame to LOCAL_NED (frame=1) once the service is available."""
        if not self._set_mav_frame_client.service_is_ready():
            self.get_logger().info("Waiting for /mavros/setpoint_velocity/mav_frame service...")
            return
        req = SetMavFrame.Request()
        req.mav_frame = 1  # MAV_FRAME_LOCAL_NED
        future = self._set_mav_frame_client.call_async(req)
        future.add_done_callback(self._set_mav_frame_done)
        self._set_mav_frame_timer.cancel()

    def _set_mav_frame_done(self, future) -> None:
        try:
            result = future.result()
            if result.success:
                self.get_logger().info("MAVROS setpoint_velocity frame set to LOCAL_NED (1)")
            else:
                self.get_logger().error("Failed to set MAVROS frame to LOCAL_NED")
        except Exception as e:
            self.get_logger().error(f"SetMavFrame service call failed: {e}")

    def _mavros_state_callback(self, msg) -> None:
        self._guided_mode_active = (getattr(msg, "mode", "") == "GUIDED")

    def _global_position_callback(self, msg: NavSatFix) -> None:
        if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude):
            return
        lat = float(msg.latitude)
        lon = float(msg.longitude)
        if self._origin_lat_lon is None:
            self._origin_lat_lon = (lat, lon)
            self._global_ref = (lat, lon, 0.0, 0.0)
            hdg = self._latest_heading_rad if self._latest_heading_rad is not None else 0.0
            self._latest_pose = (0.0, 0.0, hdg)
            self._entity_list.set_start_position((0.0, 0.0))
            self._start_set = True
            self.get_logger().info(
                f"Start set from GPS: lat={lat:.6f} lon={lon:.6f} (x=0, y=0 m)"
            )
            return
        lat0, lon0 = self._origin_lat_lon
        m_per_deg_lat = 111320.0
        m_per_deg_lon = 111320.0 * math.cos(math.radians(lat0))
        x_m = (lon - lon0) * m_per_deg_lon   # East
        y_m = (lat - lat0) * m_per_deg_lat   # North
        hdg = self._latest_heading_rad if self._latest_heading_rad is not None else 0.0
        self._latest_pose = (x_m, y_m, hdg)

    def _heading_callback(self, msg: Float64) -> None:
        hdg_deg = float(msg.data)
        # Match boat_state_node: 0 = East, π/2 = North (compass 0° North, 90° East → 90 - hdg_deg)
        self._latest_heading_rad = math.radians(90.0 - hdg_deg)
        x, y = self._latest_pose[0], self._latest_pose[1]
        self._latest_pose = (x, y, self._latest_heading_rad)
        if not self._heading_set:
            self._heading_set = True
            self.get_logger().info(f"Heading received: {hdg_deg:.1f} deg -> {self._latest_heading_rad:.3f} rad")

    def _gp_vel_callback(self, msg: TwistStamped) -> None:
        # MAVROS gp_vel is ENU: linear.x = East, linear.y = North, linear.z = Up
        t = msg.twist
        self._latest_velocity_m_s = (float(t.linear.x), float(t.linear.y))

    def _sound_signal_callback(self, msg: Int32) -> None:
        if msg.data == 1:
            self._sound_stop = True
        # 2 = ignore for now

    def _global_detections_callback(self, msg: GlobalDetectionArray) -> None:
        n = len(msg.detections)
        pose = self._latest_pose
        prefix = _log_prefix(self.get_clock(), pose)
        buoys_m = []
        for d in msg.detections:
            item = {"id": d.id, "x": float(d.east), "y": float(d.north)}
            if getattr(d, "class_id", 255) != 255:
                item["class_id"] = d.class_id
            else:
                item["color"] = getattr(d, "class_name", "") or "unknown"
            buoys_m.append(item)
        apply_tracked_buoys(self._entity_list, buoys_m, tolerance=1.0)
        self._buoy_count = n
        gates = self._entity_list.get_gates()
        obstacles = self._entity_list.get_obstacles()
        types_seen = set()
        for d in msg.detections:
            if getattr(d, "class_id", 255) != 255:
                types_seen.add(f"class_{d.class_id}")
            else:
                types_seen.add(getattr(d, "class_name", "unknown"))
        self.get_logger().info(
            f"{prefix}Detection update: received {n} detections, types={types_seen}, "
            f"gates={len(gates)}, obstacles={len(obstacles)}"
        )
        self.get_logger().info(
            f"{prefix}Seen red-green buoy pairs: {len(gates)} pair(s) "
            f"{[(f'red@({r[0]:.1f},{r[1]:.1f}) green@({g[0]:.1f},{g[1]:.1f})') for r, g in gates]}"
        )

    def _get_pose(self):
        return self._latest_pose

    def _get_detections(self):
        return self._entity_list.to_detections()

    def _odom_to_latlon(self, x_m: float, y_m: float):
        """Convert odom position (m) to (lat, lon) using stored global reference. Returns (lat, lon) or None."""
        if self._global_ref is None:
            return None
        lat_ref, lon_ref, x_ref_m, y_ref_m = self._global_ref
        # ENU: x East, y North. meters to degrees (approx)
        m_per_deg_lat = 111320.0
        m_per_deg_lon = 111320.0 * math.cos(math.radians(lat_ref))
        lat = lat_ref + (y_m - y_ref_m) / m_per_deg_lat
        lon = lon_ref + (x_m - x_ref_m) / m_per_deg_lon
        return (lat, lon)

    def _planning_tick(self) -> None:
        self._entity_list.prune_stale(max_age_sec=20.0)

        # Publish current task (int32)
        self._curr_task_pub.publish(Int32(data=self._task_id))

        pose = self._get_pose()
        prefix = _log_prefix(self.get_clock(), pose)

        # Sound interrupt: 1 = stop boat
        if self._sound_stop:
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self._cmd_vel_pub.publish(twist)
            return

        if not self._guided_mode_active:
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self._cmd_vel_pub.publish(twist)
            return

        # (4) Skip planning until we have received at least one GPS fix.
        if not self._start_set:
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self._cmd_vel_pub.publish(twist)
            return

        try:
            # Map bounds: prefer static (from parameter) over dynamic (from gates/entities)
            if self._static_map_bounds is not None:
                map_bounds = self._static_map_bounds
            else:
                map_bounds = getattr(self._entity_list, "get_map_bounds", lambda: None)()
            
            result = self._task_master.run_one_shot(
                get_pose=self._get_pose,
                get_detections=self._get_detections,
                use_planning=True,
                map_bounds=map_bounds,
            )

            path = result.get("path") or []
            velocities = getattr(self._task_master.manager, "current_velocities", None)
            speeds = getattr(self._task_master.manager, "current_speeds", None)
            goals = self._entity_list.get_goals()
            hdg_for_gates = self._latest_heading_rad
            boat_xy = (self._latest_pose[0], self._latest_pose[1])
            gates = self._entity_list.get_gates(boat_heading_rad=hdg_for_gates, boat_pos=boat_xy)
            no_go_pts = self._entity_list.get_no_go_obstacle_points(
                map_bounds=map_bounds, boat_heading_rad=hdg_for_gates,
            )

            # Current task
            task_id = self._task_master.task_id
            self.get_logger().info(f"{prefix}Current task: task_id={task_id}")

            # Task 1 (Evacuation): report entry gates completed then exit gates completed via /messages/gate_pass
            if task_id == 1:
                mgr = self._task_master.manager
                if getattr(mgr, "entry_gates_completed", lambda: False)() and not self._task1_entry_reported:
                    self._gate_pass_pub.publish(StringMsg(data="start"))
                    self._task1_entry_reported = True
                    self.get_logger().info(f"{prefix}Task1: entry gates completed -> sent gate_pass 'start'")
                if getattr(mgr, "exit_gates_completed", lambda: False)() and not self._task1_exit_reported:
                    self._gate_pass_pub.publish(StringMsg(data="end"))
                    self._task1_exit_reported = True
                    self.get_logger().info(f"{prefix}Task1: exit gates completed -> sent gate_pass 'end' (heartbeat/time+position via message_node)")

            if task_id == 1:
                phase = result.get("phase", "?")
                self.get_logger().info(f"{prefix}Task1 phase={phase}")
            if task_id == 3:
                phase = result.get("phase", "?")
                self.get_logger().info(f"{prefix}Task3 phase={phase}")
            if task_id == 4:
                phase = result.get("phase", "?")
                pump_on = getattr(self._task_master.manager, "pump_on", False)
                self.get_logger().info(f"{prefix}Task4 phase={phase} pump_on={pump_on}")

            # Seen red-green buoy pairs
            self.get_logger().info(
                f"{prefix}Seen red-green buoy pairs: {len(gates)} "
                f"{[(f'({r[0]:.1f},{r[1]:.1f})-({g[0]:.1f},{g[1]:.1f})') for r, g in gates]}"
            )

            # Goal / next goal
            if goals:
                next_goal = goals[0]
                self.get_logger().info(
                    f"{prefix}Seen goal / next goal: count={len(goals)}, next=({next_goal[0]:.1f}, {next_goal[1]:.1f}) m"
                )
            else:
                self.get_logger().info(f"{prefix}No goal seen")

            # No-go zone
            if no_go_pts:
                self.get_logger().info(f"{prefix}No-go zone identified: {len(no_go_pts)} boundary points")
            else:
                self.get_logger().info(f"{prefix}No-go zone: none (no gate pairs or using cached)")

            # State: following goal vs going straight / prediction
            has_vel = velocities is not None and len(velocities) > 0 and speeds is not None and len(speeds) > 0
            if goals and has_vel:
                self.get_logger().info(f"{prefix}State: following goal")
            elif not goals:
                self.get_logger().info(f"{prefix}State: no goal (idle or going straight)")
            else:
                self.get_logger().info(f"{prefix}State: goal set but no velocity (waiting or stuck)")

            # Current entity list (summary by type and id)
            by_type = {}
            for e in self._entity_list.entities:
                if e.type == "goal":
                    continue
                key = e.type
                if key not in by_type:
                    by_type[key] = []
                eid = e.entity_id if e.entity_id is not None else "?"
                by_type[key].append(f"id={eid}@({e.position[0]:.1f},{e.position[1]:.1f})")
            self.get_logger().info(f"{prefix}Current entity list: {by_type}")

            # Task 2: report lat/long of black buoys (debris) and green/red indicators to /gs_message_send (throttled)
            if task_id == 2 and self._global_ref is not None:
                time_now_sec = self.get_clock().now().nanoseconds * 1e-9
                if self._last_debris_report_time is None or (time_now_sec - self._last_debris_report_time) >= self._debris_report_interval_sec:
                    # Black buoys (debris) - same format as before
                    black_buoys = self._entity_list.get_black_buoys()
                    if black_buoys:
                        parts = []
                        for eid, pos in black_buoys:
                            ll = self._odom_to_latlon(float(pos[0]), float(pos[1]))
                            if ll is not None:
                                lat, lon = ll
                                parts.append(f"{eid},{lat:.6f},{lon:.6f}")
                        if parts:
                            self._gs_message_pub.publish(StringMsg(data="DEBRIS_LATLON|" + "|".join(parts)))
                    # Green and red indicators - same style (id,lat,lon) stored/sent like black buoys
                    green_ents = self._entity_list.get_by_type("green_indicator")
                    red_ents = self._entity_list.get_by_type("red_indicator")
                    ind_parts = []
                    for e in green_ents:
                        ll = self._odom_to_latlon(float(e.position[0]), float(e.position[1]))
                        if ll is not None:
                            lat, lon = ll
                            eid = e.entity_id if e.entity_id is not None else 0
                            ind_parts.append(f"green,{eid},{lat:.6f},{lon:.6f}")
                    for e in red_ents:
                        ll = self._odom_to_latlon(float(e.position[0]), float(e.position[1]))
                        if ll is not None:
                            lat, lon = ll
                            eid = e.entity_id if e.entity_id is not None else 0
                            ind_parts.append(f"red,{eid},{lat:.6f},{lon:.6f}")
                    if ind_parts:
                        self._gs_message_pub.publish(StringMsg(data="INDICATOR_LATLON|" + "|".join(ind_parts)))
                    self._last_debris_report_time = time_now_sec

            # Task 3: report indicator color to /gs_message_send (same topic as Task 2, throttled)
            if task_id == 3:
                indicator_color = getattr(self._task_master.manager, "indicator_color", None)
                if indicator_color is not None:
                    time_now_sec = self.get_clock().now().nanoseconds * 1e-9
                    if self._last_indicator_report_time is None or (time_now_sec - self._last_indicator_report_time) >= self._indicator_report_interval_sec:
                        msg_str = f"INDICATOR_COLOR|{indicator_color}"
                        self._gs_message_pub.publish(StringMsg(data=msg_str))
                        self._last_indicator_report_time = time_now_sec

            # Task 4: pump via pump_hold.py subprocess and serviced vessel reporting
            if task_id == 4 and self._pump_hold_script:
                pump_on = getattr(self._task_master.manager, "pump_on", False)
                if pump_on:
                    if self._pump_hold_proc is None or self._pump_hold_proc.poll() is not None:
                        try:
                            self._pump_hold_proc = subprocess.Popen(
                                [sys.executable, self._pump_hold_script],
                                start_new_session=True,
                            )
                            self.get_logger().info("Task4: started pump_hold.py")
                        except Exception as e:
                            self.get_logger().warn(f"Task4: failed to start pump_hold.py: {e}")
                else:
                    if self._pump_hold_proc is not None and self._pump_hold_proc.poll() is None:
                        try:
                            self._pump_hold_proc.terminate()
                            self._pump_hold_proc.wait(timeout=2.0)
                        except (subprocess.TimeoutExpired, ProcessLookupError):
                            try:
                                self._pump_hold_proc.kill()
                            except ProcessLookupError:
                                pass
                        self._pump_hold_proc = None
                        self.get_logger().info("Task4: stopped pump_hold.py")
            if task_id == 4:
                serviced = getattr(self._task_master.manager, "serviced_for_report", [])
                if serviced and self._global_ref is not None:
                    time_now_sec = self.get_clock().now().nanoseconds * 1e-9
                    if self._last_supply_report_time is None or (time_now_sec - self._last_supply_report_time) >= self._supply_report_interval_sec:
                        parts = []
                        for eid, pos in serviced:
                            ll = self._odom_to_latlon(float(pos[0]), float(pos[1]))
                            if ll is not None:
                                lat, lon = ll
                                parts.append(f"yellow,{eid},{lat:.6f},{lon:.6f}")
                        if parts:
                            self._gs_message_pub.publish(StringMsg(data="SUPPLY_WATER|" + "|".join(parts)))
                        self._task_master.manager.serviced_for_report.clear()
                        self._last_supply_report_time = time_now_sec

            # Next path given and velocity given
            if goals:
                next_wp = goals[0]
                self.get_logger().info(
                    f"{prefix}Next path (waypoint) given: ({next_wp[0]:.1f}, {next_wp[1]:.1f}) m"
                )
            else:
                self.get_logger().info(f"{prefix}Next path given: none")

            if has_vel:
                vx = float(velocities[0][0])
                vy = float(velocities[0][1])
                speed = float(speeds[0])
                self.get_logger().info(
                    f"{prefix}Velocity given: vx={vx:.2f} vy={vy:.2f} m/s speed={speed:.2f} m/s"
                )
            else:
                self.get_logger().info(f"{prefix}Velocity given: zero (no path)")

            # Watchdog disabled for testing
            # Normal: use planner output
            if goals:
                path_msg = Path()
                path_msg.header.stamp = self.get_clock().now().to_msg()
                path_msg.header.frame_id = "map"
                next_wp = goals[0]
                ps = PoseStamped()
                ps.header = path_msg.header
                ps.pose.position.x = float(next_wp[0])
                ps.pose.position.y = float(next_wp[1])
                ps.pose.position.z = 0.0
                path_msg.poses.append(ps)
                self._path_pub.publish(path_msg)

            twist = Twist()
            if has_vel:
                # Planner gives world-frame velocity (ENU: x=East, y=North).
                # MAVROS setpoint_velocity expects LOCAL_NED: x=North, y=East, z=Down.
                vx_w = float(velocities[0][0])  # East
                vy_w = float(velocities[0][1])  # North
                twist.linear.x = vy_w   # North (NED x)
                twist.linear.y = vx_w   # East (NED y)
                twist.linear.z = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = 0.0
                self._cmd_vel_pub.publish(twist)
            else:
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = 0.0
                self._cmd_vel_pub.publish(twist)

            if result.get("status") == "SUCCESS":
                self.get_logger().info(f"{prefix}Task completed successfully")
                self._timer.cancel()

        except Exception as e:
            self.get_logger().error(f"Planning tick error: {e}\n{traceback.format_exc()}")
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self._cmd_vel_pub.publish(twist)

    def destroy_node(self) -> None:
        self._timer.cancel()
        if self._pump_hold_proc is not None and self._pump_hold_proc.poll() is None:
            try:
                self._pump_hold_proc.terminate()
                self._pump_hold_proc.wait(timeout=2.0)
            except (subprocess.TimeoutExpired, ProcessLookupError):
                try:
                    self._pump_hold_proc.kill()
                except ProcessLookupError:
                    pass
            self._pump_hold_proc = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
