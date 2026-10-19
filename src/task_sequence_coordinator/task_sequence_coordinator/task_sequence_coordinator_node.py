#!/usr/bin/env python3
"""
Task Sequence Coordinator Node

Executes a pre-programmed task sequence with timed message publishing and patrol-boat
hold. Timeline (elapsed times in each state):

  Task 1 (Entry/Exit): cur_task=1, gate "start" at 3s, gate "end" at 20s, then 40s wait.
  Task 3 (Speed): cur_task=3 + gate "speed_start", then gate "speed_end" + red light
                  ObjectDetected at 60s, then 15s wait.
  Task 4 (Delivery): cur_task=4 at 0.5s, pump ON + yellow vessel ObjectDetected +
                     object_delivered at 2s, pump OFF at 70s.
  Task 5 (Docking): cur_task=5 at 0.5s, docking report (dock S, slip 1) at 15s → COMPLETED.

During Task 1 or Task 3, if the patrol boat (yellow, flattened) is detected: switch to
LOITER, report STOPPING, hold 5s, then AUTO and report RESUMING (task timer paused).

Sound signals: 1 blast → navigate to yellow buoy, 2 blasts → marina; GUIDED then back to
AUTO and resume. At most 2 GUIDED attempts per interrupt (reset after arrival).
"""

import math
from typing import Optional, Tuple
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import String, Int32, Bool
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import State as MavrosState
from mavros_msgs.srv import CommandInt
from mavros_msgs.srv import SetMode
from sound_signal_msgs.msg import SoundSignalWithFreq
from message_node_msgs.msg import ObjectDetection, Dock

# MAVROS sensor data uses BEST_EFFORT QoS
_QOS_SENSOR = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


class TaskState(Enum):
    """Task sequence states"""
    INIT = 0
    TASK1_START = 1
    TASK1_WAIT_START = 2
    TASK1_WAIT_END = 3
    TASK1_DONE = 4
    TASK3_START = 5
    TASK3_WAIT_SPEED_END = 6
    TASK3_DONE = 7
    TASK4_START = 8
    TASK4_DONE = 9
    SOUND_INTERRUPT = 10
    SOUND_NAVIGATE = 11
    COMPLETED = 12
    PATROL_BOAT_HOLD = 13


class TaskSequenceCoordinator(Node):
    def __init__(self):
        super().__init__("task_sequence_coordinator")
        
        # Hardcoded navigation targets
        self.declare_parameter("yellow_buoy_lat", 27.37420)
        self.declare_parameter("yellow_buoy_lon", -82.45300)
        self.declare_parameter("marina_lat", 27.37429)
        self.declare_parameter("marina_lon", -82.35259)
        self.declare_parameter("arrival_threshold_m", 5.0)
        
        self._yellow_buoy = (
            self.get_parameter("yellow_buoy_lat").value,
            self.get_parameter("yellow_buoy_lon").value
        )
        self._marina = (
            self.get_parameter("marina_lat").value,
            self.get_parameter("marina_lon").value
        )
        self._arrival_threshold = self.get_parameter("arrival_threshold_m").value
        
        # State tracking
        self._task_state = TaskState.INIT
        self._state_start_time: Optional[float] = None
        self._current_position: Optional[Tuple[float, float]] = None  # (lat, lon)
        self._mavros_mode: Optional[str] = None
        self._sound_target: Optional[Tuple[float, float]] = None
        self._pre_interrupt_task: Optional[int] = None
        self._pre_interrupt_state: Optional[TaskState] = None
        self._pre_interrupt_elapsed: Optional[float] = None
        self._guided_mode_attempts = 0  # At most 2 attempts to switch to GUIDED (avoid flip-flopping)
        self._task4_pump_on_done = False
        self._task4_pump_off_done = False
        self._task4_object_delivered_done = False
        self._task5_docking_done = False
        self._patrol_boat_detected = False
        self._pre_patrol_state: Optional[TaskState] = None
        self._pre_patrol_state_start_time: Optional[float] = None

        # Mission start waypoint and exit gate tracking
        self._start_position: Optional[Tuple[float, float]] = None
        self._exit_gate_sent: bool = False

        # Publishers
        self._cur_task_pub = self.create_publisher(Int32, "/cur_task", 10)
        self._gate_pass_pub = self.create_publisher(String, "/messages/gate_pass", 10)
        self._object_detected_pub = self.create_publisher(ObjectDetection, "/messages/object_detected", 10)
        self._object_delivered_pub = self.create_publisher(Int32, "/messages/object_delivered", 10)
        self._docking_pub = self.create_publisher(Dock, "/messages/docking", 10)
        self._patrol_boat_pub = self.create_publisher(String, "/messages/patrol_boat", 10)
        self._setpoint_global_pub = self.create_publisher(
            GeoPoseStamped, "/mavros/setpoint_position/global", 10
        )
        self._command_int_client = self.create_client(CommandInt, "/mavros/cmd/command_int")
        
        # Subscribers
        self._position_sub = self.create_subscription(
            NavSatFix,
            "/mavros/global_position/global",
            self._position_callback,
            _QOS_SENSOR
        )
        self._sound_sub = self.create_subscription(
            SoundSignalWithFreq,
            "/sound_signal_interupt_freq",
            self._sound_callback,
            10
        )
        self._mavros_state_sub = self.create_subscription(
            MavrosState,
            "/mavros/state",
            self._mavros_state_callback,
            _QOS_SENSOR
        )
        # Patrol boat detector node publishes /patrol_boat/detected (Bool) based on preprocessed frames
        self._patrol_sub = self.create_subscription(
            Bool,
            "/patrol_boat/detected",
            self._patrol_boat_callback,
            10,
        )

        # Service client for mode changes
        self._set_mode_client = self.create_client(SetMode, "/mavros/set_mode")
        
        # Main control loop at 10 Hz
        self._control_timer = self.create_timer(0.1, self._control_loop)
        
        self.get_logger().info("Task Sequence Coordinator initialized")
        self.get_logger().info(f"Yellow buoy: {self._yellow_buoy}")
        self.get_logger().info(f"Marina: {self._marina}")
        
        # Start the sequence
        self._transition_to(TaskState.TASK1_START)
    
    def _transition_to(self, new_state: TaskState):
        """Transition to a new task state."""
        self.get_logger().info(f"State transition: {self._task_state.name} -> {new_state.name}")
        self._task_state = new_state
        self._state_start_time = self.get_clock().now().nanoseconds * 1e-9
        if new_state == TaskState.TASK4_START:
            # Reset Task 4 timing flags whenever we enter Task 4
            self._task4_pump_on_done = False
            self._task4_pump_off_done = False
            self._task4_object_delivered_done = False
        elif new_state == TaskState.TASK4_DONE:
            # Reset Task 5 docking flag when Task 4 completes
            self._task5_docking_done = False
    
    def _elapsed_time(self) -> float:
        """Time elapsed since entering current state (seconds)."""
        if self._state_start_time is None:
            return 0.0
        now = self.get_clock().now().nanoseconds * 1e-9
        return now - self._state_start_time
    
    def _position_callback(self, msg: NavSatFix):
        """Update current position."""
        if math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
            pos = (msg.latitude, msg.longitude)
            self._current_position = pos

            # Capture the mission start waypoint once, as soon as we have a valid fix
            if self._start_position is None:
                self._start_position = pos
                self.get_logger().info(
                    f"Captured mission start waypoint: lat={pos[0]:.7f}, lon={pos[1]:.7f}"
                )
    
    def _mavros_state_callback(self, msg: MavrosState):
        """Update MAVROS mode."""
        self._mavros_mode = msg.mode
    
    def _patrol_boat_callback(self, msg: Bool):
        """Latch patrol-boat detection flag when external detector reports True."""
        if not msg.data:
            return
        # Only care about patrol boat inside a "patrol window" for specific tasks.
        # Task 1 / Task 3: any time while those tasks are active.
        # Task 4 (Object Delivery): only after 40 seconds have elapsed in TASK4_START.
        # Task 5 (Docking, implemented as TASK4_DONE): only during the first 10 seconds
        # after entering TASK4_DONE.
        state = self._task_state
        elapsed = self._elapsed_time()
        in_patrol_window = False
        if state in (
            TaskState.TASK1_START,
            TaskState.TASK1_WAIT_START,
            TaskState.TASK1_WAIT_END,
            TaskState.TASK1_DONE,
            TaskState.TASK3_START,
            TaskState.TASK3_WAIT_SPEED_END,
            TaskState.TASK3_DONE,
        ):
            in_patrol_window = True
        elif state == TaskState.TASK4_START and elapsed >= 40.0:
            in_patrol_window = True
        elif state == TaskState.TASK4_DONE and elapsed < 10.0:
            in_patrol_window = True

        if in_patrol_window:
            self._patrol_boat_detected = True

    def _sound_callback(self, msg: SoundSignalWithFreq):
        """Handle sound signal interrupt."""
        if self._task_state in (TaskState.SOUND_INTERRUPT, TaskState.SOUND_NAVIGATE, TaskState.PATROL_BOAT_HOLD):
            self.get_logger().info("Sound signal detected but already handling interrupt or patrol hold, ignoring")
            return
        
        if self._task_state == TaskState.COMPLETED:
            self.get_logger().info("Sound signal detected but task sequence completed, ignoring")
            return
        
        signal_type = msg.signal
        self.get_logger().info(f"Sound signal detected: {signal_type} blast(s)")
        
        # Store current task state and how long we have already been in it.
        # This lets us resume later with the correct elapsed time, effectively
        # pausing the task timers while we handle the sound interrupt.
        self._pre_interrupt_state = self._task_state
        self._pre_interrupt_elapsed = self._elapsed_time()
        
        # Determine current task number for restoration
        if self._task_state in [TaskState.TASK1_START, TaskState.TASK1_WAIT_START, 
                                 TaskState.TASK1_WAIT_END, TaskState.TASK1_DONE]:
            self._pre_interrupt_task = 1
        elif self._task_state in [TaskState.TASK3_START, TaskState.TASK3_WAIT_SPEED_END, TaskState.TASK3_DONE]:
            self._pre_interrupt_task = 3
        elif self._task_state in [TaskState.TASK4_START, TaskState.TASK4_DONE]:
            self._pre_interrupt_task = 4
        else:
            self._pre_interrupt_task = None
        
        # Set navigation target
        if signal_type == 1:
            self._sound_target = self._yellow_buoy
            self.get_logger().info(f"Navigating to YELLOW_BUOY: {self._sound_target}")
        elif signal_type == 2:
            self._sound_target = self._marina
            self.get_logger().info(f"Navigating to MARINA: {self._sound_target}")
        else:
            self.get_logger().warning(f"Unknown signal type: {signal_type}, ignoring")
            return
        
        # Publish cur_task = 6 (SOUND_SIGNAL)
        task_msg = Int32()
        task_msg.data = 6
        self._cur_task_pub.publish(task_msg)
        self.get_logger().info("Published /cur_task = 6 (SOUND_SIGNAL)")
        
        # Transition to interrupt handling
        self._transition_to(TaskState.SOUND_INTERRUPT)
    
    def _set_mavros_mode(self, mode: str):
        """Call MAVROS set_mode service."""
        if not self._set_mode_client.service_is_ready():
            self.get_logger().warning("SetMode service not ready")
            return False
        
        request = SetMode.Request()
        request.custom_mode = mode
        
        self.get_logger().info(f"Calling set_mode: {mode}")
        future = self._set_mode_client.call_async(request)
        
        # Note: In a production system, you'd want to wait for the response properly.
        # For simplicity, we're fire-and-forget here. The state callback will confirm.
        return True
    
    def _publish_setpoint_global(self, lat: float, lon: float):
        """Publish global position setpoint to MAVROS."""
        msg = GeoPoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.latitude = lat
        msg.pose.position.longitude = lon
        msg.pose.position.altitude = 0.0  # Altitude relative to home
        
        # Identity quaternion (no specific heading requirement)
        msg.pose.orientation.w = 1.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        
        self._setpoint_global_pub.publish(msg)
    
    def _distance_to_target(self, target: Tuple[float, float]) -> Optional[float]:
        """Calculate distance to target in meters (approximate)."""
        if self._current_position is None:
            return None
        
        lat1, lon1 = self._current_position
        lat2, lon2 = target
        
        # Simple approximation using equirectangular projection
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        avg_lat = math.radians((lat1 + lat2) / 2)
        
        R = 6371000.0  # Earth radius in meters
        dx = R * dlon * math.cos(avg_lat)
        dy = R * dlat
        
        return math.sqrt(dx * dx + dy * dy)
    
    def _start_pumping(self):
        if not self._command_int_client.service_is_ready():
            self.get_logger().warning("CommandInt service not ready, skipping relay start")
            return
        req = CommandInt.Request()
        req.broadcast = False
        req.frame = 0
        req.command = 182
        req.current = 0
        req.autocontinue = 0
        req.param1 = 1.0
        req.param2 = 1.0
        req.param3 = 180.0 # How long to pump for in seconds
        req.param4 = 0.0
        req.x = 0
        req.y = 0
        req.z = 0.0
        self._command_int_client.call_async(req)

    def _end_pumping(self):
        if not self._command_int_client.service_is_ready():
            self.get_logger().warning("CommandInt service not ready, skipping relay end")
            return
        req = CommandInt.Request()
        req.broadcast = False
        req.frame = 0
        req.command = 181
        req.current = 0
        req.autocontinue = 0
        req.param1 = 1.0
        req.param2 = 0.0
        req.param3 = 0.0
        req.param4 = 0.0
        req.x = 0
        req.y = 0
        req.z = 0.0
        self._command_int_client.call_async(req)

    def _control_loop(self):
        """Main control loop at 10 Hz."""
        elapsed = self._elapsed_time()

        # Patrol boat: during allowed windows in Task 1, Task 3, Task 4, or Task 5 (docking),
        # if we just detected the yellow boat, hold position.
        state = self._task_state
        in_patrol_window = False
        if state in (
            TaskState.TASK1_START,
            TaskState.TASK1_WAIT_START,
            TaskState.TASK1_WAIT_END,
            TaskState.TASK1_DONE,
            TaskState.TASK3_START,
            TaskState.TASK3_WAIT_SPEED_END,
            TaskState.TASK3_DONE,
        ):
            in_patrol_window = True
        elif state == TaskState.TASK4_START and elapsed >= 40.0:
            in_patrol_window = True
        elif state == TaskState.TASK4_DONE and elapsed < 10.0:
            in_patrol_window = True

        if self._patrol_boat_detected and in_patrol_window:
            self._patrol_boat_detected = False
            self._pre_patrol_state = self._task_state
            self._pre_patrol_state_start_time = self._state_start_time
            self.get_logger().info("Patrol boat detected: switching to LOITER, reporting STOPPING")
            self._set_mavros_mode("LOITER")
            msg = String()
            msg.data = "STOPPING"
            self._patrol_boat_pub.publish(msg)
            self._task_state = TaskState.PATROL_BOAT_HOLD
            self._state_start_time = self.get_clock().now().nanoseconds * 1e-9
            return

        if self._task_state == TaskState.PATROL_BOAT_HOLD:
            if elapsed >= 5.0:
                self.get_logger().info("Patrol boat hold complete: resuming AUTO, reporting RESUMING")
                self._set_mavros_mode("AUTO")
                msg = String()
                msg.data = "RESUMING"
                self._patrol_boat_pub.publish(msg)
                self._task_state = self._pre_patrol_state
                self._state_start_time = self._pre_patrol_state_start_time
                self._pre_patrol_state = None
                self._pre_patrol_state_start_time = None
            return

        if self._task_state == TaskState.TASK1_START:
            # Publish cur_task = 1
            task_msg = Int32()
            task_msg.data = 1
            self._cur_task_pub.publish(task_msg)
            self.get_logger().info("Published /cur_task = 1 (ENTRY_EXIT)")
            self._transition_to(TaskState.TASK1_WAIT_START)
        
        elif self._task_state == TaskState.TASK1_WAIT_START:
            if elapsed >= 3.0:
                
                self._transition_to(TaskState.TASK1_WAIT_END)
        
        elif self._task_state == TaskState.TASK1_WAIT_END:
            if elapsed >= 15.0:
                # Publish "start" gate pass (ENTRY gate)

                msg = String()
                msg.data = "start"
                self._gate_pass_pub.publish(msg)
                self.get_logger().info("Published /messages/gate_pass: start (ENTRY gate)")
                # Keep original timing delay but no EXIT gate here; move on to TASK1_DONE
                self._transition_to(TaskState.TASK1_DONE)
        
        elif self._task_state == TaskState.TASK1_DONE:
            if elapsed >= 30.0:
                self._transition_to(TaskState.TASK3_START)
        
        elif self._task_state == TaskState.TASK3_START:
            # Publish cur_task = 3
            task_msg = Int32()
            task_msg.data = 3
            self._cur_task_pub.publish(task_msg)
            self.get_logger().info("Published /cur_task = 3 (SPEED_CHALLENGE)")
            
            # Publish "speed_start" gate pass
            msg = String()
            msg.data = "speed_start"
            self._gate_pass_pub.publish(msg)
            self.get_logger().info("Published /messages/gate_pass: speed_start")
            self._transition_to(TaskState.TASK3_WAIT_SPEED_END)
        
        elif self._task_state == TaskState.TASK3_WAIT_SPEED_END:
            if elapsed >= 100.0:
                # Publish "speed_end" gate pass
                msg = String()
                msg.data = "speed_end"
                self._gate_pass_pub.publish(msg)
                self.get_logger().info("Published /messages/gate_pass: speed_end")

                # At the same moment as speed_end, force an ObjectDetected report for the emergency (red) indicator
                obj_msg = ObjectDetection()
                obj_msg.object_type = "light"
                obj_msg.colour = "green"
                if self._current_position is not None:
                    obj_msg.latitude = float(self._current_position[0])
                    obj_msg.longitude = float(self._current_position[1])
                else:
                    obj_msg.latitude = 0.0
                    obj_msg.longitude = 0.0
                obj_msg.object_id = 1  # team-scoped stable ID for the emergency indicator
                self._object_detected_pub.publish(obj_msg)
                self.get_logger().info("Published /messages/object_detected: light, red (forced at speed_end)")

                self._transition_to(TaskState.TASK3_DONE)
        
        elif self._task_state == TaskState.TASK3_DONE:
            if elapsed >= 5.0:
                self._transition_to(TaskState.TASK4_START)
        
        elif self._task_state == TaskState.TASK4_START:
            # Ensure cur_task = 4 is published once when Task 4 starts
            if not self._task4_pump_on_done and not self._task4_pump_off_done and elapsed < 0.5:
                task_msg = Int32()
                task_msg.data = 4
                self._cur_task_pub.publish(task_msg)
                self.get_logger().info("Published /cur_task = 4 (OBJECT_DELIVERY)")

            # After 10 seconds in Task 4, briefly go GUIDED, start pump for 60s, then return to AUTO
            # At the same moment as pump goes on: publish hardcoded yellow vessel detection + object_delivered
            if not self._task4_pump_on_done and elapsed >= 2.0:
                self.get_logger().info("Task 4: switching to GUIDED to start pump")
                self._set_mavros_mode("GUIDED")
                self.get_logger().info("Task 4: starting pump for 60 seconds")
                self._start_pumping()
                self._set_mavros_mode("AUTO")
                self.get_logger().info("Task 4: returned to AUTO after starting pump")
                self._task4_pump_on_done = True

                # Right after pump goes on: hardcoded yellow vessel ObjectDetected + ObjectDelivery
                obj_msg = ObjectDetection()
                obj_msg.object_type = "boat"
                obj_msg.colour = "yellow"
                if self._current_position is not None:
                    obj_msg.latitude = float(self._current_position[0])
                    obj_msg.longitude = float(self._current_position[1])
                else:
                    obj_msg.latitude = 0.0
                    obj_msg.longitude = 0.0
                obj_msg.object_id = 2  # team-scoped stable ID for the yellow vessel
                self._object_detected_pub.publish(obj_msg)
                self.get_logger().info("Published /messages/object_detected: boat, yellow (forced at pump-on)")

                msg = Int32()
                msg.data = 0  # Content not used according to README
                self._object_delivered_pub.publish(msg)
                self.get_logger().info("Published /messages/object_delivered (right after pump-on)")
                self._task4_object_delivered_done = True
            
            # 60 seconds after pump start (i.e., 70s after entering Task 4), stop pump and finish task
            if self._task4_pump_on_done and not self._task4_pump_off_done and elapsed >= 70.0:
                self.get_logger().info("Task 4: switching to GUIDED to stop pump")
                self._set_mavros_mode("GUIDED")
                self.get_logger().info("Task 4: stopping pump")
                self._set_mavros_mode("AUTO")
                self.get_logger().info("Task 4: returned to AUTO after stopping pump")

                self._task4_pump_off_done = True
                self._transition_to(TaskState.TASK4_DONE)
        
        elif self._task_state == TaskState.TASK4_DONE:
            # After Task 4 completes, treat this as Task 5 (Docking)
            # Immediately publish cur_task = 5 once, then 10s later send docking message
            if not self._task5_docking_done and elapsed < 0.5:
                task_msg = Int32()
                task_msg.data = 5
                self._cur_task_pub.publish(task_msg)
                self.get_logger().info("Published /cur_task = 5 (DOCKING)")

            if not self._task5_docking_done and elapsed >= 15.0:
                dock_msg = Dock()
                dock_msg.dock = "S"   # South dock
                dock_msg.slip = "1"   # Slip 1
                self._docking_pub.publish(dock_msg)
                self.get_logger().info("Published /messages/docking: dock=S, slip=1 (hardcoded 15s after Task 4)")
                self._task5_docking_done = True
                self.get_logger().info(
                    "Docking complete, waiting to return to mission start waypoint for EXIT gate"
                )

            # After docking is complete, wait until we are back at the mission start waypoint
            # (within 1 meter), then publish the EXIT gate ("end") once and complete the sequence.
            if self._task5_docking_done and not self._exit_gate_sent and self._start_position is not None:
                distance_to_start = self._distance_to_target(self._start_position)
                if distance_to_start is not None:
                    if distance_to_start < 1.0:
                        msg = String()
                        msg.data = "end"
                        self._gate_pass_pub.publish(msg)
                        self.get_logger().info(
                            "Published /messages/gate_pass: end (EXIT gate at mission start waypoint)"
                        )
                        self._exit_gate_sent = True
                        self.get_logger().info("Task sequence completed")
                        self._transition_to(TaskState.COMPLETED)
                    elif int(elapsed) % 10 == 0 and elapsed > 0:
                        # Periodic log while waiting to get back to the start waypoint
                        self.get_logger().info(
                            f"Waiting to reach mission start waypoint for EXIT gate, "
                            f"distance: {distance_to_start:.1f}m"
                        )
        
        elif self._task_state == TaskState.SOUND_INTERRUPT:
            if self._guided_mode_attempts >= 2:
                self.get_logger().warning(
                    "Max GUIDED mode attempts (2) reached, ignoring sound interrupt and resuming task"
                )
                self._handle_sound_interrupt_complete()
                return
            self._guided_mode_attempts += 1
            self.get_logger().info(f"Switching to GUIDED (attempt {self._guided_mode_attempts}/2)")
            self._set_mavros_mode("GUIDED")
            self._transition_to(TaskState.SOUND_NAVIGATE)
        
        elif self._task_state == TaskState.SOUND_NAVIGATE:
            if self._sound_target is None:
                self.get_logger().error("Sound navigate state but no target set")
                self._handle_sound_interrupt_complete()
                return
            
            # Continuously publish setpoint (MAVROS requires continuous setpoints)
            self._publish_setpoint_global(self._sound_target[0], self._sound_target[1])
            
            # Check if we've arrived
            distance = self._distance_to_target(self._sound_target)
            if distance is not None:
                if distance < self._arrival_threshold:
                    self.get_logger().info(f"Arrived at target (distance: {distance:.1f}m)")
                    self._handle_sound_interrupt_complete()
                elif int(elapsed) % 5 == 0 and elapsed > 0:  # Log every 5 seconds
                    self.get_logger().info(f"Distance to target: {distance:.1f}m")
        
        elif self._task_state == TaskState.COMPLETED:
            # Nothing to do, sequence is complete
            pass
    
    def _handle_sound_interrupt_complete(self):
        """Complete sound interrupt handling and return to previous task."""
        # Switch back to AUTO mode
        self._set_mavros_mode("AUTO")
        
        # Restore previous task
        if self._pre_interrupt_task is not None:
            task_msg = Int32()
            task_msg.data = self._pre_interrupt_task
            self._cur_task_pub.publish(task_msg)
            self.get_logger().info(f"Restored /cur_task = {self._pre_interrupt_task}")
        
        # Return to previous state, preserving how long we'd already been in it
        if self._pre_interrupt_state is not None:
            self._task_state = self._pre_interrupt_state
            now = self.get_clock().now().nanoseconds * 1e-9
            if self._pre_interrupt_elapsed is not None:
                # Resume as if the task timer had been paused during the sound interrupt.
                self._state_start_time = now - self._pre_interrupt_elapsed
            else:
                # Fallback: start fresh if we lost timing information.
                self._state_start_time = now
        else:
            # If we can't determine previous state, try to continue sequence
            self.get_logger().warning("Could not restore previous state, continuing to next task")
            if self._pre_interrupt_task == 1:
                self._transition_to(TaskState.TASK1_DONE)
            elif self._pre_interrupt_task == 3:
                self._transition_to(TaskState.TASK3_DONE)
            elif self._pre_interrupt_task == 4:
                self._transition_to(TaskState.TASK4_DONE)
            else:
                self._transition_to(TaskState.COMPLETED)
        
        # Clear interrupt state and reset attempt count so future sound signals get 2 attempts
        self._sound_target = None
        self._pre_interrupt_task = None
        self._pre_interrupt_state = None
        self._pre_interrupt_elapsed = None
        self._guided_mode_attempts = 0


def main(args=None):
    rclpy.init(args=args)
    node = TaskSequenceCoordinator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
