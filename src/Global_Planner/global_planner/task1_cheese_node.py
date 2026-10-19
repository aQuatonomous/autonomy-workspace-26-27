#!/usr/bin/env python3
"""
Task1 cheese: no planning. Go straight 5s (steer between gates if seen: red left, green right),
then turn 180 and go straight back to start. Uses /boat_pose and /global_detections only;
publishes cmd_vel (fused outputs, no local planner).
"""

from __future__ import annotations

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from global_frame.msg import BoatPose, GlobalDetectionArray

# class_id from entities.py: 1=green_buoy, 2=green_pole, 3=red_buoy, 4=red_pole
RED_IDS = (3, 4)
GREEN_IDS = (1, 2)

OUT_DURATION = 10.0
SPEED_M_S = 0.5
TURN_RATE_RAD_S = 0.4
HEADING_TOL_RAD = 0.15
RETURN_STOP_RADIUS_M = 0.8


class Task1CheeseNode(Node):
    def __init__(self):
        super().__init__("task1_cheese_node")
        self.declare_parameter("cmd_vel_topic", "/uas1/mavros/setpoint_velocity/cmd_vel_unstamped")
        self._cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        self._cmd_vel_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self.create_subscription(BoatPose, "/boat_pose", self._pose_cb, 10)
        self.create_subscription(GlobalDetectionArray, "/global_detections", self._detections_cb, 10)

        self._east: float | None = None
        self._north: float | None = None
        self._heading_rad: float | None = None
        self._start_east: float | None = None
        self._start_north: float | None = None
        self._start_heading_rad: float | None = None
        self._out_start_time: float | None = None
        self._gate_mid_east: float | None = None
        self._gate_mid_north: float | None = None
        self._state = "wait_pose"  # wait_pose -> out -> turn -> return -> done

        self._timer = self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"task1_cheese: no planner; straight 5s then return to start. cmd_vel={self._cmd_vel_topic}"
        )

    def _pose_cb(self, msg: BoatPose) -> None:
        self._east = msg.east
        self._north = msg.north
        self._heading_rad = msg.heading_rad
        if self._state == "wait_pose" and self._start_east is None:
            self._start_east = msg.east
            self._start_north = msg.north
            self._start_heading_rad = msg.heading_rad
            self._out_start_time = self.get_clock().now().nanoseconds / 1e9
            self._state = "out"
            self.get_logger().info(
                f"Start recorded: east={self._start_east:.1f} north={self._start_north:.1f}"
            )

    def _detections_cb(self, msg: GlobalDetectionArray) -> None:
        reds = []
        greens = []
        for d in msg.detections:
            cid = getattr(d, "class_id", 255)
            if cid in RED_IDS:
                reds.append((float(d.east), float(d.north)))
            elif cid in GREEN_IDS:
                greens.append((float(d.east), float(d.north)))
        if not reds or not greens:
            self._gate_mid_east = None
            self._gate_mid_north = None
            return
        # Nearest red-green pair as gate; midpoint
        best_mid = None
        best_dist = 1e9
        if self._east is not None and self._north is not None:
            for r in reds:
                for g in greens:
                    mid_e = (r[0] + g[0]) * 0.5
                    mid_n = (r[1] + g[1]) * 0.5
                    d = math.hypot(mid_e - self._east, mid_n - self._north)
                    if d < best_dist:
                        best_dist = d
                        best_mid = (mid_e, mid_n)
        if best_mid is not None:
            self._gate_mid_east, self._gate_mid_north = best_mid
        else:
            self._gate_mid_east = None
            self._gate_mid_north = None

    def _publish_zero(self) -> None:
        t = Twist()
        t.linear.x = 0.0
        t.linear.y = 0.0
        t.linear.z = 0.0
        t.angular.x = 0.0
        t.angular.y = 0.0
        t.angular.z = 0.0
        self._cmd_vel_pub.publish(t)

    def _publish_velocity_ned(self, north: float, east: float) -> None:
        """Publish world-frame velocity as MAVROS NED: linear.x = North, linear.y = East."""
        t = Twist()
        t.linear.x = float(north)
        t.linear.y = float(east)
        t.linear.z = 0.0
        t.angular.x = 0.0
        t.angular.y = 0.0
        t.angular.z = 0.0
        self._cmd_vel_pub.publish(t)

    def _tick(self) -> None:
        if self._east is None or self._north is None or self._heading_rad is None:
            self._publish_zero()
            return

        if self._state == "wait_pose":
            self._publish_zero()
            return

        if self._state == "out":
            t = self.get_clock().now().nanoseconds / 1e9
            elapsed = t - (self._out_start_time or t)
            if elapsed >= OUT_DURATION:
                self._state = "turn"
                self.get_logger().info("Out 5s done -> turn around")
                return
            # Straight, or toward gate midpoint if visible (red left, green right -> go between)
            if self._gate_mid_east is not None and self._gate_mid_north is not None:
                desired = math.atan2(
                    self._gate_mid_north - self._north,
                    self._gate_mid_east - self._east,
                )
            else:
                desired = self._heading_rad
            v_east = SPEED_M_S * math.cos(desired)
            v_north = SPEED_M_S * math.sin(desired)
            self._publish_velocity_ned(v_north, v_east)
            return

        if self._state == "turn":
            target_h = (self._start_heading_rad or 0.0) + math.pi
            target_h = math.atan2(math.sin(target_h), math.cos(target_h))
            err = math.atan2(
                math.sin(target_h - self._heading_rad),
                math.cos(target_h - self._heading_rad),
            )
            if abs(err) < HEADING_TOL_RAD:
                self._state = "return"
                self.get_logger().info("Turn done -> return to start")
                return
            t = Twist()
            t.linear.x = 0.0
            t.linear.y = 0.0
            t.linear.z = 0.0
            t.angular.x = 0.0
            t.angular.y = 0.0
            t.angular.z = TURN_RATE_RAD_S if err > 0 else -TURN_RATE_RAD_S
            self._cmd_vel_pub.publish(t)
            return

        if self._state == "return":
            dx = (self._start_east or 0) - self._east
            dy = (self._start_north or 0) - self._north
            dist = math.hypot(dx, dy)
            if dist < RETURN_STOP_RADIUS_M:
                self._state = "done"
                self.get_logger().info("Return complete -> stop")
                self._publish_zero()
                return
            desired = math.atan2(dy, dx)
            v_east = SPEED_M_S * math.cos(desired)
            v_north = SPEED_M_S * math.sin(desired)
            self._publish_velocity_ned(v_north, v_east)
            return

        # done
        self._publish_zero()


def main(args=None):
    rclpy.init(args=args)
    node = Task1CheeseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
