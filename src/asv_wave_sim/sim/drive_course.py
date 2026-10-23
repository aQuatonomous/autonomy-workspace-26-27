#!/usr/bin/env python3
"""Drive the boat around the course on answer-key waypoints, bypassing the team planner.

Why this exists: the global planner needs a red/green *gate pair* from CV before it will move,
and in sim the cameras are scoring 0.05-0.12 against a 0.25 threshold, so it never sees one and
the boat sits at the start. A perception scorer measured from a stationary boat is meaningless --
recall collapses to "whatever is visible from the dock" -- so for an instrumented scoring run we
drive the hull ourselves and let the perception stack observe.

This is a *measurement harness*, not autonomy: it reads the answer key, which the boat is of
course not allowed to do at competition. Nothing it does touches the perception path.

Waypoints are the gate centres in course order, then back to the start.

Frame note, verified against a live run rather than assumed: MAVROS's local ENU origin is the
*course* origin, not the boat's start pose. Measured simultaneously at one instant --
  /mavros/local_position/pose  (19.97, -28.17)
  /sim/ground_truth/odom       (19.79, -27.89)
  /boat_pose                   (-2.37,  -4.16)   + start (22.2, -23.9) = (19.83, -28.06)
-- so MAVROS agrees with Gazebo and the answer key directly, while /boat_pose is the one that is
shifted (it uses first-GPS-fix as origin). Gate centres therefore go to MAVROS *unmodified*.
Subtracting `start` here, as the obvious reading of the two frames suggests, would have aimed
every waypoint ~32 m off the course.

STATUS: the frame convention above is measured, but this script has not yet been run end to end
with the boat under way -- see the branch notes. Expect to shake out the usual MAVROS teething
problems (QoS on /mavros/local_position/pose above all). The STALL_S timeout means a silent pose
subscription degrades to "visit every waypoint on a timer" rather than hanging on the first one.
"""
import json
import math
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool

ACCEPT_M = 3.0          # waypoint capture radius; the hull is ~1.2 m and GPS wanders
STALL_S = 45.0          # give up on a waypoint rather than grind against an obstacle forever


def waypoints_from_key(path):
    key = json.load(open(path))
    pts = [(g['center'][0], g['center'][1], g['label']) for g in key['gates']]
    pts.append((key['start']['x'], key['start']['y'], 'return'))
    return pts


class Driver(Node):
    def __init__(self, pts):
        super().__init__('drive_course')
        self._pts = pts
        self._i = 0
        self._pose = None
        self._state = None
        self._since = self.get_clock().now()
        # MAVROS publishes state and local_position with SENSOR_DATA-ish QoS; match it or the
        # subscription silently never fires.
        sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST,
                            durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 self._on_pose, sensor)
        self.create_subscription(State, '/mavros/state', self._on_state, sensor)
        self._sp = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self._mode = self.create_client(SetMode, '/mavros/set_mode')
        self._arm = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.create_timer(0.1, self._tick)
        self.create_timer(5.0, self._ensure_guided)

    def _on_pose(self, msg):
        self._pose = (msg.pose.position.x, msg.pose.position.y)

    def _on_state(self, msg):
        self._state = msg

    def _ensure_guided(self):
        s = self._state
        if s is None:
            return
        if s.mode != 'GUIDED' and self._mode.service_is_ready():
            self._mode.call_async(SetMode.Request(custom_mode='GUIDED'))
        elif not s.armed and self._arm.service_is_ready():
            self._arm.call_async(CommandBool.Request(value=True))

    def _tick(self):
        if self._i >= len(self._pts):
            return
        x, y, label = self._pts[self._i]
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.pose.position.x, m.pose.position.y = x, y
        m.pose.orientation.w = 1.0
        self._sp.publish(m)

        if self._pose is None:
            return
        d = math.hypot(self._pose[0] - x, self._pose[1] - y)
        age = (self.get_clock().now() - self._since).nanoseconds * 1e-9
        if d < ACCEPT_M or age > STALL_S:
            self.get_logger().info(
                f'{"reached" if d < ACCEPT_M else "gave up on"} {label} '
                f'({x:.1f}, {y:.1f}) d={d:.1f}m after {age:.0f}s')
            self._i += 1
            self._since = self.get_clock().now()
        elif age % 10 < 0.1:
            self.get_logger().info(f'-> {label} ({x:.1f}, {y:.1f}) d={d:.1f}m '
                                   f'at ({self._pose[0]:.1f}, {self._pose[1]:.1f})')


def main():
    key = os.environ.get('SIM_ANSWER_KEY') or sys.argv[1]
    pts = waypoints_from_key(key)
    print(f'{len(pts)} waypoints: ' + ', '.join(f'{p[2]}({p[0]:.0f},{p[1]:.0f})' for p in pts))
    rclpy.init()
    node = Driver(pts)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
