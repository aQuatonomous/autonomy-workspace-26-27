#!/usr/bin/env python3
"""Synthetic /boat_pose + /fused_buoys, driven from a real answer key.

Gazebo runs at 0.5-1.1x realtime and needs a GPU, a SITL, and five terminals. That is the right
place to find out whether the *perception* works, and the wrong place to find out that a field
was named `east` instead of `x`. This node fakes the two inputs the scorer consumes, from the
same answer key the evaluator reads, so the whole pipeline -- frame conversion, association,
scoring, matching, report generation -- can be exercised end to end in seconds.

It is a plumbing test, not a perception test. The detections it emits come from the truth, so
good numbers here prove only that the machinery is wired up; they say nothing about the real
detector. Expect it to score well, and be suspicious if it does not.

  ros2 run truth_scorer simulate_course --ros-args -p answer_key:=<key>.json

Knobs worth using: `miss_rate` and `false_positive_rate` to check that existence responds the way
it should, `mislabel_rate` to check the class posterior, and `speed` to run the whole course in
a few seconds.
"""

from __future__ import annotations

import math
import os
import random
from typing import List, Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from builtin_interfaces.msg import Time as TimeMsg
from global_frame.msg import BoatPose as BoatPoseMsg
from pointcloud_filters.msg import FusedBuoy, FusedBuoyArray

from truth_scorer.core.classes import UNKNOWN_ID, class_name
from truth_scorer.core.truth_mapping import load_truth_elements


class SimulateCourseNode(Node):

    def __init__(self) -> None:
        super().__init__('simulate_course')

        self.declare_parameter('answer_key', os.environ.get('SIM_ANSWER_KEY', ''))
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('speed_mps', 6.0)
        self.declare_parameter('sensor_range_m', 30.0)
        self.declare_parameter('position_noise_m', 0.10)
        self.declare_parameter('miss_rate', 0.10)
        self.declare_parameter('mislabel_rate', 0.15)
        self.declare_parameter('false_positive_rate', 0.05)
        self.declare_parameter('class_score', 0.70)
        self.declare_parameter('cv_stale_rate', 0.20)
        # Track ids are deliberately recycled, because that is the behaviour that motivated the
        # scorer: buoy_tracker drops a track after 10 misses and re-acquires it under a new id.
        self.declare_parameter('track_churn_period_s', 8.0)
        self.declare_parameter('seed', 1)

        key_path = str(self.get_parameter('answer_key').value)
        if not key_path or not os.path.isfile(key_path):
            raise RuntimeError(f'answer key not found: {key_path!r}')
        import json
        with open(key_path, 'r', encoding='utf-8') as handle:
            key = json.load(handle)

        self._truth = [e for e in load_truth_elements(key) if e.point_like]
        start = key.get('start') or {}
        self._offset = (float(start.get('x', 0.0)), float(start.get('y', 0.0)))

        # A simple patrol: visit every truth element in order, in the map frame.
        self._waypoints: List[Tuple[float, float]] = [
            (e.east - self._offset[0], e.north - self._offset[1]) for e in self._truth]
        self._wp = 0
        self._east, self._north, self._yaw = 0.0, 0.0, 0.0
        self._t = 0.0
        self._rng = random.Random(int(self.get_parameter('seed').value))
        self._next_track_id = 1
        self._track_ids = {e.element_id: self._take_track_id() for e in self._truth}
        self._last_churn = 0.0

        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        self._pose_pub = self.create_publisher(BoatPoseMsg, '/boat_pose', qos)
        self._fused_pub = self.create_publisher(FusedBuoyArray, '/fused_buoys', qos)

        rate = float(self.get_parameter('rate_hz').value)
        self._dt = 1.0 / rate
        self.create_timer(self._dt, self._tick)
        self.get_logger().info(
            f'simulating a course drive over {len(self._truth)} point-like elements from '
            f'{key_path}; this is a plumbing test, not a perception test')

    def _take_track_id(self) -> int:
        self._next_track_id += 1
        return self._next_track_id

    def _stamp(self, t: float) -> TimeMsg:
        s = TimeMsg()
        s.sec = int(t)
        s.nanosec = int((t - int(t)) * 1e9)
        return s

    def _tick(self) -> None:
        self._t += self._dt
        self._advance()
        self._publish_pose()
        self._publish_detections()

    def _advance(self) -> None:
        """Drive toward the next waypoint, standing off so the boat does not sit on top of it."""
        if not self._waypoints:
            return
        tx, ty = self._waypoints[self._wp % len(self._waypoints)]
        de, dn = tx - self._east, ty - self._north
        dist = math.hypot(de, dn)
        if dist < 4.0:
            self._wp += 1
            return
        self._yaw = math.atan2(dn, de)
        step = float(self.get_parameter('speed_mps').value) * self._dt
        self._east += step * math.cos(self._yaw)
        self._north += step * math.sin(self._yaw)

    def _publish_pose(self) -> None:
        msg = BoatPoseMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.east = self._east
        msg.north = self._north
        msg.heading_rad = self._yaw
        self._pose_pub.publish(msg)

    def _publish_detections(self) -> None:
        now = self.get_clock().now()
        now_s = now.nanoseconds * 1e-9
        out = FusedBuoyArray()
        out.header.stamp = now.to_msg()
        out.header.frame_id = 'rslidar'

        rng = self._rng
        sensor_range = float(self.get_parameter('sensor_range_m').value)
        noise = float(self.get_parameter('position_noise_m').value)
        miss = float(self.get_parameter('miss_rate').value)
        mislabel = float(self.get_parameter('mislabel_rate').value)
        score = float(self.get_parameter('class_score').value)
        stale_rate = float(self.get_parameter('cv_stale_rate').value)

        churn_period = float(self.get_parameter('track_churn_period_s').value)
        if churn_period > 0 and (self._t - self._last_churn) >= churn_period:
            self._last_churn = self._t
            victim = rng.choice(list(self._track_ids))
            self._track_ids[victim] = self._take_track_id()

        for e in self._truth:
            te = e.east - self._offset[0]
            tn = e.north - self._offset[1]
            de, dn = te - self._east, tn - self._north
            rng_m = math.hypot(de, dn)
            if rng_m > sensor_range or rng_m < 0.5:
                continue
            if rng.random() < miss:
                continue

            ee = te + rng.gauss(0.0, noise)
            nn = tn + rng.gauss(0.0, noise)
            out.buoys.append(self._make(
                ee, nn, self._track_ids[e.element_id],
                self._class_for(e, mislabel, rng), score, now_s, stale_rate, rng))

        # Clutter: a return with no object behind it, at a fresh place each time, so it is never
        # confirmed. This is what the existence filter is supposed to retire.
        if rng.random() < float(self.get_parameter('false_positive_rate').value):
            ee = self._east + rng.uniform(-sensor_range, sensor_range)
            nn = self._north + rng.uniform(-sensor_range, sensor_range)
            out.buoys.append(self._make(ee, nn, self._take_track_id(), UNKNOWN_ID, 0.0,
                                        now_s, stale_rate, rng))

        self._fused_pub.publish(out)

    def _class_for(self, element, mislabel_rate: float, rng: random.Random) -> int:
        truth_id = element.label.canonical_class_id
        if rng.random() >= mislabel_rate:
            return truth_id
        # Mislabels are drawn from the acceptable set when possible, so the default behaviour
        # exercises the equivalence-group logic rather than producing obvious nonsense.
        alternatives = sorted(element.label.acceptable_class_ids - {truth_id})
        return rng.choice(alternatives) if alternatives else UNKNOWN_ID

    def _make(self, east: float, north: float, track_id: int, class_id: int, score: float,
              now_s: float, stale_rate: float, rng: random.Random) -> FusedBuoy:
        de, dn = east - self._east, north - self._north
        b = FusedBuoy()
        b.id = int(track_id)
        b.range = float(math.hypot(de, dn))
        # /fused_buoys bearings are boat-relative, which is exactly the conversion the scorer
        # has to get right; emitting map-frame bearings here would hide a sign error forever.
        b.bearing = float(math.atan2(math.sin(math.atan2(dn, de) - self._yaw),
                                     math.cos(math.atan2(dn, de) - self._yaw)))
        b.x = float(b.range * math.cos(b.bearing))
        b.y = float(b.range * math.sin(b.bearing))
        b.z_mean = 0.0
        b.class_id = int(class_id)
        b.class_name = class_name(class_id)
        b.confidence = 0.8
        b.observation_count = 1
        b.first_seen = self._stamp(now_s)
        b.last_seen = self._stamp(now_s)
        b.class_confidence = float(score if class_id != UNKNOWN_ID else 0.0)
        if class_id == UNKNOWN_ID:
            b.class_stamp = TimeMsg()
        else:
            age = 1.5 if rng.random() < stale_rate else 0.05
            b.class_stamp = self._stamp(max(0.0, now_s - age))
        return b


def main(args=None):
    rclpy.init(args=args)
    node = SimulateCourseNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
