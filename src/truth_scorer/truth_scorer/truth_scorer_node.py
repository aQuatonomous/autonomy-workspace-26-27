#!/usr/bin/env python3
"""truth_scorer_node: /fused_buoys + /boat_pose -> /scored_objects.

A pure observer. Nothing in the control path subscribes to /scored_objects, so if this node
dies the boat still runs the course exactly as before. That is deliberate: the scorer earns its
way into the planner by being measured first, not by being wired in first.

All the maths lives in truth_scorer.core, which imports no ROS at all. This file is only
plumbing: unpack messages, call ObjectMap.update(), pack the result back up.

Two message-level hazards it handles explicitly, both discovered by reading the fusion node:

1. **Duplicate frames.** vision_lidar_fusion publishes on *both* its CV and its LiDAR callback
   (:97 and :101), so one LiDAR scan can be republished several times as CV frames arrive. Those
   repeats are not new evidence. The viewpoint discount would absorb most of the damage, but
   observation_count and the stats would still be inflated, so identical frames are dropped
   outright by fingerprint.

2. **Stale CV.** The class on a FusedBuoy can be older than the LiDAR geometry it is attached to,
   because fusion reuses `_latest_cv_detections` whenever a new scan arrives. If the message
   carries a `class_stamp` we compute a real CV age and let the classifier discard stale evidence;
   if it does not, we pass None and the classifier treats the age as unknown. Both fields are read
   with getattr so this node works against the FusedBuoy definition before *and* after the
   class_confidence/class_stamp append.
"""

from __future__ import annotations

import math
import os
from typing import List, Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from builtin_interfaces.msg import Time as TimeMsg
from global_frame.msg import BoatPose as BoatPoseMsg
from pointcloud_filters.msg import FusedBuoyArray
from truth_scorer_msgs.msg import ScoredObject, ScoredObjectArray

from truth_scorer.config_io import load_confusion_matrix, load_scorer_config
from truth_scorer.core.association import Measurement
from truth_scorer.core.classes import UNKNOWN_ID
from truth_scorer.core.scorer import ObjectMap
from truth_scorer.core.visibility import BoatPose


def _stamp_to_sec(stamp: Optional[TimeMsg]) -> Optional[float]:
    if stamp is None:
        return None
    sec = int(getattr(stamp, 'sec', 0))
    nsec = int(getattr(stamp, 'nanosec', 0))
    if sec == 0 and nsec == 0:
        return None          # unset stamp, not "the epoch"
    return sec + nsec * 1e-9


def _sec_to_stamp(seconds: float) -> TimeMsg:
    stamp = TimeMsg()
    if seconds < 0.0:
        seconds = 0.0
    stamp.sec = int(seconds)
    stamp.nanosec = int(round((seconds - stamp.sec) * 1e9))
    if stamp.nanosec >= 1_000_000_000:          # rounding can carry
        stamp.sec += 1
        stamp.nanosec -= 1_000_000_000
    return stamp


def _normalize_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


class TruthScorerNode(Node):

    def __init__(self) -> None:
        super().__init__('truth_scorer_node')

        share = ''
        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory('truth_scorer')
        except Exception:
            pass

        self.declare_parameter('fused_buoys_topic', '/fused_buoys')
        self.declare_parameter('boat_pose_topic', '/boat_pose')
        self.declare_parameter('scored_objects_topic', '/scored_objects')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter(
            'scorer_config_file', os.path.join(share, 'config', 'truth_scorer.yaml') if share else '')
        self.declare_parameter(
            'confusion_prior_file',
            os.path.join(share, 'config', 'confusion_prior.yaml') if share else '')
        # Publish everything by default. Filtering here would hide exactly the low-confidence
        # objects the calibration report needs in order to say whether 0.2 really means 0.2.
        self.declare_parameter('publish_min_p_exist', 0.0)
        self.declare_parameter('boat_position_sigma_m', 0.5)
        self.declare_parameter('pose_stale_s', 2.0)
        self.declare_parameter('status_log_period_s', 10.0)

        self._map_frame = str(self.get_parameter('map_frame').value)
        self._publish_min_p = float(self.get_parameter('publish_min_p_exist').value)
        self._pose_sigma = float(self.get_parameter('boat_position_sigma_m').value)
        self._pose_stale_s = float(self.get_parameter('pose_stale_s').value)

        config_path = str(self.get_parameter('scorer_config_file').value)
        confusion_path = str(self.get_parameter('confusion_prior_file').value)
        config = load_scorer_config(config_path or None)
        confusion = load_confusion_matrix(confusion_path or None)
        self._map = ObjectMap(config, confusion)

        self._pose: Optional[BoatPose] = None
        self._pose_time: Optional[float] = None
        self._last_fingerprint: Optional[Tuple] = None
        self._dropped_duplicates = 0
        self._dropped_no_pose = 0
        self._frames_with_class = 0
        self._measurements_with_class = 0
        self._warned_missing_class_stamp = False

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(BoatPoseMsg, str(self.get_parameter('boat_pose_topic').value),
                                 self._pose_callback, qos)
        self.create_subscription(FusedBuoyArray, str(self.get_parameter('fused_buoys_topic').value),
                                 self._fused_callback, qos)
        self._pub = self.create_publisher(
            ScoredObjectArray, str(self.get_parameter('scored_objects_topic').value), qos)

        period = float(self.get_parameter('status_log_period_s').value)
        if period > 0.0:
            self.create_timer(period, self._log_status)

        self.get_logger().info(
            f'truth_scorer: config={config_path or "<defaults>"} '
            f'confusion={confusion_path or "<built-in prior>"} -> '
            f'{self.get_parameter("scored_objects_topic").value}')

    # -- clock -----------------------------------------------------------------------------------

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # -- callbacks -------------------------------------------------------------------------------

    def _pose_callback(self, msg: BoatPoseMsg) -> None:
        stamp = _stamp_to_sec(msg.header.stamp) or self._now()
        self._pose = BoatPose(
            east=float(msg.east),
            north=float(msg.north),
            yaw_rad=float(msg.heading_rad),
            stamp=stamp,
            position_sigma_m=self._pose_sigma,
        )
        self._pose_time = self._now()

    def _fingerprint(self, msg: FusedBuoyArray) -> Tuple:
        """Identify a *LiDAR* frame, ignoring the CV layer stacked on top of it.

        Keyed on the track geometry and the tracker's own last_seen, which is what actually
        changes when a new scan arrives. The CV class deliberately does not appear: a republish
        triggered purely by a new CV message carries no new range evidence and must not be
        counted as a second look.
        """
        return tuple(
            (int(b.id), round(float(b.range), 4), round(float(b.bearing), 5),
             int(b.last_seen.sec), int(b.last_seen.nanosec))
            for b in msg.buoys
        )

    def _fused_callback(self, msg: FusedBuoyArray) -> None:
        now = self._now()

        if self._pose is None:
            self._dropped_no_pose += 1
            return
        if self._pose_time is not None and (now - self._pose_time) > self._pose_stale_s:
            # Without a current pose we cannot place a detection, and we certainly cannot judge
            # whether a non-detection means "not there" or "not looking".
            self.get_logger().warn('boat pose is stale; skipping frame', throttle_duration_sec=5.0)
            return

        fingerprint = self._fingerprint(msg)
        if fingerprint == self._last_fingerprint:
            self._dropped_duplicates += 1
            return
        self._last_fingerprint = fingerprint

        pose = self._pose
        measurements = [m for m in (self._to_measurement(b, pose, now) for b in msg.buoys)
                        if m is not None]
        self._measurements_with_class += sum(1 for m in measurements if m.class_id != UNKNOWN_ID)
        if any(m.class_id != UNKNOWN_ID for m in measurements):
            self._frames_with_class += 1

        self._map.update(measurements, pose, now=now)
        self._publish(now)

    def _to_measurement(self, buoy, pose: BoatPose, now: float) -> Optional[Measurement]:
        range_m = float(buoy.range)
        if not math.isfinite(range_m) or range_m <= 0.0:
            return None
        bearing = float(buoy.bearing)
        if not math.isfinite(bearing):
            return None

        global_bearing = _normalize_angle(pose.yaw_rad + bearing)
        east = pose.east + range_m * math.cos(global_bearing)
        north = pose.north + range_m * math.sin(global_bearing)

        class_id = int(buoy.class_id)
        if not (0 <= class_id <= 22):
            class_id = UNKNOWN_ID

        # Present only after the FusedBuoy append; absent on an older build.
        score = getattr(buoy, 'class_confidence', None)
        if score is None:
            # Falling back to `confidence` would be wrong: that is the LiDAR point-count
            # heuristic (min(1, n_points/15)), not a class score, and weighting class evidence by
            # cluster density is nonsense. Substitute the reference score instead -- i.e. "an
            # ordinary detection" -- rather than 1.0, which would silently treat every unscored
            # observation as the most confident kind the detector can produce.
            score = self._map.config.classifier.score_reference
        else:
            score = float(score)

        class_stamp = getattr(buoy, 'class_stamp', None)
        cv_age_s: Optional[float] = None
        stamp_sec = _stamp_to_sec(class_stamp)
        if stamp_sec is not None:
            cv_age_s = max(0.0, now - stamp_sec)
        elif class_id != UNKNOWN_ID and not self._warned_missing_class_stamp:
            self._warned_missing_class_stamp = True
            self.get_logger().warn(
                'FusedBuoy has no class_stamp; CV staleness cannot be measured and stale class '
                'evidence will be accepted. Rebuild pointcloud_filters and cv_lidar_fusion.')

        return Measurement(
            east=east,
            north=north,
            range_m=range_m,
            global_bearing_rad=global_bearing,
            class_id=class_id,
            class_score=(score if math.isfinite(score) else 0.0),
            track_id=int(buoy.id),
            source='lidar',
            stamp=_stamp_to_sec(buoy.last_seen) or now,
            cv_age_s=cv_age_s,
        )

    # -- output ----------------------------------------------------------------------------------

    def _publish(self, now: float) -> None:
        out = ScoredObjectArray()
        out.header.stamp = _sec_to_stamp(now)
        out.header.frame_id = self._map_frame

        objects: List[ScoredObject] = []
        for snap in self._map.snapshot():
            if snap['p_exist'] < self._publish_min_p:
                continue
            o = ScoredObject()
            o.oid = int(snap['oid'])
            o.east = float(snap['east'])
            o.north = float(snap['north'])
            o.position_cov = [float(v) for v in snap['position_cov']]
            o.p_exist = float(snap['p_exist'])
            o.class_id = int(snap['class_id'])
            o.class_name = str(snap['class_name'])
            o.class_prob = float(snap['class_prob'])
            o.class_entropy = float(snap['class_entropy'])
            # Marginalised over the posterior, not class_colour(class_id). See ScoredObject.msg:
            # a red cross placard is `cross` (colourless) at the argmax but confidently red once
            # the three red buoy classes are summed, and colour is what the handbook scores.
            # colour_probabilities() buckets the colourless classes under the literal key
            # 'none'; on the wire that must be the empty string, or a consumer comparing
            # strings would treat "no colour" as a colour named "none".
            colour = str(snap['colour'] or '')
            o.colour = '' if colour == 'none' else colour
            o.colour_prob = float(snap['colour_prob'])
            o.observation_count = int(snap['observation_count'])
            o.track_id = int(snap['track_id'])
            o.first_seen = _sec_to_stamp(snap['first_seen'])
            o.last_seen = _sec_to_stamp(snap['last_seen'])
            objects.append(o)

        out.objects = objects
        self._pub.publish(out)

    def _log_status(self) -> None:
        s = self._map.stats
        if s['frames'] == 0:
            self.get_logger().warn(
                'no usable /fused_buoys frames yet '
                f'(dropped {self._dropped_no_pose} for missing pose, '
                f'{self._dropped_duplicates} duplicates)')
            return
        confident = len(self._map.confident_objects(0.5))
        # Class coverage is the first number to read in a bad run: if CV contributed nothing,
        # every class figure downstream is meaningless and should be reported as such.
        coverage = self._measurements_with_class / max(1, s['measurements'])
        self.get_logger().info(
            f"objects={len(self._map)} (p>0.5: {confident}) frames={s['frames']} "
            f"meas={s['measurements']} match={s['matches']} birth={s['births']} "
            f"defer={s['deferred']} miss={s['expected_misses']} prune={s['pruned']} "
            f"dup_dropped={self._dropped_duplicates} class_coverage={coverage:.0%}")


def main(args=None):
    rclpy.init(args=args)
    node = TruthScorerNode()
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
