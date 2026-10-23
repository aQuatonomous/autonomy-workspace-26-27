#!/usr/bin/env python3
"""truth_eval_node: /scored_objects + the answer key -> a run report.

This is the node that knows the truth, and it is the only one. truth_scorer_node never loads the
answer key, which is what keeps the scorer itself deployable at a competition where no answer key
exists. Turning this node off changes no published topic.

The report is written top-down in the order you should actually read it:

1. **Input health.** If the cameras contributed nothing, every class number below is
   uninterpretable, and the report must say so before you waste an afternoon tuning a posterior
   that never received evidence.
2. **Frame offset.** The map origin is the boat's first GPS fix; the answer key's origin is the
   course centre. If that offset is wrong, every metric reads zero and the cause is invisible.
3. **Existence** precision/recall, against the objects the boat actually drove within range of.
4. **Calibration** -- ECE and Brier. When it says 0.8, is it right 80 % of the time?
5. **Position** error percentiles and NEES: is the covariance honest, or merely decorative?
6. **Flicker and track-id churn** -- the direct before/after numbers against today's behaviour.

The JSONL alongside it carries per-detection records so analyse_run.py can re-fit the confusion
matrix and sweep tunables without paying for another 0.5-1.1x-realtime sim run.
"""

from __future__ import annotations

import datetime
import json
import math
import os
import threading
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from global_frame.msg import BoatPose as BoatPoseMsg
from pointcloud_filters.msg import FusedBuoyArray
from truth_scorer_msgs.msg import ScoredObjectArray

from truth_scorer.core import metrics
from truth_scorer.core.classes import UNKNOWN_ID, class_colour, colour_is_expressible
from truth_scorer.core.truth_mapping import load_truth_elements, unmapped_kinds


def _sigma_minor(cov_flat) -> float:
    """Shortest axis of the 1-sigma error ellipse, from a row-major flat 2x2."""
    a, b, c, d = (float(v) for v in cov_flat)
    tr, det = a + d, a * d - b * c
    disc = max(0.0, tr * tr / 4.0 - det)
    return math.sqrt(max(0.0, tr / 2.0 - math.sqrt(disc)))


class TruthEvalNode(Node):

    def __init__(self) -> None:
        super().__init__('truth_eval_node')

        self.declare_parameter('answer_key', os.environ.get('SIM_ANSWER_KEY', ''))
        self.declare_parameter('report_dir', os.environ.get('SIM_RUNS_DIR', ''))
        self.declare_parameter('scored_objects_topic', '/scored_objects')
        self.declare_parameter('boat_pose_topic', '/boat_pose')
        self.declare_parameter('fused_buoys_topic', '/fused_buoys')
        self.declare_parameter('match_distance_m', 2.0)
        self.declare_parameter('opportunity_range_m', 30.0)
        self.declare_parameter('jsonl_period_s', 1.0)
        self.declare_parameter('gazebo_odom_topic', '')     # '' = /model/<boat>/odometry
        self.declare_parameter('gazebo_boat_model', 'ourboat')
        self.declare_parameter('use_gazebo_offset', True)
        self.declare_parameter('status_log_period_s', 20.0)
        # Diagnostic only: never used to score, only to explain a conservative NEES. Keep it in
        # step with position.covariance_floor_m in truth_scorer.yaml.
        self.declare_parameter('covariance_floor_m', 0.25)

        self._match_m = float(self.get_parameter('match_distance_m').value)
        self._opportunity_m = float(self.get_parameter('opportunity_range_m').value)

        key_path = str(self.get_parameter('answer_key').value)
        if not key_path or not os.path.isfile(key_path):
            raise RuntimeError(
                f'answer key not found: {key_path!r}. Pass answer_key:=<path> or set '
                '$SIM_ANSWER_KEY (env.sh / sim_load_course does this).')
        with open(key_path, 'r', encoding='utf-8') as handle:
            self._key = json.load(handle)
        self._key_path = os.path.abspath(key_path)

        missing = unmapped_kinds(self._key)
        if missing:
            # Scoring an unmapped element as a perception failure looks identical to a broken
            # detector, and costs an afternoon to tell apart. Refuse instead.
            raise RuntimeError(
                f'answer key contains element kinds the scorer does not know: {missing}. '
                'Add them to core/truth_mapping.py rather than scoring them as misses.')

        self._truth = load_truth_elements(self._key)
        self._point_truth = [e for e in self._truth if e.point_like]
        start = self._key.get('start') or {}
        # answer_key_xy = map_xy + (start.x, start.y), because use_first_fix_as_origin puts the
        # ENU origin at the boat's start position while the key is in Gazebo world coordinates.
        self._expected_offset = (float(start.get('x', 0.0)), float(start.get('y', 0.0)))
        self._offset = self._expected_offset
        self._offset_source = 'answer key start (assumed)'
        self._offset_samples: List[Tuple[float, float]] = []

        # Opportunity: how close the boat ever got to each truth element. Recall against all 44
        # elements measures where the boat drove, not how well it saw.
        self._closest_approach = {e.element_id: float('inf') for e in self._truth}

        self._lock = threading.Lock()
        self._pose: Optional[Tuple[float, float, float]] = None
        self._last_snapshot: List[Dict] = []
        self._class_sequences: Dict[int, List[int]] = {}
        # Distinct LiDAR track ids seen on each persistent oid. Recomputed here rather than
        # taken from the scorer so the number is an independent observation of the wire, not
        # the scorer grading its own homework.
        self._track_ids: Dict[int, set] = {}
        self._first_stamp: Optional[float] = None
        self._last_stamp: Optional[float] = None
        self._scored_msgs = 0
        self._fused_msgs = 0
        self._fused_detections = 0
        self._fused_with_class = 0
        self._fused_scores: List[float] = []
        self._cv_ages: List[float] = []
        self._stale_cv = 0
        self._confusion_counts: Dict[int, Dict[int, float]] = {}
        self._last_frame_jsonl = 0.0
        self._last_detection_jsonl = 0.0

        report_dir = str(self.get_parameter('report_dir').value) or os.path.expanduser(
            '~/.aquatonomous_sim/runs')
        os.makedirs(report_dir, exist_ok=True)
        stem = datetime.datetime.now().strftime('%Y%m%d-%H%M%S') + \
            f'_seed{self._key.get("seed")}_truthscore'
        self._stem = os.path.join(report_dir, stem)
        self._jsonl = open(self._stem + '.jsonl', 'w', encoding='utf-8')
        self._write_jsonl({
            'type': 'meta',
            'answer_key': self._key_path,
            'seed': self._key.get('seed'),
            'world': self._key.get('world'),
            'start': start,
            'match_distance_m': self._match_m,
            'opportunity_range_m': self._opportunity_m,
            'truth_elements': [
                {'id': e.element_id, 'name': e.name, 'kind': e.kind, 'colour': e.colour,
                 # 'x'/'y' are the answer-key frame, matching the key's own vocabulary;
                 # TruthElement calls them east/north because they are ENU.
                 'x': e.east, 'y': e.north,
                 'canonical_class_id': e.label.canonical_class_id,
                 'acceptable_class_ids': sorted(e.label.acceptable_class_ids),
                 'point_like': e.point_like}
                for e in self._truth
            ],
        })

        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(ScoredObjectArray,
                                 str(self.get_parameter('scored_objects_topic').value),
                                 self._scored_callback, qos)
        self.create_subscription(BoatPoseMsg, str(self.get_parameter('boat_pose_topic').value),
                                 self._pose_callback, qos)
        self.create_subscription(FusedBuoyArray,
                                 str(self.get_parameter('fused_buoys_topic').value),
                                 self._fused_callback, qos)

        self._gz = None
        if bool(self.get_parameter('use_gazebo_offset').value):
            self._start_gazebo_truth()

        period = float(self.get_parameter('status_log_period_s').value)
        if period > 0.0:
            self.create_timer(period, self._log_status)

        self.get_logger().info(
            f'truth_eval: key={self._key_path} ({len(self._truth)} elements), '
            f'report -> {self._stem}.{{jsonl,json,md}}')

    # -- frame offset ----------------------------------------------------------------------------

    def _start_gazebo_truth(self) -> None:
        """Measure the map<->answer-key offset instead of trusting it.

        The expected value is the answer key's `start`, but a mis-set origin would make every
        metric read zero with no visible cause, so we compare /boat_pose against Gazebo's own
        odometry the way sim_judge does and shout if they disagree.
        """
        topic = str(self.get_parameter('gazebo_odom_topic').value)
        if not topic:
            topic = f'/model/{self.get_parameter("gazebo_boat_model").value}/odometry'
        try:
            from gz.msgs10.odometry_pb2 import Odometry
            from gz.transport13 import Node as GzNode
        except Exception as exc:
            self.get_logger().warn(
                f'gz-transport unavailable ({exc}); frame offset falls back to the answer key '
                f'start {self._expected_offset}. It will not be verified.')
            return

        def on_msg(msg):
            with self._lock:
                if self._pose is None:
                    return
                dx = msg.pose.position.x - self._pose[0]
                dy = msg.pose.position.y - self._pose[1]
                self._offset_samples.append((dx, dy))
                if len(self._offset_samples) > 2000:
                    del self._offset_samples[:1000]

        self._gz = GzNode()
        if not self._gz.subscribe(Odometry, topic, on_msg):
            self.get_logger().warn(f'could not subscribe to Gazebo odometry on {topic}')
            self._gz = None
        else:
            self.get_logger().info(f'measuring frame offset against Gazebo odometry on {topic}')

    def _resolve_offset(self) -> None:
        with self._lock:
            samples = list(self._offset_samples)
        if len(samples) < 20:
            self._offset = self._expected_offset
            self._offset_source = 'answer key start (no Gazebo odometry)'
            return
        # Median, not mean: a handful of samples taken before the EKF origin settles would drag
        # a mean by metres.
        dx = metrics.percentiles([s[0] for s in samples], (50,))['p50']
        dy = metrics.percentiles([s[1] for s in samples], (50,))['p50']
        self._offset = (dx, dy)
        self._offset_source = f'measured against Gazebo odometry ({len(samples)} samples)'

    def _truth_in_map(self, element) -> Tuple[float, float]:
        return (element.east - self._offset[0], element.north - self._offset[1])

    # -- callbacks --------------------------------------------------------------------------------

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _pose_callback(self, msg: BoatPoseMsg) -> None:
        with self._lock:
            self._pose = (float(msg.east), float(msg.north), float(msg.heading_rad))
        # Closest approach is tracked in the answer-key frame using the current offset estimate.
        for e in self._truth:
            te, tn = self._truth_in_map(e)
            d = math.hypot(te - msg.east, tn - msg.north)
            if d < self._closest_approach[e.element_id]:
                self._closest_approach[e.element_id] = d

    def _fused_callback(self, msg: FusedBuoyArray) -> None:
        """Input health, and the per-detection records the confusion fit needs.

        Matching raw detections to truth here (rather than in the scorer) is what lets us count
        "CV said green while the thing really was red" without the scorer ever seeing the key.
        """
        with self._lock:
            pose = self._pose
        self._fused_msgs += 1
        if pose is None:
            return
        now = self._now()
        east0, north0, yaw = pose

        # Decide once per frame, not once per detection: otherwise the first buoy in a frame
        # trips the throttle and the rest of that frame is dropped, biasing the log toward
        # whatever the clustering happened to order first.
        period = float(self.get_parameter('jsonl_period_s').value)
        log_frame = (now - self._last_detection_jsonl) >= period
        if log_frame:
            self._last_detection_jsonl = now

        for b in msg.buoys:
            self._fused_detections += 1
            rng = float(b.range)
            bearing = float(b.bearing)
            if not (math.isfinite(rng) and math.isfinite(bearing)) or rng <= 0.0:
                continue
            alpha = math.atan2(math.sin(yaw + bearing), math.cos(yaw + bearing))
            east = east0 + rng * math.cos(alpha)
            north = north0 + rng * math.sin(alpha)

            cid = int(b.class_id)
            cid = cid if 0 <= cid <= 22 else UNKNOWN_ID
            score = float(getattr(b, 'class_confidence', 0.0) or 0.0)
            if cid != UNKNOWN_ID:
                self._fused_with_class += 1
                self._fused_scores.append(score)

            cv_age = None
            stamp = getattr(b, 'class_stamp', None)
            if stamp is not None and (stamp.sec or stamp.nanosec):
                cv_age = max(0.0, now - (stamp.sec + stamp.nanosec * 1e-9))
                self._cv_ages.append(cv_age)
                if cv_age > 0.75:
                    self._stale_cv += 1

            # Nearest truth element, for the confusion tally. Deliberately a plain nearest-
            # neighbour at the match gate: a detection with no truth nearby is a clutter return
            # and has no "true class" to tally against.
            #
            # Point-like elements only. A banner or a dock has a *nominal anchor*, not an object
            # centre, so a buoy detection can land within the gate of one and get filed under its
            # class -- which teaches the fitted confusion matrix that e.g. digit_1 is never
            # observed correctly, when in truth it was never observed at all.
            best, best_d = None, self._match_m
            for e in self._point_truth:
                te, tn = self._truth_in_map(e)
                d = math.hypot(te - east, tn - north)
                if d < best_d:
                    best, best_d = e, d

            if best is not None and cid != UNKNOWN_ID:
                row = self._confusion_counts.setdefault(best.label.canonical_class_id, {})
                row[cid] = row.get(cid, 0.0) + 1.0

            if log_frame:
                self._write_jsonl({
                    'type': 'detection', 't': now, 'track_id': int(b.id),
                    'east': east, 'north': north, 'range_m': rng,
                    'class_id': cid, 'class_score': score, 'cv_age_s': cv_age,
                    'truth_id': best.element_id if best else None,
                    'truth_class_id': best.label.canonical_class_id if best else None,
                    'truth_distance_m': best_d if best else None,
                })

    def _scored_callback(self, msg: ScoredObjectArray) -> None:
        now = self._now()
        if self._first_stamp is None:
            self._first_stamp = now
        self._last_stamp = now
        self._scored_msgs += 1

        snapshot = []
        for o in msg.objects:
            snapshot.append({
                'oid': int(o.oid), 'east': float(o.east), 'north': float(o.north),
                'position_cov': [float(v) for v in o.position_cov],
                'p_exist': float(o.p_exist), 'class_id': int(o.class_id),
                'class_name': str(o.class_name), 'class_prob': float(o.class_prob),
                'class_entropy': float(o.class_entropy),
                # Take the scorer's marginal colour when it is on the wire. Deriving colour from
                # the class argmax instead throws away exactly the cases the marginal exists for
                # (a red cross reads colourless), and would score the scorer on a worse answer
                # than the one it actually computed. getattr keeps older bags readable.
                'colour': (str(getattr(o, 'colour', '')) or None),
                'colour_prob': float(getattr(o, 'colour_prob', float('nan'))),
                'observation_count': int(o.observation_count), 'track_id': int(o.track_id),
            })
            self._class_sequences.setdefault(int(o.oid), []).append(int(o.class_id))
            if o.track_id:
                self._track_ids.setdefault(int(o.oid), set()).add(int(o.track_id))

        with self._lock:
            self._last_snapshot = snapshot

        if now - self._last_frame_jsonl >= float(self.get_parameter('jsonl_period_s').value):
            self._last_frame_jsonl = now
            with self._lock:
                pose = self._pose
            self._write_jsonl({'type': 'frame', 't': now,
                               'boat': {'east': pose[0], 'north': pose[1], 'yaw': pose[2]}
                               if pose else None,
                               'objects': snapshot})

    def _write_jsonl(self, record: Dict) -> None:
        try:
            self._jsonl.write(json.dumps(record, default=str) + '\n')
        except Exception:
            pass

    def _log_status(self) -> None:
        self._resolve_offset()
        with self._lock:
            n = len(self._last_snapshot)
        expected = self._expected_offset
        drift = math.hypot(self._offset[0] - expected[0], self._offset[1] - expected[1])
        warn = '  <-- DISAGREES WITH THE ANSWER KEY' if drift > 2.0 else ''
        self.get_logger().info(
            f'objects={n} scored_msgs={self._scored_msgs} fused_msgs={self._fused_msgs} '
            f'detections={self._fused_detections} with_class={self._fused_with_class} '
            f'offset=({self._offset[0]:.1f}, {self._offset[1]:.1f}) '
            f'expected=({expected[0]:.1f}, {expected[1]:.1f}){warn}')

    # -- report ------------------------------------------------------------------------------------

    def build_report(self) -> Dict:
        self._resolve_offset()
        with self._lock:
            snapshot = list(self._last_snapshot)

        observable = [e for e in self._truth
                      if self._closest_approach[e.element_id] <= self._opportunity_m]
        truth_xy = [self._truth_in_map(e) for e in observable]

        scored_xy = [(o['east'], o['north']) for o in snapshot]
        scored_p = [o['p_exist'] for o in snapshot]

        match = metrics.match_to_truth(scored_xy, truth_xy, self._match_m)
        sweep = metrics.precision_recall_sweep(scored_xy, scored_p, truth_xy,
                                               max_distance_m=self._match_m)

        # Calibration: every published object is a claim that something is there, and the
        # outcome is whether it matched a truth element at the final frame.
        matched_scored = {i for i, _ in match.matches}
        outcomes = [1 if i in matched_scored else 0 for i in range(len(snapshot))]

        # Position + class, over matched pairs only.
        errors, nees_values, class_outcomes = [], [], []
        sigma_minors: List[float] = []
        floor = float(self.get_parameter('covariance_floor_m').value)
        per_object = []
        for si, ti in match.matches:
            o = snapshot[si]
            e = observable[ti]
            te, tn = self._truth_in_map(e)
            de, dn = o['east'] - te, o['north'] - tn
            err = math.hypot(de, dn)
            errors.append(err)
            nv = metrics.nees(de, dn, o['position_cov'])
            nees_values.append(nv)
            sigma_minors.append(_sigma_minor(o['position_cov']))

            # Prefer the marginal; fall back to the argmax class's colour only for bags recorded
            # before ScoredObject carried one.
            predicted_colour = o.get('colour') or class_colour(o['class_id'])
            class_outcomes.append(metrics.ClassOutcome(
                predicted_id=o['class_id'],
                predicted_prob=o['class_prob'],
                entropy=o['class_entropy'],
                truth_canonical_id=e.label.canonical_class_id,
                acceptable_ids=frozenset(e.label.acceptable_class_ids),
                predicted_colour=predicted_colour,
                truth_colour=e.colour,
                # Without the full posterior on the wire we can only bound the mass on the true
                # class: it is class_prob when the argmax is acceptable, and otherwise at most
                # (1 - class_prob). Recorded as a bound, not a measurement.
                p_true_class=(o['class_prob'] if o['class_id'] in e.label.acceptable_class_ids
                              else max(1e-6, 1.0 - o['class_prob'])),
                colour_expressible=colour_is_expressible(
                    e.colour, e.label.canonical_class_id),
            ))
            per_object.append({
                'oid': o['oid'], 'truth_id': e.element_id, 'truth_name': e.name,
                'position_error_m': err, 'nees': nv, 'p_exist': o['p_exist'],
                'class_id': o['class_id'], 'class_name': o['class_name'],
                'truth_class_id': e.label.canonical_class_id,
                'class_correct': o['class_id'] in e.label.acceptable_class_ids,
                'colour': predicted_colour, 'truth_colour': e.colour,
                'class_entropy': o['class_entropy'],
                'track_id_churn': len(self._track_ids.get(o['oid'], ())) or 1,
            })

        detections = max(1, self._fused_detections)
        health = {
            'fused_messages': self._fused_msgs,
            'detections': self._fused_detections,
            'detections_with_class': self._fused_with_class,
            'class_coverage': self._fused_with_class / detections,
            'mean_class_score': (sum(self._fused_scores) / len(self._fused_scores)
                                 if self._fused_scores else float('nan')),
            'max_class_score': max(self._fused_scores) if self._fused_scores else float('nan'),
            'cv_age_p50_s': metrics.percentiles(self._cv_ages, (50,))['p50'],
            'cv_age_p95_s': metrics.percentiles(self._cv_ages, (95,))['p95'],
            'stale_cv_fraction': self._stale_cv / detections,
            'scored_messages': self._scored_msgs,
            'duration_s': ((self._last_stamp - self._first_stamp)
                           if (self._first_stamp and self._last_stamp) else 0.0),
        }
        health['verdict'] = self._health_verdict(health)

        return {
            'answer_key': self._key_path,
            'seed': self._key.get('seed'),
            'world': self._key.get('world'),
            'input_health': health,
            'frame_offset': {
                'estimated': list(self._offset),
                'expected': list(self._expected_offset),
                'source': self._offset_source,
                'disagreement_m': math.hypot(self._offset[0] - self._expected_offset[0],
                                             self._offset[1] - self._expected_offset[1]),
            },
            'opportunity': {
                'range_m': self._opportunity_m,
                'truth_total': len(self._truth),
                'truth_observable': len(observable),
                'note': 'recall is computed against observable elements only; '
                        'the rest measure where the boat drove, not what it saw',
            },
            'existence': {
                'objects_published': len(snapshot),
                'matched': len(match.matches),
                'false_positives': len(match.unmatched_scored),
                'missed': len(match.unmatched_truth),
                'pr_sweep': [p.to_dict() for p in sweep],
            },
            'calibration': {
                'brier': metrics.brier_score(scored_p, outcomes),
                'ece': metrics.expected_calibration_error(scored_p, outcomes),
                'bins': [b.to_dict() for b in metrics.reliability_bins(scored_p, outcomes)],
            },
            'position': {
                'count': len(errors),
                'error_m': metrics.percentiles(errors, (50, 90, 95)),
                'mean_error_m': sum(errors) / len(errors) if errors else float('nan'),
                'nees': metrics.nees_summary(nees_values),
                'sigma_minor_m': metrics.percentiles(sigma_minors, (50,)),
                'covariance_floor_m': floor,
                # A conservative NEES means one of two very different things: the measurement
                # model is pessimistic, or the covariance floor is simply larger than the real
                # error. Only the first is worth tuning, so say which one it is.
                'floor_bound_frac': (
                    sum(1 for s in sigma_minors if s <= floor * 1.01) / len(sigma_minors)
                    if sigma_minors else float('nan')),
            },
            'classification': metrics.class_summary(class_outcomes),
            'stability': {
                'flicker': metrics.flicker_rate(self._class_sequences),
                'churn': metrics.churn_summary([len(s) for s in self._track_ids.values()]),
            },
            'confusion_counts': {str(k): {str(kk): vv for kk, vv in v.items()}
                                 for k, v in self._confusion_counts.items()},
            'objects': per_object,
            'unmatched_truth': [observable[j].name for j in match.unmatched_truth],
        }

    @staticmethod
    def _health_verdict(health: Dict) -> str:
        if health['scored_messages'] == 0:
            return ('NO DATA: the scorer published nothing. Check /fused_buoys and /boat_pose '
                    'before reading anything below.')
        if health['detections'] == 0:
            return 'NO DETECTIONS: LiDAR produced no tracks. Nothing below is meaningful.'
        if health['class_coverage'] < 0.05:
            return ('CV CONTRIBUTED ALMOST NOTHING ({:.1%} of detections carried a class). Every '
                    'class metric below is uninterpretable; a high entropy here is the scorer '
                    'being honest, not the scorer failing.'.format(health['class_coverage']))
        if health['stale_cv_fraction'] > 0.5:
            return ('CV IS MOSTLY STALE ({:.0%} of detections carried class data older than '
                    '0.75 s). Class metrics are measuring latency as much as accuracy.'
                    .format(health['stale_cv_fraction']))
        return 'inputs look usable'

    def _markdown(self, r: Dict) -> str:
        h = r['input_health']
        fo = r['frame_offset']
        L = [f'# Truth scorer run report: seed {r["seed"]}', '',
             f'- Answer key: `{r["answer_key"]}`',
             f'- Sim duration: {h["duration_s"]:.0f} s, {h["scored_messages"]} scored frames',
             '', '## 1. Input health', '',
             f'**{h["verdict"]}**', '',
             '| Signal | Value |', '|---|---|',
             f'| LiDAR detections | {h["detections"]} over {h["fused_messages"]} frames |',
             f'| Carrying a CV class | {h["detections_with_class"]} '
             f'({h["class_coverage"]:.1%}) |',
             f'| Mean / max CV score | {h["mean_class_score"]:.3f} / {h["max_class_score"]:.3f} |',
             f'| CV age p50 / p95 | {h["cv_age_p50_s"]:.2f} s / {h["cv_age_p95_s"]:.2f} s |',
             f'| Stale CV (>0.75 s) | {h["stale_cv_fraction"]:.1%} |',
             '', '## 2. Frame offset', '',
             f'- Estimated: ({fo["estimated"][0]:.2f}, {fo["estimated"][1]:.2f}) '
             f'-- {fo["source"]}',
             f'- Expected from the answer key: ({fo["expected"][0]:.2f}, '
             f'{fo["expected"][1]:.2f})',
             f'- Disagreement: **{fo["disagreement_m"]:.2f} m**'
             + ('  <-- every metric below is suspect' if fo['disagreement_m'] > 2.0 else ''),
             '']

        o = r['opportunity']
        e = r['existence']
        L += ['## 3. Existence', '',
              f'{o["truth_observable"]} of {o["truth_total"]} truth elements came within '
              f'{o["range_m"]:.0f} m of the boat. {o["note"]}.', '',
              f'- Published objects: {e["objects_published"]}, matched {e["matched"]}, '
              f'false positives {e["false_positives"]}, missed {e["missed"]}',
              '', '| Threshold | Precision | Recall | F1 | TP | FP | FN |', '|---|---|---|---|---|---|---|']
        for p in e['pr_sweep']:
            L.append(f'| {p["threshold"]:.1f} | {p["precision"]:.2f} | {p["recall"]:.2f} | '
                     f'{p["f1"]:.2f} | {p["tp"]} | {p["fp"]} | {p["fn"]} |')

        c = r['calibration']
        L += ['', '## 4. Calibration', '',
              'When it says 0.8, is it right 80 % of the time?', '',
              f'- **ECE: {c["ece"]:.3f}**, Brier: {c["brier"]:.3f}', '',
              '| p_exist bin | n | claimed | observed |', '|---|---|---|---|']
        for b in c['bins']:
            if b['count'] == 0:
                continue
            L.append(f'| {b["low"]:.1f}-{b["high"]:.1f} | {b["count"]} | '
                     f'{b["mean_predicted"]:.2f} | {b["observed_rate"]:.2f} |')

        p = r['position']
        n = p['nees']
        L += ['', '## 5. Position', '',
              f'- Error over {p["count"]} matched objects: '
              f'p50 {p["error_m"]["p50"]:.2f} m, p90 {p["error_m"]["p90"]:.2f} m, '
              f'p95 {p["error_m"]["p95"]:.2f} m',
              f'- **NEES mean {n["mean"]:.2f}** (median {n["median"]:.2f}, n={n["count"]}) '
              f'-- expected ~2.0 for an honest 2-D covariance',
              f'- Verdict: **{n["verdict"]}**']
        fb = p.get('floor_bound_frac', float('nan'))
        if fb == fb and fb > 0.5 and n['verdict'].startswith('conservative'):
            L.append(
                f'- Caveat: {fb:.0%} of matched objects sit at the {p["covariance_floor_m"]:.2f} m '
                f'covariance floor, so this NEES measures the floor, not the measurement model. '
                f'Lower the floor only if you believe the GPS bias it stands in for is absent.')
        elif fb == fb:
            L.append(f'- sigma_minor p50 {p["sigma_minor_m"]["p50"]:.2f} m '
                     f'({fb:.0%} at the {p["covariance_floor_m"]:.2f} m floor)')
        L.append('')

        cl = r['classification']
        L += ['## 6. Classification', '',
              f'- Top-1 (colour-equivalence group accepted): {cl["top1"]:.1%} over '
              f'{cl["count"]} objects',
              f'- **Colour accuracy: {cl["colour_accuracy"]:.1%}** over {cl["colour_count"]} '
              'coloured elements -- this is the one the handbook actually scores',
              f'- Mean NLL of the true class: {cl["mean_nll"]:.3f} '
              '(the only metric here that rewards saying "I do not know")',
              f'- Mean class entropy: {cl["mean_entropy"]:.3f}']
        inx = cl.get('colour_inexpressible_count', 0)
        if inx:
            L.append(
                f'- Excluded from colour accuracy: {inx} coloured element(s) whose *canonical* '
                'class is colourless in `class_mapping.yaml`. These are the Task 5 dock '
                'indicators: a red cross is scored red by the handbook, but the right answer '
                'for the detector is `cross`, which carries no colour. A detector doing its job '
                'perfectly still reports no colour here, so counting these would drag colour '
                'accuracy down *and* pin it -- no CV improvement could ever move them. '
                'This is a real capability gap, and closing it means adding coloured placard '
                'classes (red_cross, green_triangle, ...) to the CV vocabulary.')
        L.append('')

        s = r['stability']
        L += ['## 7. Stability', '',
              f'- Class flicker: {s["flicker"]["mean_per_object"]:.3f} per object per frame '
              f'({s["flicker"]["changes"]} changes over {s["flicker"]["transitions"]} '
              'transitions)',
              f'- Track-id churn: mean {s["churn"]["mean"]:.2f}, max {s["churn"]["max"]}, '
              f'{s["churn"]["objects_with_churn"]} objects absorbed more than one LiDAR track',
              '  (each of those is an object a track-id-keyed map would have thrown away)', '']

        if r['unmatched_truth']:
            L += ['## Missed truth elements', ''] + \
                 [f'- {name}' for name in r['unmatched_truth']] + ['']
        return '\n'.join(L)

    def write_report(self) -> Optional[str]:
        try:
            report = self.build_report()
        except Exception as exc:          # a crash here must not eat the JSONL
            self.get_logger().error(f'could not build report: {exc}')
            try:
                self._jsonl.close()
            except Exception:
                pass
            return None
        # The offset is measured during the run, so it can only be logged at the end. Without
        # it, analyse_run.py would silently fall back to the answer key's assumed start and
        # could place every truth element a metre or two off.
        self._write_jsonl({'type': 'offset', **report['frame_offset']})
        with open(self._stem + '.json', 'w', encoding='utf-8') as handle:
            json.dump(report, handle, indent=1, default=str)
        with open(self._stem + '.md', 'w', encoding='utf-8') as handle:
            handle.write(self._markdown(report))
        try:
            self._jsonl.close()
        except Exception:
            pass
        return self._stem


def main(args=None):
    rclpy.init(args=args)
    node = TruthEvalNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        stem = node.write_report()
        if stem:
            print(f'\nTruth scorer report: {stem}.md '
                  f'(details in {stem}.json, raw records in {stem}.jsonl)', flush=True)
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
