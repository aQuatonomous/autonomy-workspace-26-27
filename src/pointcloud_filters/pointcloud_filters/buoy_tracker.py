#!/usr/bin/env python3
"""
Buoy Tracker Node

Maintains persistent tracking of detected buoys with stable IDs across frames.
Performs data association to match new detections with existing tracked buoys.
Color is not set by LiDAR; the fusion node assigns color from CV and publishes /fused_buoys.

Subscribes to:
    /buoy_detections (pointcloud_filters/BuoyDetectionArray): Raw detections from DBSCAN

Publishes:
    /tracked_buoys (pointcloud_filters/TrackedBuoyArray): Buoys with persistent IDs (no color)
    /tracked_buoys_json (std_msgs/String): Same content as JSON so downstream/fusion can use LiDAR like CV
"""

import json
import math
from typing import Dict, Set, Tuple, List, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.time import Time
from std_msgs.msg import String

from pointcloud_filters.msg import BuoyDetectionArray, BuoyDetection
from pointcloud_filters.msg import TrackedBuoyArray, TrackedBuoy


def _time_to_float(t) -> float:
    """Convert builtin_interfaces/Time to seconds (for JSON)."""
    return float(t.sec) + float(t.nanosec) * 1e-9


class BuoyTracker(Node):
    """
    Tracks buoys across frames with persistent IDs using nearest-neighbor data association.
    
    Maintains a hashmap of tracked buoys, updating positions when re-detected and
    removing buoys that haven't been seen for several frames.
    """
    
    def __init__(self):
        super().__init__('buoy_tracker')
        
        # Parameters
        self.declare_parameter('association_threshold', 0.76)  # Max distance (m) to match detection to track/candidate
        self.declare_parameter('max_consecutive_misses', 10)   # Frames before removing a tracked buoy
        self.declare_parameter('position_alpha', 0.7)          # Exponential smoothing (0=old, 1=new)
        self.declare_parameter('min_observations_for_publish', 3)  # Observations before a track is published
        self.declare_parameter('min_observations_to_add', 5)
        self.declare_parameter(
            'candidate_max_consecutive_misses', 4,
        )

        self.association_threshold = self.get_parameter('association_threshold').value
        self.max_consecutive_misses = self.get_parameter('max_consecutive_misses').value
        self.position_alpha = self.get_parameter('position_alpha').value
        self.min_observations = self.get_parameter('min_observations_for_publish').value
        self.min_observations_to_add = int(self.get_parameter('min_observations_to_add').value)
        self.candidate_max_misses = int(self.get_parameter('candidate_max_consecutive_misses').value)

        # Tracked buoys: {id: buoy_data_dict}
        self.tracked_buoys: Dict[int, dict] = {}
        self.next_id = 0
        # Candidates: must be seen min_observations_to_add times in same area before becoming a track
        self.candidates: List[dict] = []
        
        # QoS profile: depth=1 so we always process latest frame (low latency)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        
        # Subscriber to raw detections
        self.detection_sub = self.create_subscription(
            BuoyDetectionArray,
            '/buoy_detections',
            self.detection_callback,
            qos
        )
        
        # Publisher for tracked buoys
        self.tracked_pub = self.create_publisher(
            TrackedBuoyArray,
            '/tracked_buoys',
            qos
        )
        # JSON publisher (same content as TrackedBuoyArray, for fusion/downstream in CV-like format)
        self.json_pub = self.create_publisher(String, '/tracked_buoys_json', qos)

        self.get_logger().info('Buoy tracker started (publishes /tracked_buoys and /tracked_buoys_json)')
        self.get_logger().info(
            f'Association threshold: {self.association_threshold:.2f}m, '
            f'Max misses: {self.max_consecutive_misses}, '
            f'Min observations to add track: {self.min_observations_to_add}, '
            f'Candidate max misses: {self.candidate_max_misses}'
        )
    
    def detection_callback(self, msg: BuoyDetectionArray):
        """
        Process new detections: associate with tracked buoys, update existing, add new, remove lost.
        """
        header = msg.header
        try:
            # Convert detections to list with Cartesian coordinates
            new_detections = []
            for det in msg.detections:
                x = det.range * math.cos(det.bearing)
                y = det.range * math.sin(det.bearing)
                new_detections.append({
                    'range': det.range,
                    'bearing': det.bearing,
                    'x': x,
                    'y': y,
                    'z_mean': det.z_mean,
                    'confidence': det.confidence,
                    'num_points': det.num_points,
                    'detection': det  # Keep original
                })

            # Data association with tracked buoys
            matches, unmatched_detections, unmatched_buoys = self.associate_detections(new_detections)
            current_time = Time.from_msg(header.stamp)

            # Update matched tracked buoys
            for buoy_id, det_idx in matches.items():
                self.update_buoy(buoy_id, new_detections[det_idx], current_time)

            # Associate unmatched detections to candidates (probation); promote or add candidates
            unmatched_after_candidates = self.update_candidates(
                new_detections, unmatched_detections, current_time
            )

            # Increment miss counter for tracked buoys that were not matched
            for buoy_id in unmatched_buoys:
                self.tracked_buoys[buoy_id]['consecutive_misses'] += 1

            # Remove tracked buoys with too many consecutive misses
            to_remove = [
                bid for bid, buoy in self.tracked_buoys.items()
                if buoy['consecutive_misses'] > self.max_consecutive_misses
            ]
            for bid in to_remove:
                self.get_logger().info(f'Removing buoy {bid} (not seen for {self.max_consecutive_misses} frames)')
                del self.tracked_buoys[bid]

            # Log summary (no spam: only when something notable)
            num_promoted = len(self.tracked_buoys) - (len(matches) + len(to_remove)) + len(unmatched_buoys)
            if unmatched_after_candidates or to_remove or num_promoted:
                self.get_logger().info(
                    f'Tracking: {len(self.tracked_buoys)} buoys | '
                    f'Matched: {len(matches)} | Candidates: {len(self.candidates)} | '
                    f'New candidates: {len(unmatched_after_candidates)} | Lost: {len(to_remove)}'
                )
        except Exception as e:
            self.get_logger().error(f'Tracker callback error: {e}', throttle_duration_sec=5.0)

        # Always publish (even on exception) so subscribers never hang waiting for first message
        self.publish_tracked_buoys(header)
    
    def associate_detections(self, new_detections: List[dict]) -> Tuple[Dict[int, int], Set[int], Set[int]]:
        """
        Associate new detections with tracked buoys using nearest-neighbor matching.
        
        Returns:
            matches: {buoy_id: detection_index}
            unmatched_detections: set of detection indices
            unmatched_buoys: set of buoy IDs
        """
        matches = {}
        unmatched_detections = set(range(len(new_detections)))
        unmatched_buoys = set(self.tracked_buoys.keys())
        
        # Build distance matrix and find best matches
        # Simple greedy nearest-neighbor (could upgrade to Hungarian algorithm)
        for buoy_id, buoy in self.tracked_buoys.items():
            best_dist = float('inf')
            best_idx = None
            
            for det_idx in list(unmatched_detections):
                detection = new_detections[det_idx]
                dist = self.compute_distance_3d(buoy, detection)
                
                if dist < best_dist and dist < self.association_threshold:
                    best_dist = dist
                    best_idx = det_idx
            
            if best_idx is not None:
                matches[buoy_id] = best_idx
                unmatched_detections.remove(best_idx)
                unmatched_buoys.remove(buoy_id)
        
        return matches, unmatched_detections, unmatched_buoys
    
    def compute_distance_3d(self, buoy: dict, detection: dict) -> float:
        """
        Compute 3D Euclidean distance between tracked buoy/candidate and detection.
        Uses (x, y, range) to distinguish buoys at same bearing but different distances.
        """
        dx = buoy['x'] - detection['x']
        dy = buoy['y'] - detection['y']
        dr = buoy['range'] - detection['range']
        return math.sqrt(dx*dx + dy*dy + dr*dr)

    def update_candidates(
        self,
        new_detections: List[dict],
        unmatched_detection_indices: Set[int],
        current_time: Time,
    ) -> Set[int]:
        """
        Associate unmatched detections to candidates; promote candidates that have been
        seen enough times, drop stale candidates, add new candidates for still-unmatched detections.
        Returns the set of detection indices that were added as new candidates.
        """
        unmatched_dets = set(unmatched_detection_indices)
        # Match unmatched detections to candidates (greedy nearest-neighbor)
        cand_matches: Dict[int, int] = {}  # candidate_index -> det_idx
        for ci, cand in enumerate(self.candidates):
            best_dist = float('inf')
            best_idx = None
            for det_idx in list(unmatched_dets):
                dist = self.compute_distance_3d(cand, new_detections[det_idx])
                if dist < best_dist and dist < self.association_threshold:
                    best_dist = dist
                    best_idx = det_idx
            if best_idx is not None:
                cand_matches[ci] = best_idx
                unmatched_dets.discard(best_idx)

        to_remove_candidates: Set[int] = set()
        # Update matched candidates; promote to track if observation_count >= min_observations_to_add
        for ci, det_idx in cand_matches.items():
            det = new_detections[det_idx]
            cand = self.candidates[ci]
            alpha = self.position_alpha
            cand['x'] = alpha * det['x'] + (1 - alpha) * cand['x']
            cand['y'] = alpha * det['y'] + (1 - alpha) * cand['y']
            cand['z_mean'] = alpha * det['z_mean'] + (1 - alpha) * cand['z_mean']
            cand['range'] = math.sqrt(cand['x']**2 + cand['y']**2)
            cand['bearing'] = math.atan2(cand['y'], cand['x'])
            cand['observation_count'] += 1
            cand['consecutive_misses'] = 0
            cand['last_seen'] = current_time
            cand['confidence'] = max(cand['confidence'], det['confidence'])
            if cand['observation_count'] >= self.min_observations_to_add:
                self.add_new_buoy_from_candidate(cand, current_time)
                to_remove_candidates.add(ci)

        # Unmatched candidates: increment misses, mark for removal if over limit
        for ci, cand in enumerate(self.candidates):
            if ci in cand_matches:
                continue
            cand['consecutive_misses'] += 1
            if cand['consecutive_misses'] > self.candidate_max_misses:
                to_remove_candidates.add(ci)

        self.candidates = [c for i, c in enumerate(self.candidates) if i not in to_remove_candidates]

        # Add new candidates for detections that didn't match any candidate
        for det_idx in unmatched_dets:
            self.candidates.append(self._make_candidate(new_detections[det_idx], current_time))

        return unmatched_dets

    def _make_candidate(self, detection: dict, current_time: Time) -> dict:
        """Create a candidate dict from a detection (probation phase)."""
        return {
            'x': detection['x'],
            'y': detection['y'],
            'range': detection['range'],
            'bearing': detection['bearing'],
            'z_mean': detection['z_mean'],
            'confidence': detection['confidence'],
            'observation_count': 1,
            'consecutive_misses': 0,
            'first_seen': current_time,
            'last_seen': current_time,
        }

    def add_new_buoy_from_candidate(self, cand: dict, current_time: Time) -> None:
        """Promote a candidate to a full tracked buoy (seen enough times in same area)."""
        buoy_id = self.next_id
        self.next_id += 1
        self.tracked_buoys[buoy_id] = {
            'id': buoy_id,
            'range': cand['range'],
            'bearing': cand['bearing'],
            'x': cand['x'],
            'y': cand['y'],
            'z_mean': cand['z_mean'],
            'confidence': cand['confidence'],
            'observation_count': cand['observation_count'],
            'first_seen': cand['first_seen'],
            'last_seen': current_time,
            'consecutive_misses': 0,
        }
        self.get_logger().info(
            f'New buoy {buoy_id} (promoted after {cand["observation_count"]} observations) '
            f'at range={cand["range"]:.2f}m, bearing={math.degrees(cand["bearing"]):.1f}°'
        )
    
    def update_buoy(self, buoy_id: int, detection: dict, current_time: Time):
        """
        Update existing buoy with new detection using exponential smoothing for position.
        """
        buoy = self.tracked_buoys[buoy_id]
        alpha = self.position_alpha
        
        # Smooth position (exponential moving average)
        buoy['x'] = alpha * detection['x'] + (1 - alpha) * buoy['x']
        buoy['y'] = alpha * detection['y'] + (1 - alpha) * buoy['y']
        buoy['z_mean'] = alpha * detection['z_mean'] + (1 - alpha) * buoy['z_mean']
        
        # Recompute polar coordinates from smoothed Cartesian
        buoy['range'] = math.sqrt(buoy['x']**2 + buoy['y']**2)
        buoy['bearing'] = math.atan2(buoy['y'], buoy['x'])
        
        # Update confidence (take max or average)
        buoy['confidence'] = max(buoy['confidence'], detection['confidence'])
        
        # Update timestamps and counters
        buoy['last_seen'] = current_time
        buoy['observation_count'] += 1
        buoy['consecutive_misses'] = 0  # Reset miss counter
    
    def publish_tracked_buoys(self, header):
        """
        Publish tracked buoys that meet minimum observation threshold.
        Always publishes at least an empty message so subscribers never hang.
        """
        msg = TrackedBuoyArray()
        msg.header = header
        msg.header.stamp = self.get_clock().now().to_msg()

        for buoy_id, buoy in self.tracked_buoys.items():
            if buoy['observation_count'] < self.min_observations:
                continue
            tracked_msg = TrackedBuoy()
            tracked_msg.id = buoy['id']
            tracked_msg.range = buoy['range']
            tracked_msg.bearing = buoy['bearing']
            tracked_msg.x = buoy['x']
            tracked_msg.y = buoy['y']
            tracked_msg.z_mean = buoy['z_mean']
            tracked_msg.confidence = buoy['confidence']
            tracked_msg.observation_count = buoy['observation_count']
            try:
                tracked_msg.first_seen = buoy['first_seen'].to_msg()
                tracked_msg.last_seen = buoy['last_seen'].to_msg()
            except (AttributeError, TypeError) as e:
                self.get_logger().warn(f'Skipping buoy {buoy_id}: invalid first_seen/last_seen ({e})', throttle_duration_sec=5.0)
                continue
            msg.buoys.append(tracked_msg)

        self.tracked_pub.publish(msg)

        # Publish same content as JSON (format similar to CV for fusion/downstream)
        detections = []
        for b in msg.buoys:
            detections.append({
                'id': b.id,
                'range': b.range,
                'bearing': b.bearing,
                'x': b.x,
                'y': b.y,
                'z_mean': b.z_mean,
                'confidence': b.confidence,
                'observation_count': b.observation_count,
                'first_seen': _time_to_float(b.first_seen),
                'last_seen': _time_to_float(b.last_seen),
            })
        payload = {
            'timestamp': _time_to_float(msg.header.stamp) if msg.header.stamp else 0.0,
            'frame_id': msg.header.frame_id or 'base_link',
            'num_detections': len(detections),
            'detections': detections,
        }
        json_msg = String()
        json_msg.data = json.dumps(payload)
        self.json_pub.publish(json_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BuoyTracker()
    
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
