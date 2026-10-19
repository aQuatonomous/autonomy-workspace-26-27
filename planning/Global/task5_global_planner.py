"""
Global Planner Task 5: Autonomous Docking

Manages high-level docking mission state machine and dock selection.
Uses DockingLocalPlanner for low-level motion control.
"""

# Coordinate Frame Assumptions: Assume East is +x, North is +y, and North is 0rad heading and heading is between 0 and 2pi rad.

#TODONE Fix bugs coming from using dist_north and dist_south to stay centred in the lane. This will fail if the dock on one side is occupyed. Also our boat can't go sideways
# Fixed by making the local planner not center if the difference between dist_north and dist_sounth is large. Also used this to determine if docks are vacant

from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Dict, List, Optional, Tuple, Set
from statistics import mean
import time
import numpy as np

from docking_local_planner import (
    DockingLocalPlanner, 
    DockingState, 
    ControlMode,
    SafetyStatus,
    DOCKING_TARGET_DISTANCE  # Import constant
)

Vec2 = Tuple[float, float]
Vec3 = Tuple[float, float, float]
Pose = Tuple[float, float, float]


@dataclass
class DockData:
    """Complete information about a detected dock"""
    dock_id: int
    side: str  # 'north' or 'south'
    
    # Visual indicators (from vision system)
    indicator_color: str = 'unknown'  # 'red', 'green', 'unknown'
    number: int = -1  # 1, 2, 3, or -1 if unknown
    vacant: bool = True  # Default assume vacant
    
    # Geometry (from LiDAR)
    start_pos: Vec2 = (0.0, 0.0)
    end_pos: Vec2 = (0.0, 0.0)
    width: float = 0.0
    length: float = 0.0
    
    # Confidence tracking
    detections: int = 1
    confidence: float = 0.5
    
    # Timestamps
    first_seen: Optional[float] = None
    last_seen: Optional[float] = None

    def is_valid_target(self) -> bool:
        """Check if dock meets selection criteria"""
        return (
            self.indicator_color == 'green' and
            self.vacant #and       I DONT CARE - IF I SAW A DOCK IM GOING IN
            #self.number > 0 and
            #self.confidence > 0.2 and # TODONE I don't care much about confidence
            #self.detections >= 1 # TODONE might want to just make this one
        )
    
    def center_point(self) -> Vec2:
        """Calculate center of dock entrance"""
        return (
            (self.start_pos[0] + self.end_pos[0]) / 2.0,
            (self.start_pos[1] + self.end_pos[1]) / 2.0
        )

# We don't use goals here
#@dataclass
#class GoalPlan:
#    """Individual goal with completion tracking"""
#    goal_id: str
#    position: Vec2
#    completed: bool = False
#    tolerance: float = 2.0


@dataclass
class DetectedEntity:
    """Entity from perception system"""
    entity_id: int
    entity_type: str
    position: Vec2


class TaskStatus(Enum):
    RUNNING = "RUNNING"
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"


class Phase(Enum):
    """Docking mission phases"""
    INITIAL_TURN = "INITIAL_TURN"
    APPROACH = "APPROACH"
    INITIAL_CENTERING = "INITIAL_CENTERING"
    SCANNING = "SCANNING"
    SELECTING = "SELECTING"
    RETURNING = "RETURNING"  # Return to start if no dock found
    TURNING = "TURNING"
    RECENTERING = "RECENTERING"
    APPROACHING = "APPROACHING"
    ROTATING_TO_DOCK = "ROTATING_TO_DOCK"
    DOCKING = "DOCKING"
    DOCKED = "DOCKED"
    ROTATING_IN_DOCK = "ROTATING_IN_DOCK"
    UNDOCKING = "UNDOCKING"
    DONE = "DONE"


class Task5Manager:
    def __init__(self, entities, map_bounds: Vec2, start_pose: Pose):
        self.entities = entities
        self.map_bounds = (float(map_bounds[0]), float(map_bounds[1]))
        self.start_pos: Vec2 = (float(start_pose[0]), float(start_pose[1]))
        self.pose: Pose = start_pose
        self.prev_pos: Optional[Vec2] = None
        
        #Velocities for Global Planner to Read (x,y,theta)
        self.current_velocities: Vec3 = (0.0, 0.0, 0.0)
   
        # Phase management
        self.phase = Phase.INITIAL_TURN
        
        # Dock tracking
        self.detected_docks: Dict[int, DockData] = {}
        self.next_dock_id = 0
        self.target_dock: Optional[DockData] = None
        
        # LiDAR distance tracking
        self.dist_north: Optional[float] = None
        self.dist_south: Optional[float] = None
        self.dist_forward: Optional[float] = None
        self.laser_scan: Optional[List[float]] = None
        self.prev_dist_north: Optional[float] = None
        self.prev_dist_south: Optional[float] = None
        
        # Scanning state
        self.scan_start_x: Optional[float] = None
        self.scan_distance: float = 0.0
        self.expected_scan_distance: float = 8.0  # meters (16 blocks * 20 inches/block * 2.5 cm/inch) TODONE meters
        
        # We don't use goals here
        # Goal management
        #self.goal_plan: List[GoalPlan] = []
        #self.goal_queue: List[Vec2] = []
        
        # Docking parameters
        self.target_dock_depth = 2.032   # meters, depth of dock #TODONE tune also never used 
        
        # Edge detection
        self.edge_threshold = 1.7  # meters (dock is 80 inchs deep)
        self.vacancy_threshold = 0.4 # meters (1 foot) arbityray choice needs tuned
        
        # Local planner for motion control
        self.local_planner = DockingLocalPlanner()
        
        # Target heading storage for rotation phases
        self.target_heading: Optional[float] = None
        
        # Mission timing
        self.phase_start_time: Optional[float] = None
        self.mission_start_time: float = time.time()
    
    # ========== UPDATE METHODS ==========
    
    def update_pose(self, x: float, y: float, heading: float) -> None:
        """Update boat position and heading"""
        if self.prev_pos is None:
            self.prev_pos = (self.pose[0], self.pose[1])
        else:
            self.prev_pos = (self.pose[0], self.pose[1])
        
        self.pose = (float(x), float(y), float(heading))
        
        # Update scan distance during scanning phase
        if self.phase == Phase.SCANNING and self.scan_start_x is not None:
            self.scan_distance = abs(x - self.scan_start_x) #TODONE meters
    
    # This makes the assumption that the array of distances is a180deg laser scan from the left of the boat to the right 
    def update_lidar_distances(self, distances: List[float]) -> None:
        """Update distances to North/South dock walls"""
        self.prev_dist_north = self.dist_north
        self.prev_dist_south = self.dist_south
        self.dist_north = distances[0] 
        self.dist_south = distances[-1] 
        self.dist_forward = distances[len(distances)//2]

        self.laser_scan = distances 
        
        # Detect edges when distances change significantly
        if self.phase == Phase.SCANNING:
            self._detect_dock_edges()
    
    def add_detected(self, det: DetectedEntity) -> None:
        """Add detected entity from perception"""
        # Add to entity list
        if hasattr(self.entities, "add_or_update"):
            self.entities.add_or_update(
                det.entity_type, 
                det.position,
                name=f"{det.entity_type}_{det.entity_id}"
            )
        else:
            self.entities.add(
                det.entity_type, 
                det.position,
                name=f"{det.entity_type}_{det.entity_id}"
            )
        
        # Match vision detections to docks
        if det.entity_type in ('red_indicator', 'green_indicator'):
            self._update_dock_color(det)
        elif det.entity_type == 'dock_number':
            self._update_dock_number(det)
        
    
    # ========== DOCK DETECTION ==========
    
    def _detect_dock_edges(self) -> None:
        """Detect dock edges from distance changes"""
        if (self.prev_dist_north is None or self.prev_dist_south is None or
            self.dist_north is None or self.dist_south is None):
            return
        
        # Check North side
        north_change = abs(self.dist_north - self.prev_dist_north)
        if north_change > self.edge_threshold:
            edge_type = 'end' if self.dist_north < self.prev_dist_north else 'start'
            self._process_edge('north', edge_type, (self.pose[0], self.pose[1] + self.dist_north)) # TODONE This was making the dock position in the center of the marina? And that will mess up assigning number/colour so I add 1 to the y coordinate so north docks are to the north TODONE meters
        elif north_change > self.vacancy_threshold:
            self._update_dock_vacancy('north')
        
        # Check South side
        south_change = abs(self.dist_south - self.prev_dist_south)
        if south_change > self.edge_threshold:
            edge_type = 'end' if self.dist_south < self.prev_dist_south else 'start'
            self._process_edge('south', edge_type, (self.pose[0], self.pose[1] - self.dist_south))
        elif south_change > self.vacancy_threshold:
            self._update_dock_vacancy('south')
    
    def _process_edge(self, side: str, edge_type: str, position: Vec2) -> None:
        """Process detected edge"""
        if edge_type == 'start':
            # New dock starting
            dock = DockData(
                dock_id=self.next_dock_id,
                side=side,
                start_pos=position,
                first_seen=time.time()
            )
            self.detected_docks[self.next_dock_id] = dock
            self.next_dock_id += 1
            print(f"[DOCK] New dock {dock.dock_id} started on {side} at {position}")
            
        elif edge_type == 'end':
            # Find matching dock to complete
            for dock in self.detected_docks.values():
                if dock.side == side and dock.end_pos == (0.0, 0.0):
                    dock.end_pos = position
                    dock.length = self._distance(dock.start_pos, dock.end_pos)
                    dock.last_seen = time.time()
                    print(f"[DOCK] Dock {dock.dock_id} completed: length={dock.length:.2f}m")
                    break
    
    def _update_dock_color(self, det: DetectedEntity) -> None:
        """Match color detection to nearest dock"""
        dock_id = self._find_nearest_dock(det.position)
        if dock_id is not None:
            color = 'green' if det.entity_type == 'green_indicator' else 'red'
            self.detected_docks[dock_id].indicator_color = color
            self.detected_docks[dock_id].detections += 1
            self.detected_docks[dock_id].confidence = min(1.0, 
                self.detected_docks[dock_id].confidence + 0.1)
    
    def _update_dock_number(self, det: DetectedEntity) -> None:
        """Match number detection to nearest dock"""
        dock_id = self._find_nearest_dock(det.position)
        if dock_id is not None:
            # Assume entity_id represents the number (1, 2, 3)
            self.detected_docks[dock_id].number = det.entity_id
            self.detected_docks[dock_id].detections += 1
    
    #TODONE This is called by update_dock_edges_ if it sees an edge but not one big enough to be the end of the dock
    def _update_dock_vacancy(self, side: str) -> None:
        """Update dock vacancy status"""
        for dock in self.detected_docks.values():
            if dock.side == side and dock.end_pos == (0.0, 0.0):
                dock.vacant = False
    
    # Used to associate detections to docks so it uses the docks start instead of center becuase it may not have an end
    def _find_nearest_dock(self, position: Vec2) -> Optional[int]:
        """Find dock closest to detection position"""
        if not self.detected_docks:
            return None
        
        min_dist = float('inf')
        nearest_id = None
        
        for dock_id, dock in self.detected_docks.items():
            dist = self._distance(position, dock.start_pos)
            if dist < min_dist:
                min_dist = dist
                nearest_id = dock_id
        
        return nearest_id
    
    def _select_best_dock(self) -> Optional[DockData]:
        """Select optimal dock from detected docks"""
        valid_docks = [d for d in self.detected_docks.values() if d.is_valid_target()]
        
        if not valid_docks:
            # Go back to starting posistion and rescan docs or scan on way up and go into first open dock
            return None
        
        # Sort by number (ascending)
        valid_docks.sort(key=lambda d: d.number)
        
        return valid_docks[0]
    
    # ========== PHASE TRANSITIONS ==========
    
    def _enter_phase(self, new_phase: Phase) -> None:
        """Transition to new phase"""
        print(f"[PHASE] {self.phase.value} → {new_phase.value}")
        self.phase = new_phase
        self.phase_start_time = time.time()
        
        # Phase-specific initialization
        if new_phase == Phase.SCANNING:
            self.scan_start_x = self.pose[0]
            self.scan_distance = 0.0
            self.detected_docks.clear()
            self.next_dock_id = 0
        
        elif new_phase == Phase.TURNING or new_phase == Phase.ROTATING_IN_DOCK:
            # Calculate 180° turn target
            self.target_heading = self._normalize_angle(self.pose[2] + np.pi)
        
        elif new_phase == Phase.ROTATING_TO_DOCK:
            # Calculate 90° rotation to face dock
            if self.target_dock:
                if self.target_dock.side == 'north':
                    self.target_heading = 0  # Face North
                else:
                    self.target_heading = np.pi  # Face South
    
    # ========== STATE METHODS ==========
    
    def _state_initial_turn(self) -> None:
        target_head = np.pi / 2 
        state = DockingState(
            dist_north=self.dist_north,
            dist_south=self.dist_south,
            x=self.pose[0],
            y=self.pose[1],
            heading=self.pose[2],
            target_heading= target_head,   
            target_speed=1,  # Move at a moderate pace
            mode=ControlMode.FORWARD
        )
        
        velocities = self.local_planner.compute_velocities(state)
        self._publish_velocity(
            velocities['forward'],
            velocities['lateral'],
            velocities['angular']
        )
        
        # Transition when scan complete
        if self.local_planner.is_heading_aligned(state, tolerance_deg=2.0):
            self._enter_phase(Phase.INITIAL_CENTERING) 
            self.start_pos = (self.pose[0], self.pose[1])

    # On the 20th I updated things to skip this and go right to initial_centering becuase we won't be that far from the dock
    # And becuase the dock is really wide and there is the beach so this might not work
    def _state_initial_approach(self) -> None:
        # NO PID HERE NO LOCAL PLANNER JUST RAW DOGGING THE BOAT INTO THE RIGHT POSITION RAHHHHHHH
        fov90 = self.laser_scan[len(self.laser_scan)//4 : len(self.laser_scan)//4 * 3]
        best_cluster_size = 0
        best_cluster_start = 0
        cur_cluster_size = 0
        cur_cluster_start = 0
        prev_x = float('inf')
        for x in fov90:
            if x == float('inf') or abs(x - prev_x) > 1: # Tune sensitivity here
                if cur_cluster_size > best_cluster_start:
                    best_cluster_size = cur_cluster_size
                    best_cluster_start = cur_cluster_start
                cur_cluster_size = 0
            else:
                if cur_cluster_size == 0:
                    cur_cluster_start = x
                cur_cluster_size += 1
            prev_x = x
        
        best_cluster_center = (best_cluster_start + best_cluster_size) // 2
        best_cluster_heading = self._normalize_angle(self.heading + (2 * best_cluster_center/180*np.pi - np.pi/4))

        self._publish_velocity(math.cos(best_cluster_heading), math.sin(best_cluster_heading), 0)

        if (mean(fov90[len(fov90)//4, len(fov90)//4 * 3]) < 10):
            self._enter_phase(Phase.INITIAL_CENTERING)

    def _state_initial_centering(self) -> None:
        """Initial centering between docks"""
        # NO PID HERE NO LOCAL PLANNER JUST RAW DOGGING THE BOAT INTO THE RIGHT POSITION RAHHHHHHH
        # The strat is to find the corners of the enterance and drive to whichever one is further (and if you're within like a metre you've commited to your course so try to hit the centre)
        fov90 = self.laser_scan[len(self.laser_scan)//4 : len(self.laser_scan)//4 * 3] # THIS MAY BE TOO THIN FOR WHEN WE GET CLOSE
        left_edge = 0
        right_edge = -1
        prev_left = float('inf')
        prev_right = float('inf')
        for i in range(1,45): # sweep across until there's a big change
            left = fov90[i]
            right = fov90[-i]
            if left_edge == 0 and prev_left != float('inf') and abs(left - prev_left) > 0.8: # Tune sensitivity here
                left_edge = i
            if right_edge == -1 and prev_right != float('inf') and abs(right - prev_right) > 0.8: # Tune sensitivity here
                right_edge = -i
            prev_left = left
            prev_right = right
        
        # When you're close try to shoot the centre and slow down
        if fov90[left_edge] < 5 or fov90[right_edge] < 5: #TODO Tune this
            centre = (left_edge - right_edge)/2
            desired_heading = self._normalize_angle(self.heading + (2 * centre/180*np.pi - np.pi/4))
            _publish_velocity(0.5*math.cos(desired_heading), 0.5*math.sin(desired_heading), 0)
        # When you're far go towards the farther edge to get centered
        else:
            if fov90[left_edge] > fov90[right_edge]:
                desired_heading = self._normalize_angle(self.heading + (2 * left_edge/180*np.pi - np.pi/4))
            else:
                desired_heading = self._normalize_angle(self.heading + (2 * right_edge/180*np.pi + np.pi/4))
            _publish_velocity(math.cos(desired_heading), math.sin(desired_heading), 0)

        if self.laser_scan[0] != float('inf') and abs(self.laser_scan[0] - self.laser_scan[-1]) < 10: # TODO DOCKS are 10 m appart?
            self._enter_phase(Phase.SCANNING)
    
    def _state_scanning(self) -> None:
        """Scan marina while driving East"""
        target_head = np.pi / 2 
        state = DockingState(
            dist_north=self.dist_north,
            dist_south=self.dist_south,
            x=self.pose[0],
            y=self.pose[1],
            heading=self.pose[2],
            target_heading= target_head,  # East = 90° = π/2 #TODO make this robust to the dock not being aligned N/S #TODONE East West
            target_speed=0.3,  # Slow scanning speed
            mode=ControlMode.FORWARD
        )
        
        velocities = self.local_planner.compute_velocities(state)
        self._publish_velocity(
            velocities['forward'],
            velocities['lateral'],
            velocities['angular']
        )
        
        # Transition when scan complete
        if self.scan_distance >= self.expected_scan_distance:
            self._enter_phase(Phase.SELECTING)
    
    def _state_selecting(self) -> None:
        """Select best dock"""
        # Stop motion
        self._publish_velocity(0.0, 0.0, 0.0)
        
        # Select dock
        self.target_dock = self._select_best_dock()
        
        if self.target_dock:
            print(f"[SELECT] Chose dock {self.target_dock.dock_id}: "
                  f"{self.target_dock.side} side, number {self.target_dock.number}")
            self._enter_phase(Phase.TURNING)
        else:
            print("[SELECT] No valid dock found! Giving up")
            self._enter_phase(Phase.RETURNING)
    
    def _state_turning(self) -> None:
        """Execute 180° turn"""
        state = DockingState(
            dist_north=self.dist_north,
            dist_south=self.dist_south,
            x=self.pose[0],
            y=self.pose[1],
            heading=self.pose[2],
            target_heading=self.target_heading,
            mode=ControlMode.ROTATE
        )
        
        velocities = self.local_planner.compute_velocities(state)
        self._publish_velocity(
            velocities['forward'],
            velocities['lateral'],
            velocities['angular']
        )
        
        # Transition when aligned
        if self.local_planner.is_heading_aligned(state, tolerance_deg=2.0):
            #self._enter_phase(Phase.RECENTERING) 
            # Our boat can go sideways, go straight to APPROACHING
            self._enter_phase(Phase.APPROACHING) 
    
    # Never used becuase boat can't go sideways
    def _state_recentering(self) -> None:
        """Re-center after turn"""
        # Same as initial centering
        self._state_initial_centering()
        
        # But transition to approaching instead
        state = DockingState(
            dist_north=self.dist_north,
            dist_south=self.dist_south,
            x=self.pose[0],
            y=self.pose[1],
            heading=self.pose[2],
            mode=ControlMode.CENTERING
        )
        
        if self.local_planner.is_centered(state):
            self._enter_phase(Phase.APPROACHING)
    
    def _state_approaching(self) -> None:
        """Navigate to dock's Y-position"""
        # In the scan we assumed East was the X-axis so I changed this method to move in X
        if not self.target_dock:
            return
        
        # Calculate waypoint at dock's lateral position
        #dock_center = self.target_dock.center_point()
        #waypoint_y = dock_center[1]
        
        # Check if reached lateral position
        #y_error = abs(self.pose[1] - waypoint_y)
        
        #if y_error < 1.0:  # Within 1m
        #    self._enter_phase(Phase.ROTATING_TO_DOCK)
        #    return

        # Calculate waypoint at dock's lateral position
        dock_center = self.target_dock.center_point()
        waypoint_x = dock_center[0]
        
        # Check if reached lateral position
        x_error = abs(self.pose[0] - waypoint_x)
        
        if x_error < 0.15:  # Within 1/2 ft TODONE meters
            self._enter_phase(Phase.ROTATING_TO_DOCK)
            return
        
        # Use forward motion toward waypoint
        state = DockingState(
            dist_north=self.dist_north,
            dist_south=self.dist_south,
            x=self.pose[0],
            y=self.pose[1],
            heading=self.pose[2],
            target_heading=self.target_heading,
            target_speed=0.2,
            mode=ControlMode.FORWARD
        )
        
        velocities = self.local_planner.compute_velocities(state)
        self._publish_velocity(
            velocities['forward'],
            velocities['lateral'],
            velocities['angular']
        )
    
    def _state_rotating_to_dock(self) -> None:
        """Rotate 90° to face dock"""
        state = DockingState(
            dist_north=self.dist_north,
            dist_south=self.dist_south,
            x=self.pose[0],
            y=self.pose[1],
            heading=self.pose[2],
            target_heading=self.target_heading,
            mode=ControlMode.ROTATE
        )
        
        velocities = self.local_planner.compute_velocities(state)
        self._publish_velocity(
            velocities['forward'],
            velocities['lateral'],
            velocities['angular']
        )
        
        # Transition when aligned
        if self.local_planner.is_heading_aligned(state, tolerance_deg=2.0):
            self._enter_phase(Phase.DOCKING)
    
    def _state_returning(self) -> None:
        """Return to initial position to rescan"""
        # Calculate distance to start position
        dx = self.start_pos[0] - self.pose[0]
        dy = self.start_pos[1] - self.pose[1]
        distance_to_start = self._distance(self.start_pos, (self.pose[0], self.pose[1]))
        
        # Check if reached start position (within 1.0m tolerance)
        if distance_to_start < 1.0: # TODONE meters
            print(f"[RETURN] Reached start position. Wiping docks and restarting scan.")
            self._publish_velocity(0.0, 0.0, 0.0)
            self.detected_docks.clear()
            self.next_dock_id = 0
            self.target_dock = None
            self._enter_phase(Phase.DONE)
            return 
        
        # Navigate back to start position
        target_heading = self._normalize_angle(np.arctan2(dy, dx))
        
        state = DockingState(
            dist_north=self.dist_north,
            dist_south=self.dist_south,
            x=self.pose[0],
            y=self.pose[1],
            heading=self.pose[2],
            target_heading=target_heading,
            target_speed=1,  # Moderate return speed
            mode=ControlMode.FORWARD
        )
        
        velocities = self.local_planner.compute_velocities(state)
        self._publish_velocity(
            velocities['forward'],
            velocities['lateral'],
            velocities['angular']
        )
    
    def _state_docking(self) -> None:
        """Final docking approach"""
        state = DockingState(
            dist_north=self.dist_north,
            dist_south=self.dist_south,
            dist_forward=self.dist_forward,
            x=self.pose[0],
            y=self.pose[1],
            heading=self.pose[2],
            target_heading=self.target_heading,
            mode=ControlMode.DOCKING
        )
        
        velocities = self.local_planner.compute_velocities(state)
        
        # Safety check
        if velocities['safety'] == SafetyStatus.CRITICAL:
            print('[DOCKING] EMERGENCY STOP - Clearance violation!')
            self._emergency_stop()
            return
        
        # Publish velocities
        self._publish_velocity(
            velocities['forward'],
            velocities['lateral'],
            velocities['angular']
        )
        
        # Check if docked
        if state.dist_forward and state.dist_forward <= DOCKING_TARGET_DISTANCE:
            self._enter_phase(Phase.DOCKED)
            print(f"[SUCCESS] Docked in {time.time() - self.mission_start_time:.1f}s")

    def _state_docked(self) -> None:
        self._enter_phase(Phase.ROTATING_IN_DOCK)

    def _state_rotating_in_dock(self) -> None:
        """Execute 180° turn""" # Identical copy of what we did in turning
        state = DockingState(
            dist_north=self.dist_north,
            dist_south=self.dist_south,
            x=self.pose[0],
            y=self.pose[1],
            heading=self.pose[2],
            target_heading=self.target_heading,
            mode=ControlMode.ROTATE
        )
        
        velocities = self.local_planner.compute_velocities(state)
        self._publish_velocity(
            velocities['forward'],
            velocities['lateral'],
            velocities['angular']
        )
        
        # Transition when aligned
        if self.local_planner.is_heading_aligned(state, tolerance_deg=2.0):
            self._enter_phase(Phase.UNDOCKING) 

    def _state_undocking(self) -> None:
        # Calculate waypoint at dock's vertical position
        dock_center = self.target_dock.center_point()
        waypoint_y = dock_center[1] - 4 if (self.target_dock.side == 'north') else dock_center[1] + 4 # It's wide so pull 4 metres out of the dock
        
        # Check if reached lateral position
        y_error = abs(self.pose[1] - waypoint_y)
        
        if y_error < 0.15:  # Within 1/2 ft TODONE meters
            self._enter_phase(Phase.RETURNING)
            return
        
        # Use forward motion toward waypoint
        state = DockingState(
            dist_north=self.dist_north,
            dist_south=self.dist_south,
            x=self.pose[0],
            y=self.pose[1],
            heading=self.pose[2],
            target_heading=self.target_heading,
            target_speed=0.2,
            mode=ControlMode.FORWARD
        )
        
        velocities = self.local_planner.compute_velocities(state)
        self._publish_velocity(
            velocities['forward'],
            velocities['lateral'],
            velocities['angular']
        )
 

    # ========== HELPER METHODS ==========
   
   # Never used
    def _get_dock_heading(self) -> float:
        """Get heading to face target dock"""
        if self.target_dock:
            if self.target_dock.side == 'north':
                return np.pi / 2
            else:
                return -np.pi / 2
        return self.pose[2]
    
    def _emergency_stop(self) -> None:
        """Emergency stop all motion"""
        self._publish_velocity(0.0, 0.0, 0.0)
        self.local_planner.reset()
    
    def _distance(self, a: Vec2, b: Vec2) -> float:
        """Calculate Euclidean distance"""
        return float(np.hypot(a[0] - b[0], a[1] - b[1]))
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-π, π]"""
        while angle > 2.0 * np.pi:
            angle -= 2.0 * np.pi
        while angle < 0:
            angle += 2.0 * np.pi
        return angle
    
    # ========== MAIN UPDATE ==========
    
    def tick(self) -> None:
        """Main update loop"""
        # Call phase-specific state function
        if self.phase == Phase.INITIAL_TURN:
            self._state_initial_turn()
        if self.phase == Phase.APPROACH:
            self._state_initial_approach()
        if self.phase == Phase.INITIAL_CENTERING: 
            self._state_initial_centering()
        elif self.phase == Phase.SCANNING:
            self._state_scanning()
        elif self.phase == Phase.SELECTING:
            self._state_selecting()
        elif self.phase == Phase.TURNING:
            self._state_turning()
        #elif self.phase == Phase.RECENTERING:
        #    self._state_recentering()
        elif self.phase == Phase.APPROACHING:
            self._state_approaching()
        elif self.phase == Phase.ROTATING_TO_DOCK:
            self._state_rotating_to_dock()
        elif self.phase == Phase.DOCKING:
            self._state_docking()
        elif self.phase == Phase.DOCKED:
            self._state_docked()
        elif self.phase == Phase.ROTATING_IN_DOCK:
            self._state_rotating_in_dock()
        elif self.phase == Phase.UNDOCKING:
            self._state_undocking()
        elif self.phase == Phase.RETURNING:
            self._state_returning()
        elif self.phase == Phase.DONE:
            pass
        
    
    def done(self) -> bool:
        """Check if mission complete"""
        return self.phase == Phase.DONE
    
    # No longer used, forward distance comes in with the rest of lidar data
    def _get_forward_distance(self) -> Optional[float]:
        """
        Get forward distance from LiDAR
    
        TODO: 
            Use the pointcloud to laser scan that Ethan made to get forward distance
        """
        return None  # Placeholder

    def _publish_velocity(self, forward, lateral, angular):
        """
        TODO: Publish to your motion controller
        
            publish velo vector
        """
        # We set this paramater that the TaskMaster reads from
        # Might need to change the format so TaskMaster reads the angular
        self.current_velocities = (forward, lateral, angular)

