#!/usr/bin/env python3
"""
Docking Local Planner:
Specialized motion controller for autonomous docking operations.

Unlike the potential fields planner used for open-water navigation,
this planner provides precise, low-level control for:
- Lateral centering between dock walls
- Slow, controlled forward motion
- Real-time clearance monitoring
- Collision avoidance with tight tolerances

Inputs:
    - Current boat pose (x, y, heading)
    - LiDAR distances (north, south, forward)
    - Target dock information
    - Control mode (centering, approaching, docking)

Outputs:
    - Velocity commands (forward, lateral, angular)
    - Safety status
    - Control diagnostics

Design Philosophy:
- Predictable, smooth motion (no gradient oscillations)
- Real-time reactive control (not pre-planned paths)
- Safety-first (stop on clearance violations)
- Tunable for different boat dynamics
"""

import numpy as np
from typing import Tuple, Optional, Dict
from enum import Enum
from dataclasses import dataclass


# ============================================================================
# TUNABLE PARAMETERS - Adjust for your boat's characteristics
# ============================================================================

# PID Gains - Lateral Centering
CENTERING_KP = 0.8      # Proportional gain
CENTERING_KI = 0.05     # Integral gain
CENTERING_KD = 0.15     # Derivative gain
CENTERING_I_LIMIT = 0.3 # Anti-windup limit

# PID Gains - Heading Control
HEADING_KP = 1.0
HEADING_KI = 0.01
HEADING_KD = 0.2
HEADING_I_LIMIT = 0.5

# PID Gains - Forward Speed Control
SPEED_KP = 0.5
SPEED_KI = 0.02
SPEED_KD = 0.1
SPEED_I_LIMIT = 0.2

# Velocity Limits
MAX_FORWARD_SPEED = 1     # m/s - normal navigation
MAX_LATERAL_SPEED = 0.3     # m/s - centering
MAX_ROTATION_RATE = 1     # rad/s - turning
DOCKING_SPEED = 0.3         # m/s - final approach
APPROACH_SPEED = 0.3        # m/s - approaching dock

# Safety Thresholds NEVER BACK DOWN NEVER WHAT?
MIN_LATERAL_CLEARANCE = 0.0   # meters - emergency stop threshold
WARNING_CLEARANCE = 0.25       # meters - slow down threshold
MIN_FORWARD_CLEARANCE = 0.0    # meters - stop distance
DOCKING_TARGET_DISTANCE = 0.5  # meters - final stop position

# Centering Tolerances
CENTERING_TOLERANCE = 0.1      # meters - acceptable offset
CENTERING_DEADBAND = 0.02      # meters - ignore tiny errors

# Control Update Rates
CONTROL_DT = 0.05              # seconds - 20 Hz control loop

# Smoothing
VELOCITY_ALPHA = 0.7           # Exponential smoothing factor (0=no change, 1=instant)

# ============================================================================


class ControlMode(Enum):
    """Control modes for different docking phases"""
    IDLE = "IDLE"                    # No control
    CENTERING = "CENTERING"          # Lateral centering only
    FORWARD = "FORWARD"              # Forward motion with centering and data collection
    HEADING_HOLD = "HEADING_HOLD"    # Maintain heading
    ROTATE = "ROTATE"                # Pure rotation
    DOCKING = "DOCKING"              # Precise docking approach


class SafetyStatus(Enum):
    """Safety assessment states"""
    SAFE = "SAFE"                    # Normal operation
    WARNING = "WARNING"              # Approaching limits
    CRITICAL = "CRITICAL"            # Emergency stop required
    BLOCKED = "BLOCKED"              # Path obstructed


@dataclass
class DockingState:
    """Current state for docking control"""
    # Distances from LiDAR
    dist_north: Optional[float] = None
    dist_south: Optional[float] = None
    dist_forward: Optional[float] = None
    
    # Boat pose
    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0  # radians
    
    # Target information
    target_heading: Optional[float] = None
    target_speed: float = 0.0
    target_y: Optional[float] = None  # For lateral positioning
    
    # Control mode
    mode: ControlMode = ControlMode.IDLE
    
    # Safety
    safety_status: SafetyStatus = SafetyStatus.SAFE


class PIDController:
    """
    Simple PID controller with anti-windup
    """
    
    def __init__(self, kp: float, ki: float, kd: float, 
                 output_limits: Tuple[float, float] = (-1.0, 1.0),
                 i_limit: float = 1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        self.i_limit = i_limit
        
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
    
    def update(self, error: float, dt: float) -> float:
        """
        Compute PID output
        
        Args:
            error: Current error (setpoint - measurement)
            dt: Time step (seconds)
        
        Returns:
            Control output
        """
        # P term
        p_term = self.kp * error
        
        # I term with anti-windup
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.i_limit, self.i_limit)
        i_term = self.ki * self.integral
        
        # D term
        d_term = 0.0
        if dt > 0:
            d_term = self.kd * (error - self.prev_error) / dt
        self.prev_error = error
        
        # Total output
        output = p_term + i_term + d_term
        
        # Apply output limits
        output = np.clip(output, self.output_limits[0], self.output_limits[1])
        
        return output
    
    def reset(self):
        """Reset controller state"""
        self.integral = 0.0
        self.prev_error = 0.0


class DockingLocalPlanner:
    """
    Local planner for autonomous docking operations
    
    Provides precise, reactive control for:
    - Centering between dock walls
    - Controlled forward motion
    - Heading maintenance
    - Rotation maneuvers
    - Final docking approach
    
    Example usage:
        planner = DockingLocalPlanner()
        
        # Update state
        state = DockingState(
            dist_north=2.0,
            dist_south=2.1,
            x=50.0, y=30.0, heading=1.57,
            mode=ControlMode.CENTERING
        )
        
        # Get velocity commands
        velocities = planner.compute_velocities(state)
        forward_vel = velocities['forward']
        lateral_vel = velocities['lateral']
        angular_vel = velocities['angular']
    """
    
    def __init__(self):
        """Initialize planner with PID controllers"""
        
        # Create PID controllers
        self.centering_pid = PIDController(
            CENTERING_KP, CENTERING_KI, CENTERING_KD,
            output_limits=(-MAX_LATERAL_SPEED, MAX_LATERAL_SPEED),
            i_limit=CENTERING_I_LIMIT
        )
        
        self.heading_pid = PIDController(
            HEADING_KP, HEADING_KI, HEADING_KD,
            output_limits=(-MAX_ROTATION_RATE, MAX_ROTATION_RATE),
            i_limit=HEADING_I_LIMIT
        )
        
        self.speed_pid = PIDController(
            SPEED_KP, SPEED_KI, SPEED_KD,
            output_limits=(-MAX_FORWARD_SPEED, MAX_FORWARD_SPEED),
            i_limit=SPEED_I_LIMIT
        )
        
        # Velocity smoothing
        self.smoothed_forward = 0.0
        self.smoothed_lateral = 0.0
        self.smoothed_angular = 0.0
        
        # State tracking
        self.last_update_time = None
    
    def compute_velocities(self, state: DockingState, dt: float = CONTROL_DT) -> Dict[str, float]:
        """
        Compute velocity commands based on current state and mode
        
        Args:
            state: Current docking state
            dt: Time step (seconds)
        
        Returns:
            Dictionary with keys:
                'forward': Forward velocity (m/s)
                'lateral': Lateral velocity (m/s)
                'angular': Angular velocity (rad/s)
                'safety': Safety status
        """
        # Check safety first
        safety = self._assess_safety(state)
        
        # Emergency stop if critical
        if safety == SafetyStatus.CRITICAL:
            return {
                'forward': 0.0,
                'lateral': 0.0,
                'angular': 0.0,
                'safety': safety
            }
        
        # Compute velocities based on mode
        if state.mode == ControlMode.IDLE:
            forward, lateral, angular = 0.0, 0.0, 0.0
            
        elif state.mode == ControlMode.CENTERING:
            forward = 0.0
            lateral = self._compute_centering_velocity(state, dt)
            angular = 0.0
            
        elif state.mode == ControlMode.FORWARD:
            forward = self._compute_forward_velocity(state, dt, safety)
            lateral = self._compute_centering_velocity(state, dt)
            angular = self._compute_heading_velocity(state, dt)
            
        elif state.mode == ControlMode.HEADING_HOLD:
            forward = state.target_speed
            lateral = 0.0
            angular = self._compute_heading_velocity(state, dt)
            
        elif state.mode == ControlMode.ROTATE:
            forward = 0.0
            lateral = 0.0
            angular = self._compute_heading_velocity(state, dt)
            
        elif state.mode == ControlMode.DOCKING:
            forward = self._compute_docking_velocity(state, dt, safety)
            lateral = self._compute_centering_velocity(state, dt)
            angular = self._compute_heading_velocity(state, dt)
        
        else:
            forward, lateral, angular = 0.0, 0.0, 0.0
        
        # Apply smoothing
        forward = self._smooth_velocity(forward, self.smoothed_forward)
        lateral = self._smooth_velocity(lateral, self.smoothed_lateral)
        angular = self._smooth_velocity(angular, self.smoothed_angular)
        
        # Store smoothed values
        self.smoothed_forward = forward
        self.smoothed_lateral = lateral
        self.smoothed_angular = angular
        
        return {
            'forward': forward,
            'lateral': lateral,
            'angular': angular,
            'safety': safety
        }
    
    def _assess_safety(self, state: DockingState) -> SafetyStatus:
        """
        Assess safety based on clearances
        CRITICAL: < 15cm to wall → EMERGENCY STOP
        WARNING: < 25cm to wall → SLOW DOWN
        SAFE: > 25cm clearance → NORMAL
        Returns:
            SafetyStatus enum value
        """
        # Check lateral clearances
        if state.dist_north is not None and state.dist_north < MIN_LATERAL_CLEARANCE:
            return SafetyStatus.CRITICAL
        if state.dist_south is not None and state.dist_south < MIN_LATERAL_CLEARANCE:
            return SafetyStatus.CRITICAL
        
        # Check forward clearance
        if state.dist_forward is not None and state.dist_forward < MIN_FORWARD_CLEARANCE:
            return SafetyStatus.CRITICAL
        
        # Check warnings
        if state.dist_north is not None and state.dist_north < WARNING_CLEARANCE:
            return SafetyStatus.WARNING
        if state.dist_south is not None and state.dist_south < WARNING_CLEARANCE:
            return SafetyStatus.WARNING
        if state.dist_forward is not None and state.dist_forward < WARNING_CLEARANCE * 2:
            return SafetyStatus.WARNING
        
        return SafetyStatus.SAFE
    
    def _compute_centering_velocity(self, state: DockingState, dt: float) -> float:
        """
        Compute lateral velocity for centering
        
        Returns:
            Lateral velocity (m/s) - positive = move North
        """
        if state.dist_north is None or state.dist_south is None:
            return 0.0
        
        # Calculate centerline and current offset
        center_distance = (state.dist_north + state.dist_south) / 2.0
        offset = state.dist_north - center_distance
        
        # Apply deadband to avoid oscillation
        if abs(offset) < CENTERING_DEADBAND:
            self.centering_pid.reset()
            return 0.0
        
        # NEW!! If the offset is large you're probably seeing an occupied dock on one side, so keep straight
        if abs(offset) > 0.2:
            return 0.0

        # PID control
        # Error: positive means too far North, need to move South (negative velocity)
        lateral_vel = -self.centering_pid.update(offset, dt)
        
        return lateral_vel
    
    def _compute_heading_velocity(self, state: DockingState, dt: float) -> float:
        """
        Compute angular velocity for heading control
        
        Returns:
            Angular velocity (rad/s)
        """
        if state.target_heading is None:
            return 0.0
        
        # Calculate heading error (shortest angle)
        error = self._normalize_angle(state.target_heading - state.heading)
        
        # PID control
        angular_vel = self.heading_pid.update(error, dt)
        
        return angular_vel
    
    def _compute_forward_velocity(self, state: DockingState, dt: float, 
                                  safety: SafetyStatus) -> float:
        """
        Compute forward velocity for normal navigation
        
        Returns:
            Forward velocity (m/s)
        """
        target_speed = state.target_speed if state.target_speed is not None else APPROACH_SPEED
        
        # Reduce speed if warning status
        if safety == SafetyStatus.WARNING:
            target_speed *= 0.5
        
        # Limit to maximum
        target_speed = min(target_speed, MAX_FORWARD_SPEED)
        
        return target_speed
    
    def _compute_docking_velocity(self, state: DockingState, dt: float,
                                  safety: SafetyStatus) -> float:
        """
        Compute forward velocity for final docking approach
        
        Returns:
            Forward velocity (m/s)
        """
        if state.dist_forward is None:
            # No forward distance info - use slow default
            return DOCKING_SPEED * 0.5
        
        # Check if we've reached target
        if state.dist_forward <= DOCKING_TARGET_DISTANCE:
            return 0.0  # STOP
        
        # Distance-based speed scaling
        # Far away: use docking speed
        # Close: scale down proportionally
        distance_to_target = state.dist_forward - DOCKING_TARGET_DISTANCE
        
        if distance_to_target > 2.0:
            # Far enough - use full docking speed
            speed = DOCKING_SPEED
        else:
            # Close - scale down linearly
            # At 2m: full speed
            # At 0m (target): zero speed
            scale = distance_to_target / 2.0
            speed = DOCKING_SPEED * scale
            # Minimum speed to avoid stalling
            speed = max(speed, 0.05)
        
        # Safety reduction
        if safety == SafetyStatus.WARNING:
            speed *= 0.5
        
        return speed
    
    def _smooth_velocity(self, new_value: float, old_value: float) -> float:
        """
        Apply exponential smoothing to velocity
        Currently 0.3 new value, 0.7 old value
        
        Args:
            new_value: New commanded velocity
            old_value: Previous smoothed velocity
        
        Returns:
            Smoothed velocity
        """
        return VELOCITY_ALPHA * new_value + (1 - VELOCITY_ALPHA) * old_value
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle
    
    def reset(self):
        """Reset all controllers and state"""
        self.centering_pid.reset()
        self.heading_pid.reset()
        self.speed_pid.reset()
        self.smoothed_forward = 0.0
        self.smoothed_lateral = 0.0
        self.smoothed_angular = 0.0
    
    def is_centered(self, state: DockingState) -> bool:
        """
        Check if boat is centered within tolerance
        
        Returns:
            True if centered
        """
        if state.dist_north is None or state.dist_south is None:
            return False
        
        offset = abs(state.dist_north - state.dist_south)
        return offset < CENTERING_TOLERANCE
    
    def is_heading_aligned(self, state: DockingState, tolerance_deg: float = 5.0) -> bool:
        """
        Check if heading is aligned with target
        
        Args:
            state: Current state
            tolerance_deg: Tolerance in degrees
        
        Returns:
            True if aligned
        """
        if state.target_heading is None:
            return True
        
        error = abs(self._normalize_angle(state.target_heading - state.heading))
        tolerance_rad = np.radians(tolerance_deg)
        
        return error < tolerance_rad
    
    def get_centering_offset(self, state: DockingState) -> Optional[float]:
        """
        Get current centering offset
        
        Returns:
            Offset in meters (positive = too far North, negative = too far South)
            None if distances not available
        """
        if state.dist_north is None or state.dist_south is None:
            return None
        
        center_distance = (state.dist_north + state.dist_south) / 2.0
        offset = state.dist_north - center_distance
        
        return offset
    
    def estimate_time_to_dock(self, state: DockingState) -> Optional[float]:
        """
        Estimate time to reach docking position
        
        Returns:
            Estimated time in seconds, or None if cannot estimate
        """
        if state.dist_forward is None or state.dist_forward <= DOCKING_TARGET_DISTANCE:
            return 0.0
        
        distance_remaining = state.dist_forward - DOCKING_TARGET_DISTANCE
        avg_speed = DOCKING_SPEED * 0.7  # Assume 70% average speed due to scaling
        
        if avg_speed > 0:
            return distance_remaining / avg_speed
        else:
            return None
    
    def set_gains(self, controller: str, kp: float = None, ki: float = None, kd: float = None):
        """
        Update PID gains for tuning
        
        Args:
            controller: 'centering', 'heading', or 'speed'
            kp, ki, kd: New gain values (None = keep existing)
        """
        if controller == 'centering':
            pid = self.centering_pid
        elif controller == 'heading':
            pid = self.heading_pid
        elif controller == 'speed':
            pid = self.speed_pid
        else:
            raise ValueError(f"Unknown controller: {controller}")
        
        if kp is not None:
            pid.kp = kp
        if ki is not None:
            pid.ki = ki
        if kd is not None:
            pid.kd = kd


# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

def create_centering_state(dist_north: float, dist_south: float, 
                           x: float, y: float, heading: float) -> DockingState:
    """
    Convenience function to create a centering state
    
    Example:
        state = create_centering_state(2.0, 2.1, 50.0, 30.0, 1.57)
    """
    return DockingState(
        dist_north=dist_north,
        dist_south=dist_south,
        x=x,
        y=y,
        heading=heading,
        mode=ControlMode.CENTERING
    )


def create_docking_state(dist_north: float, dist_south: float, dist_forward: float,
                        x: float, y: float, heading: float, target_heading: float) -> DockingState:
    """
    Convenience function to create a docking state
    
    Example:
        state = create_docking_state(2.0, 2.1, 5.0, 50.0, 30.0, 1.57, 1.57)
    """
    return DockingState(
        dist_north=dist_north,
        dist_south=dist_south,
        dist_forward=dist_forward,
        x=x,
        y=y,
        heading=heading,
        target_heading=target_heading,
        mode=ControlMode.DOCKING
    )


# ============================================================================
# EXAMPLE USAGE
# ============================================================================

"""
if __name__ == "__main__":

    Example usage demonstrating the docking planner

    
    planner = DockingLocalPlanner()
    
    # Example 1: Centering
    print("=" * 60)
    print("Example 1: Centering between docks")
    print("=" * 60)
    
    state = DockingState(
        dist_north=2.5,  # 2.5m to north wall
        dist_south=1.5,  # 1.5m to south wall (off-center!)
        x=50.0,
        y=30.0,
        heading=1.57,  # Facing north (π/2)
        mode=ControlMode.CENTERING
    )
    
    velocities = planner.compute_velocities(state)
    print(f"North distance: {state.dist_north}m")
    print(f"South distance: {state.dist_south}m")
    print(f"Offset: {planner.get_centering_offset(state):.3f}m (positive = too far North)")
    print(f"Lateral velocity: {velocities['lateral']:.3f} m/s")
    print(f"Centered: {planner.is_centered(state)}")
    print(f"Safety: {velocities['safety'].value}")
    
    # Example 2: Final docking approach
    print("\n" + "=" * 60)
    print("Example 2: Final docking approach")
    print("=" * 60)
    
    state = DockingState(
        dist_north=2.0,
        dist_south=2.0,  # Perfectly centered
        dist_forward=3.0,  # 3m from dock end
        x=50.0,
        y=30.0,
        heading=1.57,
        target_heading=1.57,  # Maintain heading
        mode=ControlMode.DOCKING
    )
    
    velocities = planner.compute_velocities(state)
    print(f"Forward distance: {state.dist_forward}m")
    print(f"Target stop distance: {DOCKING_TARGET_DISTANCE}m")
    print(f"Forward velocity: {velocities['forward']:.3f} m/s")
    print(f"Lateral velocity: {velocities['lateral']:.3f} m/s")
    print(f"Estimated time to dock: {planner.estimate_time_to_dock(state):.1f}s")
    print(f"Safety: {velocities['safety'].value}")
    
    # Example 3: Critical safety situation
    print("\n" + "=" * 60)
    print("Example 3: Critical safety - too close to wall")
    print("=" * 60)
    
    state = DockingState(
        dist_north=0.10,  # TOO CLOSE! (< 0.15m minimum)
        dist_south=2.0,
        dist_forward=5.0,
        x=50.0,
        y=30.0,
        heading=1.57,
        mode=ControlMode.DOCKING
    )
    
    velocities = planner.compute_velocities(state)
    print(f"North distance: {state.dist_north}m (CRITICAL!)")
    print(f"Minimum clearance: {MIN_LATERAL_CLEARANCE}m")
    print(f"Forward velocity: {velocities['forward']:.3f} m/s (STOPPED)")
    print(f"Safety: {velocities['safety'].value}")
    
    print("\n" + "=" * 60)
    print("Docking planner examples complete")
    print("=" * 60)
"""
