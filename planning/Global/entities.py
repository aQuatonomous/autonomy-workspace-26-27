#!/usr/bin/env python3
"""
Location: src/Planner/Global/

Entity definitions + helper methods required by Global Tasks.

Supports:
- Start position (stored on EntityList)
- Gate pairing (red_buoy + green_buoy) => get_gates()
- Obstacle extraction => get_obstacles()
- Goals written by tasks as type="goal" and names goal_wp1..N
"""

from __future__ import annotations
import math
import time
from typing import List, Tuple, Optional, Any

from Global.types import Vec2, Vec3, _to_vec3, DetectedEntity

# No-go zone: walls connecting consecutive gates (red-to-red, green-to-green)
GATE_NO_GO_DEFAULT_EXTEND_M = 30.0  # Forward/backward extension when no next/prev gate (m)
GATE_NO_GO_GAP_M = 12.0             # Stop walls this far from the next gate (m) - LARGE gap for clear approach
GATE_NO_GO_SAMPLE_SPACING = 3.0     # Sample spacing for wall points - sparse sampling

# Gate filter: minimum cross-track ratio (gate vector perpendicular to forward).
# 0.5 means at least 50% of the gate line is across the channel (allows ~60° tilt).
GATE_MIN_CROSS_RATIO = 0.5

# Matches computer_vision/class_mapping.yaml (0-22). class_id 255 = unknown (unmapped).
CLASS_ID_TO_ENTITY_TYPE = {
    0: "black_buoy",
    1: "green_buoy",
    2: "green_pole_buoy",
    3: "red_buoy",
    4: "red_pole_buoy",
    5: "yellow_buoy",
    6: "cross",
    7: "dock",
    8: "triangle",
    9: "red_indicator",
    10: "green_indicator",
    11: "yellow_supply_drop",
    12: "black_supply_drop",
    20: "digit_1",
    21: "digit_2",
    22: "digit_3",
}

# When fusion sends color/class_name string (legacy or from yaml class name)
COLOR_TO_ENTITY_TYPE = {
    "red": "red_buoy",
    "green": "green_buoy",
    "yellow": "yellow_buoy",
    "black": "black_buoy",
    "black_buoy": "black_buoy",
    "unknown": "unknown",
    "red_indicator": "red_indicator",
    "green_indicator": "green_indicator",
    "red_indicator_buoy": "red_indicator",
    "green_indicator_buoy": "green_indicator",
    "green_pole_buoy": "green_buoy",
    "red_pole_buoy": "red_buoy",
    "yellow_supply_drop": "yellow_supply_drop",
    "black_supply_drop": "black_supply_drop",
}


def _color_to_entity_type(color: str) -> str:
    """Map color/class_name string to entity type. Unmapped inputs return 'unknown', not black."""
    return COLOR_TO_ENTITY_TYPE.get((color or "unknown").strip().lower(), "unknown")


def _get_attr(obj: Any, key: str, default: Any = None) -> Any:
    """Get key from object (ROS msg) or dict."""
    if hasattr(obj, key):
        return getattr(obj, key)
    if isinstance(obj, dict):
        return obj.get(key, default)
    return default


def apply_tracked_buoys(entity_list: "EntityList", buoys: List[Any], tolerance: float = 1.0) -> None:
    """
    Update EntityList from tracked-buoy-like objects (ROS TrackedBuoy or dict).
    Each item must have: id (object_id for tracking), x, y; and either class_id (int, from
    class_mapping.yaml) or color (string). Same id = same entity.
    """
    for b in buoys:
        x = float(_get_attr(b, "x", 0.0))
        y = float(_get_attr(b, "y", 0.0))
        bid = int(_get_attr(b, "id", 0))
        class_id = _get_attr(b, "class_id")
        if class_id is not None:
            try:
                cid = int(class_id)
                entity_type = CLASS_ID_TO_ENTITY_TYPE.get(cid, "unknown")
            except (TypeError, ValueError):
                entity_type = _color_to_entity_type(str(_get_attr(b, "color") or "unknown"))
        else:
            color = _get_attr(b, "color") or "unknown"
            entity_type = _color_to_entity_type(str(color))
        if entity_type == "unknown":
            continue
        name = f"{entity_type}_{bid}"
        entity_list.add_or_update(entity_type, (x, y, 0.0), entity_id=bid, name=name)


class Entity:
    """Base class for all entities in the environment."""
    def __init__(
        self,
        entity_type: str,
        position: Tuple[float, ...],
        name: Optional[str] = None,
        entity_id: Optional[int] = None,
    ):
        self.type = entity_type
        self.position = _to_vec3(position)
        self.name = name or f"{entity_type}_{id(self)}"
        self.entity_id = entity_id  # from TrackedBuoy.id; None for goals
        self.last_seen: float = time.monotonic()

    def __repr__(self) -> str:
        return f"{self.type}({self.name}) at {self.position}"


class EntityList:
    """Container for all entities with helper methods used by global planner tasks."""
    def __init__(self, start_position: Tuple[float, ...] = (0.0, 0.0)):
        self.entities: List[Entity] = []
        self.start_position: Vec3 = _to_vec3(start_position)
        self._last_no_go_points: List[Vec2] = []
        self._no_go_gates: List[Tuple[Tuple[float, float, float], Tuple[float, float, float]]] = []
        self._boat_heading_for_no_go: Optional[float] = None
        # Locked map bounds: once computed from gates, never recompute (prevents shifting walls)
        self._locked_map_bounds: Optional[Tuple[float, float, float, float]] = None
        self._locked_map_bounds_gate_centers: List[Tuple[float, float]] = []  # Gate centers used to compute locked bounds
        # Incremental bounds extension: track which gates have been used for bounds
        self._map_bounds_gate_count: int = 0  # Number of gates used to create/extend bounds
        self._map_bounds_first_gate_center: Optional[Tuple[float, float]] = None

    # -------------------------
    # core CRUD
    # -------------------------

    def set_start_position(self, position: Tuple[float, ...]) -> None:
        self.start_position = _to_vec3(position)

    def add(
        self,
        entity_type: str,
        position: Tuple[float, ...],
        name: Optional[str] = None,
        entity_id: Optional[int] = None,
    ) -> Entity:
        ent = Entity(entity_type, position, name, entity_id=entity_id)
        self.entities.append(ent)
        return ent

    def add_or_update(
        self,
        entity_type: str,
        position: Tuple[float, ...],
        entity_id: int,
        name: Optional[str] = None,
    ) -> Entity:
        """
        Add or update entity by id. Same id = same entity.
        When the same entity_id is seen again, updates position (x,y) to track buoy shifts
        in global frame (e.g. sensor refinement, small drift, or real movement).
        """
        pos = _to_vec3(position)
        for existing in self.entities:
            if existing.entity_id is not None and existing.entity_id == entity_id:
                existing.position = pos
                existing.type = entity_type
                existing.last_seen = time.monotonic()
                if name is not None:
                    existing.name = name
                return existing
        return self.add(entity_type, position, name=name or f"{entity_type}_{entity_id}", entity_id=entity_id)

    def prune_stale(self, max_age_sec: float = 20.0) -> int:
        """Remove non-goal entities not seen for more than max_age_sec. Returns count removed."""
        now = time.monotonic()
        before = len(self.entities)
        self.entities = [
            e for e in self.entities
            if e.type == "goal" or (now - e.last_seen) <= max_age_sec
        ]
        return before - len(self.entities)

    def get_by_type(self, entity_type: str) -> List[Entity]:
        return [e for e in self.entities if e.type == entity_type]

    def get_positions_by_type(self, entity_type: str) -> List[Vec3]:
        return [e.position for e in self.entities if e.type == entity_type]

    # -------------------------
    # global planner helpers
    # -------------------------

    def get_start(self) -> Vec3:
        return self.start_position

    def get_goals(self) -> List[Vec3]:
        return self.get_positions_by_type("goal")

    def clear_goals(self) -> None:
        self.entities = [e for e in self.entities if e.type != "goal"]

    def _get_gate_entity_pairs(
        self,
        max_gate_width: float = 20.0,  # Increased from 15.0 to allow wider gates
        boat_heading_rad: Optional[float] = None,
        boat_pos: Optional[Tuple[float, float]] = None,
    ) -> List[Tuple["Entity", "Entity"]]:
        """Pair red/red_pole with green/green_pole based on left/right positioning.

        With boat_heading_rad (required for proper pairing):
        - Red buoy must be to the LEFT of boat heading
        - Green buoy must be to the RIGHT of boat heading
        - Gate center must be AHEAD of the boat
        - Distance between red and green must be < max_gate_width
        
        Without heading: falls back to nearest pairing within max_gate_width.
        """
        red_ents = self.get_by_type("red_buoy") + self.get_by_type("red_pole_buoy")
        green_ents = self.get_by_type("green_buoy") + self.get_by_type("green_pole_buoy")
        
        # Debug logging
        print(f"[GATE DEBUG] Attempting pairing: red_ents={len(red_ents)}, green_ents={len(green_ents)}, has_heading={boat_heading_rad is not None}")
        
        if not red_ents or not green_ents:
            print(f"[GATE DEBUG] No pairing possible: missing red or green buoys")
            return []
        
        pairs: List[Tuple[Entity, Entity]] = []
        used = set()
        has_heading = boat_heading_rad is not None
        
        if has_heading:
            # Calculate forward and left vectors from heading
            fwd_x = math.cos(boat_heading_rad)
            fwd_y = math.sin(boat_heading_rad)
            left_x = -math.sin(boat_heading_rad)  # Perpendicular left
            left_y = math.cos(boat_heading_rad)
            
            if boat_pos is not None:
                bx, by = float(boat_pos[0]), float(boat_pos[1])
            else:
                s = self.get_start()
                bx, by = float(s[0]), float(s[1])
            
            # For each red buoy, find green buoy to its right
            for red in red_ents:
                rp = red.position
                best_green = None
                best_dist = float("inf")
                
                for i, green in enumerate(green_ents):
                    if id(green) in used:
                        continue
                    
                    # Vector from red to green
                    gvx = green.position[0] - rp[0]
                    gvy = green.position[1] - rp[1]
                    dist = math.hypot(gvx, gvy)
                    
                    if dist > max_gate_width or dist < 1e-6:
                        continue
                    
                    # Check if green is to the RIGHT of red (relative to heading)
                    # Cross product: (red→green) × left = positive means green is on right
                    cross_product = gvx * left_y - gvy * left_x
                    
                    # More lenient check: allow some tolerance
                    if cross_product < -1.0:  # Was <= 0 - now allows more tolerance
                        # Green is significantly on left - skip
                        print(f"[GATE DEBUG] Rejected: green on left (cross={cross_product:.2f})")
                        continue
                    
                    # Check if gate center is ahead of boat (very lenient)
                    gate_cx = (rp[0] + green.position[0]) * 0.5
                    gate_cy = (rp[1] + green.position[1]) * 0.5
                    to_gate_x = gate_cx - bx
                    to_gate_y = gate_cy - by
                    dot_ahead = to_gate_x * fwd_x + to_gate_y * fwd_y
                    
                    if dot_ahead < -15.0:  # Was -2.0 - now very lenient, allows gates well behind
                        # Gate is far behind boat
                        print(f"[GATE DEBUG] Rejected: gate behind (dot={dot_ahead:.2f})")
                        continue
                    
                    # Valid gate: green is on right, within distance, ahead of boat
                    if dist < best_dist:
                        best_dist = dist
                        best_green = green
                
                if best_green is not None:
                    pairs.append((red, best_green))
                    used.add(id(best_green))
        else:
            # Fallback without heading: nearest pairing
            for red in red_ents:
                rp = red.position
                best_green = None
                best_dist = float("inf")
                
                for i, green in enumerate(green_ents):
                    if id(green) in used:
                        continue
                    gvx = green.position[0] - rp[0]
                    gvy = green.position[1] - rp[1]
                    dist = math.hypot(gvx, gvy)
                    
                    if dist > max_gate_width or dist < 1e-6:
                        continue
                    
                    if dist < best_dist:
                        best_dist = dist
                        best_green = green
                
                if best_green is not None:
                    pairs.append((red, best_green))
                    used.add(id(best_green))
                    print(f"[GATE DEBUG] ✓ Fallback paired: red={rp[:2]}, green={best_green.position[:2]}, dist={best_dist:.2f}m")
        
        print(f"[GATE DEBUG] Total gates paired: {len(pairs)}")
        return pairs

    def get_gates(
        self,
        max_gate_width: float = 15.0,
        boat_heading_rad: Optional[float] = None,
        boat_pos: Optional[Tuple[float, float]] = None,
    ) -> List[Tuple[Vec2, Vec2]]:
        """Return list of (red_pos, green_pos) gate pairs.

        When boat_heading_rad is set: gates must be ahead and across the channel.
        """
        pairs = self._get_gate_entity_pairs(
            max_gate_width, boat_heading_rad=boat_heading_rad, boat_pos=boat_pos,
        )
        return [(r.position, g.position) for r, g in pairs]

    def _channel_direction(self, gate, boat_heading_rad: Optional[float]) -> Tuple[float, float]:
        """Compute channel direction (forward) for a gate. Uses gate perpendicular aligned with boat heading."""
        hdg = boat_heading_rad if boat_heading_rad is not None else self._boat_heading_for_no_go
        red, green = gate
        dx = float(green[0]) - float(red[0])
        dy = float(green[1]) - float(red[1])
        L = math.hypot(dx, dy)
        if L < 1e-6:
            if hdg is not None:
                return (math.cos(hdg), math.sin(hdg))
            return (0.0, 1.0)
        ux, uy = dx / L, dy / L
        perp_a = (-uy, ux)
        perp_b = (uy, -ux)
        if hdg is not None:
            fwd_x = math.cos(hdg)
            fwd_y = math.sin(hdg)
            dot_a = perp_a[0] * fwd_x + perp_a[1] * fwd_y
            return perp_a if dot_a >= 0 else perp_b
        return perp_a

    def _shorten_toward(self, ax: float, ay: float, bx: float, by: float, gap_m: float) -> Tuple[float, float]:
        """Return point gap_m before (bx, by) along segment a→b."""
        dx, dy = bx - ax, by - ay
        L = math.hypot(dx, dy)
        if L <= gap_m:
            return (ax, ay)
        ratio = (L - gap_m) / L
        return (ax + dx * ratio, ay + dy * ratio)

    def _gate_no_go_segments(
        self,
        gates: List[Tuple[Tuple[float, float, float], Tuple[float, float, float]]],
        boat_heading_rad: Optional[float] = None,
        default_extend_m: float = GATE_NO_GO_DEFAULT_EXTEND_M,
        gap_m: float = GATE_NO_GO_GAP_M,
    ) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
        """Build no-go wall segments connecting consecutive gates (red-to-red, green-to-green).

        Rules:
        - Inter-gate walls connect actual buoy positions (angle follows geometry).
        - Walls stop gap_m short of the next gate so they don't block entrance.
        - First gate: walls extend backward by default_extend_m.
        - Last gate: walls extend forward by default_extend_m (replaced when new gate appears).
        """
        segments: List[Tuple[Vec2, Vec2]] = []
        if not gates:
            return segments

        for i, (red, green) in enumerate(gates):
            r = (float(red[0]), float(red[1]))
            g = (float(green[0]), float(green[1]))
            fwd = self._channel_direction((red, green), boat_heading_rad)

            # Backward extension from first gate
            if i == 0:
                back = (-fwd[0], -fwd[1])
                segments.append((r, (r[0] + back[0] * default_extend_m, r[1] + back[1] * default_extend_m)))
                segments.append((g, (g[0] + back[0] * default_extend_m, g[1] + back[1] * default_extend_m)))

            # Inter-gate wall: connect this gate to the next gate (red-to-red, green-to-green)
            if i < len(gates) - 1:
                next_red, next_green = gates[i + 1]
                nr = (float(next_red[0]), float(next_red[1]))
                ng = (float(next_green[0]), float(next_green[1]))
                red_end = self._shorten_toward(r[0], r[1], nr[0], nr[1], gap_m)
                green_end = self._shorten_toward(g[0], g[1], ng[0], ng[1], gap_m)
                segments.append((r, red_end))
                segments.append((g, green_end))

            # Forward extension from last gate (replaced when new gate appears)
            if i == len(gates) - 1:
                segments.append((r, (r[0] + fwd[0] * default_extend_m, r[1] + fwd[1] * default_extend_m)))
                segments.append((g, (g[0] + fwd[0] * default_extend_m, g[1] + fwd[1] * default_extend_m)))

        return segments

    def _sample_segment(self, a: Vec2, b: Vec2, spacing_m: float) -> List[Vec2]:
        """Sample points along segment (a, b) every spacing_m."""
        ax, ay = float(a[0]), float(a[1])
        bx, by = float(b[0]), float(b[1])
        dx, dy = bx - ax, by - ay
        length = math.hypot(dx, dy)
        if length < 1e-6:
            return [a]
        n = max(1, int(length / spacing_m) + 1)
        return [(ax + (i / n) * dx, ay + (i / n) * dy) for i in range(n + 1)]

    def _gate_center(self, gate: Tuple[Tuple[float, float, float], Tuple[float, float, float]]) -> Tuple[float, float]:
        """Gate center (x, y) for ordering and comparison."""
        r, g = gate[0], gate[1]
        return ((r[0] + g[0]) * 0.5, (r[1] + g[1]) * 0.5)

    def _no_go_gates_equal(
        self,
        a: List[Tuple[Tuple[float, float, float], Tuple[float, float, float]]],
        b: List[Tuple[Tuple[float, float, float], Tuple[float, float, float]]],
        tol: float = 0.5,
    ) -> bool:
        """True if the two ordered gate lists represent the same sequence (by center distance)."""
        if len(a) != len(b):
            return False
        for ga, gb in zip(a, b):
            ca, cb = self._gate_center(ga), self._gate_center(gb)
            if math.hypot(ca[0] - cb[0], ca[1] - cb[1]) > tol:
                return False
        return True

    def get_no_go_obstacle_points(
        self,
        sample_spacing_m: float = GATE_NO_GO_SAMPLE_SPACING,
        map_bounds: Optional[Tuple[float, float]] = None,
        boat_heading_rad: Optional[float] = None,
    ) -> List[Vec2]:
        """Build no-go wall points from detected gates.

        Walls connect consecutive gates red-to-red and green-to-green, forming a
        dynamic corridor that adapts to actual gate geometry. Gates are ordered by
        forward projection along channel. As new gates appear, the corridor extends.
        """
        self._boat_heading_for_no_go = boat_heading_rad
        if boat_heading_rad is not None:
            gates_now = self.get_gates(boat_heading_rad=boat_heading_rad)
            start = self.get_start()
            s_x, s_y = float(start[0]), float(start[1])
            fwd_x = math.cos(boat_heading_rad)
            fwd_y = math.sin(boat_heading_rad)
            def key_along_fwd(g):
                cx = (g[0][0] + g[1][0]) * 0.5
                cy = (g[0][1] + g[1][1]) * 0.5
                return (cx - s_x) * fwd_x + (cy - s_y) * fwd_y
            ordered = sorted(gates_now, key=key_along_fwd)
        else:
            gates_now = self.get_gates()
            ordered = sorted(gates_now, key=lambda g: (g[0][1] + g[1][1]) * 0.5)

        if not ordered:
            self._no_go_gates = []
            points = list(self._last_no_go_points)
        else:
            if not self._no_go_gates_equal(ordered, self._no_go_gates):
                self._no_go_gates = list(ordered)
            points = []
            segments = self._gate_no_go_segments(
                self._no_go_gates, boat_heading_rad=boat_heading_rad,
            )
            for seg_a, seg_b in segments:
                points.extend(self._sample_segment(seg_a, seg_b, sample_spacing_m))
            self._last_no_go_points = points

        if map_bounds is not None and len(map_bounds) == 4:
            min_x, min_y, max_x, max_y = (float(v) for v in map_bounds)
            w = max_x - min_x
            h = max_y - min_y
            if w > 0 and h > 0:
                y = min_y
                while y <= max_y + 1e-9:
                    points.append((min_x, y))
                    points.append((max_x, y))
                    y += sample_spacing_m
                x = min_x
                while x <= max_x + 1e-9:
                    points.append((x, min_y))
                    points.append((x, max_y))
                    x += sample_spacing_m
                points = list(dict.fromkeys(points))
        return points

    def get_map_bounds(
        self,
        padding_m: float = 20.0,
    ) -> Optional[Tuple[float, float, float, float]]:
        """Map bounds as (min_x, min_y, max_x, max_y) with padding.
        
        Incremental extension strategy:
        1. First gate detected → create initial bounds (gate + 15m ahead + 30m behind)
        2. Second gate detected → extend bounds to include it
        3. Gates stable (within 2m) → return locked bounds
        4. Gates moved significantly (>2m) → unlock and restart from step 1
        """
        current_gates = self.get_gates()
        
        # No gates yet - return None (no bounds)
        if not current_gates:
            return None
        
        # Check if locked bounds are still valid
        if self._locked_map_bounds is not None and self._map_bounds_gate_count > 0:
            current_centers = [(float((r[0] + g[0]) / 2.0), float((r[1] + g[1]) / 2.0)) 
                               for r, g in current_gates[:self._map_bounds_gate_count]]
            
            # Check if gates still match locked positions (within 2m tolerance)
            if len(current_centers) == len(self._locked_map_bounds_gate_centers):
                if self._gates_match(current_centers, self._locked_map_bounds_gate_centers, tol_m=2.0):
                    # Check if we need to extend for a new gate
                    if len(current_gates) > self._map_bounds_gate_count:
                        # New gate detected - extend bounds
                        new_gate = current_gates[self._map_bounds_gate_count]
                        self._locked_map_bounds = self._extend_bounds_for_gate(
                            self._locked_map_bounds, new_gate, padding_m
                        )
                        new_center = (float((new_gate[0][0] + new_gate[1][0]) / 2.0),
                                     float((new_gate[0][1] + new_gate[1][1]) / 2.0))
                        self._locked_map_bounds_gate_centers.append(new_center)
                        self._map_bounds_gate_count += 1
                    
                    return self._locked_map_bounds
            
            # Gates moved significantly - unlock and recompute
            self._locked_map_bounds = None
            self._locked_map_bounds_gate_centers = []
            self._map_bounds_gate_count = 0
            self._map_bounds_first_gate_center = None
        
        # Create initial bounds from first gate
        gate1 = current_gates[0]
        red, green = gate1
        gate_center = ((red[0] + green[0]) / 2.0, (red[1] + green[1]) / 2.0)
        
        # Calculate forward direction (perpendicular to gate line)
        gate_vec_x = green[0] - red[0]
        gate_vec_y = green[1] - red[1]
        gate_len = math.hypot(gate_vec_x, gate_vec_y)
        
        if gate_len > 1e-6:
            # Forward = perpendicular to gate (choose direction aligned with heading if available)
            perp_a = (-gate_vec_y / gate_len, gate_vec_x / gate_len)
            perp_b = (gate_vec_y / gate_len, -gate_vec_x / gate_len)
            
            if self._boat_heading_for_no_go is not None:
                fwd_x = math.cos(self._boat_heading_for_no_go)
                fwd_y = math.sin(self._boat_heading_for_no_go)
                dot_a = perp_a[0] * fwd_x + perp_a[1] * fwd_y
                forward = perp_a if dot_a >= 0 else perp_b
            else:
                forward = perp_a
        else:
            forward = (0.0, 1.0)
        
        # Collect boundary points for initial bounds
        points = [
            (red[0], red[1]),
            (green[0], green[1]),
            # Backward extension 30m
            (red[0] - forward[0] * 30, red[1] - forward[1] * 30),
            (green[0] - forward[0] * 30, green[1] - forward[1] * 30),
            # Forward extension 15m (will be replaced by gate 2 if it appears)
            (red[0] + forward[0] * 15, red[1] + forward[1] * 15),
            (green[0] + forward[0] * 15, green[1] + forward[1] * 15),
        ]
        
        xs = [float(p[0]) for p in points]
        ys = [float(p[1]) for p in points]
        bounds = (min(xs) - padding_m, min(ys) - padding_m,
                  max(xs) + padding_m, max(ys) + padding_m)
        
        # Lock bounds with first gate
        self._locked_map_bounds = bounds
        self._locked_map_bounds_gate_centers = [(float(gate_center[0]), float(gate_center[1]))]
        self._map_bounds_gate_count = 1
        self._map_bounds_first_gate_center = gate_center
        
        # If second gate exists, extend immediately
        if len(current_gates) >= 2:
            gate2 = current_gates[1]
            self._locked_map_bounds = self._extend_bounds_for_gate(
                self._locked_map_bounds, gate2, padding_m
            )
            gate2_center = ((gate2[0][0] + gate2[1][0]) / 2.0,
                           (gate2[0][1] + gate2[1][1]) / 2.0)
            self._locked_map_bounds_gate_centers.append((float(gate2_center[0]), float(gate2_center[1])))
            self._map_bounds_gate_count = 2
        
        return self._locked_map_bounds
    
    def _extend_bounds_for_gate(
        self,
        current_bounds: Tuple[float, float, float, float],
        new_gate: Tuple[Vec2, Vec2],
        padding_m: float,
    ) -> Tuple[float, float, float, float]:
        """Extend existing bounds to include a new gate + forward extension."""
        min_x, min_y, max_x, max_y = current_bounds
        red, green = new_gate
        
        # Calculate forward direction from gate
        gate_vec_x = green[0] - red[0]
        gate_vec_y = green[1] - red[1]
        gate_len = math.hypot(gate_vec_x, gate_vec_y)
        
        if gate_len > 1e-6:
            perp_a = (-gate_vec_y / gate_len, gate_vec_x / gate_len)
            perp_b = (gate_vec_y / gate_len, -gate_vec_x / gate_len)
            
            if self._boat_heading_for_no_go is not None:
                fwd_x = math.cos(self._boat_heading_for_no_go)
                fwd_y = math.sin(self._boat_heading_for_no_go)
                dot_a = perp_a[0] * fwd_x + perp_a[1] * fwd_y
                forward = perp_a if dot_a >= 0 else perp_b
            else:
                forward = perp_a
        else:
            forward = (0.0, 1.0)
        
        # Points to include: gate buoys + forward extension 15m
        new_points = [
            (red[0], red[1]),
            (green[0], green[1]),
            (red[0] + forward[0] * 15, red[1] + forward[1] * 15),
            (green[0] + forward[0] * 15, green[1] + forward[1] * 15),
        ]
        
        # Extend bounds (never shrink, only grow)
        for px, py in new_points:
            min_x = min(min_x, px - padding_m)
            min_y = min(min_y, py - padding_m)
            max_x = max(max_x, px + padding_m)
            max_y = max(max_y, py + padding_m)
        
        return (min_x, min_y, max_x, max_y)
    
    def unlock_map_bounds(self) -> None:
        """Manually unlock map bounds. Next call to get_map_bounds() will recompute from current gates."""
        self._locked_map_bounds = None
        self._locked_map_bounds_gate_centers = []
        self._map_bounds_gate_count = 0
        self._map_bounds_first_gate_center = None
    
    def _gates_match(
        self, 
        centers_a: List[Tuple[float, float]], 
        centers_b: List[Tuple[float, float]], 
        tol_m: float = 2.0
    ) -> bool:
        """Check if two gate center lists are the same (within tolerance).
        Returns True if same number of gates and each center pair is within tol_m distance."""
        if len(centers_a) != len(centers_b):
            return False
        if not centers_a:
            return True
        # Sort both by x coordinate for consistent comparison
        sorted_a = sorted(centers_a, key=lambda p: p[0])
        sorted_b = sorted(centers_b, key=lambda p: p[0])
        for ca, cb in zip(sorted_a, sorted_b):
            dist = math.hypot(ca[0] - cb[0], ca[1] - cb[1])
            if dist > tol_m:
                return False
        return True

    def get_obstacles(
        self,
        exclude_entity_ids: Optional[set] = None,
    ) -> List[Vec3]:
        """Return obstacle positions for potential fields.

        Excludes goals, start, and any entity whose entity_id is in exclude_entity_ids
        (gate buoys that should not repel the boat).
        """
        obstacle_types = {
            "red_buoy", "green_buoy", "green_pole_buoy", "red_pole_buoy",
            "black_buoy",
            "yellow_buoy",
            "red_indicator", "green_indicator",
            "yellow_supply_drop", "black_supply_drop",
        }
        out: List[Vec3] = []
        for e in self.entities:
            if e.type not in obstacle_types:
                continue
            if exclude_entity_ids and e.entity_id is not None and e.entity_id in exclude_entity_ids:
                continue
            out.append(e.position)
        return out

    def get_gate_entity_ids(
        self,
        boat_heading_rad: Optional[float] = None,
        boat_pos: Optional[Tuple[float, float]] = None,
    ) -> set:
        """Return entity IDs of all buoys that form gate pairs (for exclusion from obstacles)."""
        pairs = self._get_gate_entity_pairs(boat_heading_rad=boat_heading_rad, boat_pos=boat_pos)
        ids = set()
        for r, g in pairs:
            if r.entity_id is not None:
                ids.add(r.entity_id)
            if g.entity_id is not None:
                ids.add(g.entity_id)
        return ids

    def get_black_buoys(self) -> List[Tuple[int, Vec3]]:
        """
        Return list of (entity_id, position) for all black_buoy entities (debris). Position in meters.
        Used for Task 2 debris reporting (lat/long to /gs_message_send).
        """
        out: List[Tuple[int, Vec2]] = []
        for e in self.entities:
            if e.type != "black_buoy":
                continue
            eid = e.entity_id if e.entity_id is not None else 0
            out.append((eid, e.position))
        return out

    def to_detections(self) -> List[DetectedEntity]:
        """
        Return list of DetectedEntity for all non-goal entities.
        Used by TaskMaster.get_detections. Uses Entity.entity_id when set, else parses from name.
        """
        out: List[DetectedEntity] = []
        for e in self.entities:
            if e.type == "goal":
                continue
            bid = e.entity_id if e.entity_id is not None else 0
            if bid == 0 and e.name:
                parts = e.name.split("_")
                if len(parts) >= 2 and parts[-1].isdigit():
                    bid = int(parts[-1])
            out.append(DetectedEntity(entity_id=bid, entity_type=e.type, position=e.position))
        return out
