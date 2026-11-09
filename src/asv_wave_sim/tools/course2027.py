"""RoboBoat 2027 course (Core tiers) for generate_course.py; config course/roboboat2027.yaml.

Built from the 2027 Primer & Task Ideas (the only 2027 document so far): task concepts and a course
sketch, no dimensions or scoring. The YAML marks every guessed value PLACEHOLDER.

  layout full       Tasks 1-5 (Safe Passage, Emergency Response Sprint, Navigate the Marina,
                    Delivery Run, Dynamic Response)
  layout readiness  the Proof of Readiness gates alone (two red/green gates)

Tasks 4 and 5 are started by RoboCommand messages. The answer key's `tasking` list holds them
(time after the run starts + message); the sim judge sends them on its tasking topic.
"""

import math

from generate_course import Course, vec


def polyline_length(pts):
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


def point_along(pts, s):
    """Point and unit tangent at arc length s along the polyline."""
    for a, b in zip(pts, pts[1:]):
        seg = math.dist(a, b)
        if s <= seg or b is pts[-1]:
            f = min(1.0, s / seg) if seg else 0.0
            tx, ty = (b[0] - a[0]) / seg, (b[1] - a[1]) / seg
            return (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])), (tx, ty)
        s -= seg
    raise ValueError('empty polyline')


def distance_to_polyline(p, pts):
    best = math.inf
    for a, b in zip(pts, pts[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        L2 = dx * dx + dy * dy
        f = 0.0 if not L2 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
        best = min(best, math.dist(p, (a[0] + f * dx, a[1] + f * dy)))
    return best


def convex_hull(points):
    pts = sorted(set(points))
    if len(pts) < 3:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def grow(poly, margin):
    """Push each vertex of a convex polygon away from its centroid by margin."""
    cx = sum(p[0] for p in poly) / len(poly)
    cy = sum(p[1] for p in poly) / len(poly)
    out = []
    for x, y in poly:
        d = math.hypot(x - cx, y - cy) or 1.0
        out.append([round(x + (x - cx) / d * margin, 4), round(y + (y - cy) / d * margin, 4)])
    return out


class Course2027(Course):
    def __init__(self, cfg, seed, layout='full'):
        super().__init__(cfg, seed)
        self.layout = layout
        self.DEFAULT_START = 'R' if layout == 'readiness' else 'course'
        self.tasking = []

    def jitter(self, key, u, v):
        j = float(self.rand_cfg.get(key, 0.0) or 0.0)
        if not j:
            return u, v
        return u + self.rng.uniform(-j, j), v + self.rng.uniform(-j, j)

    def min_sep(self):
        return float(self.cfg.get('randomize', {}).get('min_separation', 1.5))

    # ------------------------------------------------------------------ build
    def build(self, start_task=None):
        if self.layout == 'readiness':
            self.readiness()
        else:
            self.task1()
            self.task2()
            self.task3()
            self.task4()
            self.task5()
        self.boundary()

        starts_uv = {}
        if self.layout == 'readiness':
            starts_uv['R'] = self.readiness_start
        else:
            s = self.cfg['start']
            starts_uv['course'] = (s['u'], s['v'], s['yaw'])
            d = float(self.cfg.get('task_start_standoff', 3.0))
            for task, (u, v, yaw) in self.entries.items():
                fx, fy = vec(yaw)
                starts_uv[task] = (u - fx * d, v - fy * d, yaw)
            # Tasks 4 and 5 interrupt whatever the boat is doing: they start from the course start.
            starts_uv['4'] = starts_uv['5'] = starts_uv['course']
        self.starts = {}
        for task, (u, v, yaw) in sorted(starts_uv.items()):
            x, y = self.frame.enu(u, v)
            self.starts[task] = {'x': round(x, 4), 'y': round(y, 4), 'yaw': round(self.frame.yaw(yaw), 5)}
        self.pick_start(start_task)
        self.schedule_tasking()

    # ------------------------------------------------------------------ Task 1: Safe Passage
    def task1(self):
        t = self.cfg['tasks']['task1']
        eu, ev = self.jitter('task1_endpoint_jitter', t['entry']['u'], t['entry']['v'])
        xu, xv = self.jitter('task1_endpoint_jitter', t['exit']['u'], t['exit']['v'])
        path = [(eu, ev)] + [self.jitter('task1_path_jitter', *p) for p in t['path']] + [(xu, xv)]
        entry = self.add(1, 'light_buoy', 'blue', eu, ev, role='entry')
        exit_ = self.add(1, 'light_buoy', 'blue', xu, xv, role='exit')

        colors = list(t['markers'])
        if self.random and self.rng.random() < 0.5:
            colors = ['red' if c == 'green' else 'green' if c == 'red' else c for c in colors]
        hw, clear = float(t['passage_half_width']), float(t['end_clearance'])
        L = polyline_length(path)
        a, b = clear, L - clear
        markers, placed = [], [(eu, ev), (xu, xv)]
        for k, color in enumerate(colors):
            s = a + (b - a) * (k / (len(colors) - 1) if len(colors) > 1 else 0.5)
            (pu, pv), (tx, ty) = point_along(path, s)
            lx, ly = -ty, tx                              # left of the direction of travel = port
            side = 'port' if color == 'green' else 'starboard'
            sgn = 1.0 if side == 'port' else -1.0
            mu, mv = pu + sgn * lx * hw, pv + sgn * ly * hw
            e = self.add(1, 'light_buoy', color, mu, mv, role=f'{color}_marker')
            markers.append({'id': e['id'], 'color': color, 'side': side})
            placed.append((mu, mv))

        if self.random:
            lo, hi = t['region']['u'], t['region']['v']
            n = self.rng.randint(*self.rand_cfg.get('task1_black_count', [2, 3]))
            blacks = []
            for _ in range(500):
                if len(blacks) == n:
                    break
                p = (self.rng.uniform(*lo), self.rng.uniform(*hi))
                if (distance_to_polyline(p, path) >= float(t['black_path_clearance'])
                        and all(math.dist(p, q) >= self.min_sep() for q in placed)):
                    blacks.append(p)
                    placed.append(p)
        else:
            blacks = [tuple(p) for p in t['black']]
        black_ids = [self.add(1, 'light_buoy', 'black', u, v, role='black_buoy')['id'] for u, v in blacks]

        self.tasks['1'] = {
            'name': 'Safe Passage', 'entry_id': entry['id'], 'exit_id': exit_['id'],
            'markers': markers, 'black_ids': black_ids,
            'path': [[round(c, 4) for c in self.frame.enu(u, v)] for u, v in path],
        }
        # Start behind the ENTRY buoy, facing along the first leg of the passage.
        self.entries['1'] = (eu, ev, math.degrees(math.atan2(path[1][1] - ev, path[1][0] - eu)))

    # ------------------------------------------------------------------ Task 2: Emergency Response Sprint
    def task2(self):
        t = self.cfg['tasks']['task2']
        buoys = {}
        for b in t['buoys']:
            u, v = self.jitter('task2_jitter', b['u'], b['v'])
            buoys[b['color']] = self.add(2, 'a2', b['color'], u, v, role=f'{b["color"]}_buoy')
        target = t['target']
        if self.rand_cfg.get('task2_target'):
            target = self.rng.choice(t['target_choices'])
        numbers = [b['number'] for b in t['beacons']]
        green = next(b['number'] for b in t['beacons'] if b['light'] == 'green')
        if self.rand_cfg.get('task2_green_beacon'):
            green = self.rng.choice(numbers)
        beacons, start = [], None
        for b in t['beacons']:
            light = 'green' if b['number'] == green else 'red'
            e = self.add(2, 'light_buoy', light, b['u'], b['v'], number=b['number'], role=f'beacon{b["number"]}')
            beacons.append({'id': e['id'], 'number': b['number'], 'light': light})
            if light == 'green':
                start = e
        pts = [(e['x'], e['y']) for e in self.elements if e.get('task') == 2 and 'x' in e]
        field = grow(convex_hull(pts), float(t['exit_margin']))
        tgt = buoys[target]
        self.tasks['2'] = {
            'name': 'Emergency Response Sprint', 'target_color': target, 'target_id': tgt['id'],
            'start_id': start['id'], 'start_number': green, 'beacons': beacons,
            'buoy_ids': [e['id'] for e in buoys.values()], 'field': field,
            'start_radius': float(t['start_radius']),
        }
        # Start behind the green beacon, facing the target buoy.
        su, sv = self.frame.course(start['x'], start['y'])
        gu, gv = self.frame.course(tgt['x'], tgt['y'])
        self.entries['2'] = (su, sv, math.degrees(math.atan2(gv - sv, gu - su)))

    # ------------------------------------------------------------------ Task 3: Navigate the Marina
    def task3(self):
        t = self.cfg['tasks']['task3']
        self.marina(t, 3)
        c = self.cfg['elements']['dock_cube']['size'][0]
        dock = t['docks'][0]
        spine_u = t['origin']['u'] + dock['side'] * t['spine_centre_offset']
        tip_u = spine_u - dock['side'] * (t['spine_width_cubes'] * c / 2 + t['tine_length_cubes'] * c)
        rows = t['tine_rows']
        mid = len(rows) // 2 - 1                         # middle slip
        r0, r1 = rows[mid] + 1, rows[mid + 1] - 1
        v_mid = t['origin']['v'] - (r0 + r1 + 1) / 2 * c
        d = float(self.cfg.get('task_start_standoff', 3.0))
        heading = 0.0 if dock['side'] > 0 else 180.0     # toward the spine
        fx = math.cos(math.radians(heading))
        # Start `approach` m out from the tine tips, facing the slips.
        self.entries['3'] = (tip_u - fx * float(t['approach']) + fx * d, v_mid, heading)

    # ------------------------------------------------------------------ Task 4: Delivery Run
    def task4(self):
        t = self.cfg['tasks']['task4']
        shapes = [v['shape'] for v in t['vessels']]
        if self.rand_cfg.get('vessel_shapes'):
            base = ['cross', 'circle', 'square', 'triangle']
            shapes = base + [self.rng.choice(base) for _ in range(len(shapes) - len(base))]
            self.rng.shuffle(shapes)
        vessels = []
        for vs, shape in zip(t['vessels'], shapes):
            e = self.add(4, 'vessel', vs['hull'], vs['u'], vs['v'], yaw=vs['yaw'], shape=shape,
                         role=f'delivery_{shape}')
            vessels.append({'id': e['id'], 'shape': shape, 'hull': vs['hull']})
        # RoboCommand names the vessel by its shape, so the assigned shape must be unique.
        unique = [v for v in vessels if shapes.count(v['shape']) == 1]
        if self.rand_cfg.get('task4_assignment'):
            assigned = self.rng.choice(unique)
        else:
            assigned = next(v for v in unique if v['shape'] == t['assigned'])
        self.tasks['4'] = {
            'name': 'Delivery Run', 'vessels': vessels, 'assigned': [assigned['id']],
            'assigned_shapes': [assigned['shape']],
            'delivery_range': float(t['delivery_range']), 'delivery_hold': float(t['delivery_hold']),
        }

    # ------------------------------------------------------------------ Task 5: Dynamic Response
    def task5(self):
        t = self.cfg['tasks']['task5']
        x, y = self.frame.enu(t['rally']['u'], t['rally']['v'])
        lat, lon = self.frame.latlon(x, y)
        self.tasks['5'] = {
            'name': 'Dynamic Response',
            'rally': {'x': round(x, 4), 'y': round(y, 4), 'lat': round(lat, 8), 'lon': round(lon, 8)},
            'rally_radius': float(t['rally_radius']),
        }

    # ------------------------------------------------------------------ Proof of Readiness
    def readiness(self):
        t = self.cfg['tasks']['readiness']
        w = self.pick('readiness_gates', t['gate_width'], t['gate_width_range'])
        spacing = self.pick('readiness_gates', t['gate_spacing'], t['gate_spacing_range'])
        fx, fy = vec(t['yaw'])
        g1u, g1v = t['gate1']['u'], t['gate1']['v']
        g1, _, _ = self.add_gate('R', 'gate1', t['gate_model'], g1u, g1v, t['yaw'], w)
        g2, _, _ = self.add_gate('R', 'gate2', t['gate_model'], g1u + fx * spacing, g1v + fy * spacing, t['yaw'], w)
        self.tasks['R'] = {'name': 'Proof of Readiness', 'gates': [g1['label'], g2['label']],
                           'gate_width': round(w, 3), 'gate_spacing': round(spacing, 3)}
        b = float(t['start_before'])
        self.readiness_start = (g1u - fx * b, g1v - fy * b, t['yaw'])

    # ------------------------------------------------------------------ RoboCommand tasking
    def schedule_tasking(self):
        """Messages the sim's RoboCommand stand-in sends during the run. PLACEHOLDER format: the
        2027 RoboCommand tasking messages are not published yet (SIMULATION.md, section 4)."""
        self.tasking = []
        if '2' in self.tasks and self.start_task in ('course', '2'):
            t2 = self.tasks['2']
            self.tasking.append({'at_s': 0.0, 'task': '2', 'message': {
                'type': 'SPRINT_TARGET', 'task': 2, 'target_color': t2['target_color']}})
        for task, kind in (('4', 'DELIVERY'), ('5', 'RALLY')):
            if task not in self.tasks:
                continue
            at = self.cfg['tasks'][f'task{task}'].get('tasking_at', {})
            when = at.get('course') if self.start_task == 'course' else at.get('task') if self.start_task == task else None
            if when is None:
                continue
            if kind == 'DELIVERY':
                t4 = self.tasks['4']
                msg = {'type': 'DELIVERY', 'task': 4, 'targets': [{'shape': s} for s in t4['assigned_shapes']]}
            else:
                r = self.tasks['5']['rally']
                msg = {'type': 'RALLY', 'task': 5, 'lat': r['lat'], 'lon': r['lon']}
            self.tasking.append({'at_s': float(when), 'task': task, 'message': msg})

    def key_extra(self):
        return {'year': 2027, 'layout': self.layout, 'tasking': self.tasking}

    def summary(self):
        if self.layout == 'readiness':
            r = self.tasks['R']
            return f'Proof of Readiness gates {r["gate_width"]} m wide, {r["gate_spacing"]} m apart'
        t1, t2, t3, t4 = (self.tasks[k] for k in '1234')
        green = ', '.join(f'{c["dock"]}{c["slip"]}' for c in t3['green']) or 'none'
        tasking = ', '.join(f'{m["message"]["type"]}@{m["at_s"]:.0f}s' for m in self.tasking) or 'none'
        return (f'task1 {len(t1["markers"])} markers + {len(t1["black_ids"])} black; task2 target '
                f'{t2["target_color"]} from beacon {t2["start_number"]}; task3 green slip {green}; '
                f'task4 assigned {", ".join(t4["assigned_shapes"])}; tasking {tasking}')
