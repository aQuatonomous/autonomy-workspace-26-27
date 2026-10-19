"""RoboBoat 2026 referee for Tasks 1, 2, 3 and 5.

Judging uses Gazebo ground truth (boat pose in the world ENU frame) and the course answer key
written by tools/generate_course.py. Team reports arrive through the RoboCommand stand-in and are
matched against the true object positions. Points follow the 2026 handbook scoring tables; places
where the handbook leaves room for interpretation are named in INTERPRETATIONS and in the report.
"""

import math
import re
from collections import deque

from . import geometry as geo
from . import report_compat as pb

STRIKE_POINTS = 25
MAX_STRIKES = 20
FULL_CIRCLE = 2.0 * math.pi - 0.2     # winding needed to count as circling a buoy (~349 deg)
SPEED_LOOP = math.radians(300.0)      # Task 3 loop around the yellow buoy, gate to gate
STRIKE_REARM_S = 1.0                  # separation time before the same object can strike again
GATE_DEBOUNCE_S = 1.0

INTERPRETATIONS = [
    'Task 1 advanced capability = returning through both gates (exit).',
    'Task 5 docking tiers are exclusive: the best of 50 (unoccupied) / 200 (green) / 350 (green, '
    'lowest number) is awarded, plus 100 for the matching dock/slip report.',
    'Task 2 red indicator: 100 points need a correct location report, no strike and no full circle.',
    'Task 3 report: 100 points need the indicator colour (ObjectDetected) plus SPEED_START and '
    'SPEED_END GatePass reports.',
    'Gate passes use the boat centre crossing between the two gate buoys; touching a buoy is a strike.',
    'Leaving the course boundary (boat centre) terminates the run; scoring freezes at that moment.',
]

COLOR_NAMES = {'red': pb.Color.COLOR_RED, 'green': pb.Color.COLOR_GREEN,
               'yellow': pb.Color.COLOR_YELLOW, 'black': pb.Color.COLOR_BLACK}


class Winding:
    """Signed bearing swept by the boat around a point while it stays within radius."""

    def __init__(self, c, radius):
        self.c, self.radius = c, radius
        self.reset()

    def reset(self):
        self.prev, self.total, self.best = None, 0.0, 0.0

    def update(self, x, y):
        dx, dy = x - self.c[0], y - self.c[1]
        if math.hypot(dx, dy) > self.radius:
            self.prev, self.total = None, 0.0
            return
        b = math.atan2(dy, dx)
        if self.prev is not None:
            self.total += geo.wrap(b - self.prev)
            if abs(self.total) > abs(self.best):
                self.best = self.total
        self.prev = b


def _dock_letter(text):
    m = re.search(r'\b(N|S|NORTH|SOUTH)\b', str(text).upper())
    return m.group(1)[0] if m else str(text).strip().upper()[:1]


def _slip_number(text):
    m = re.search(r'\d+', str(text))
    return int(m.group(0)) if m else None


class Referee:
    def __init__(self, key, boat_length=1.2, boat_width=0.9, report_tol=3.0, gate_report_tol=10.0,
                 circle_radius=10.0, dock_hold=3.0):
        self.key = key
        self.boat_length, self.boat_width = boat_length, boat_width
        self.report_tol, self.gate_report_tol = report_tol, gate_report_tol
        self.dock_hold = dock_hold
        d = key['datum']
        self.lat0, self.lon0, self.m_per_deg = d['lat'], d['lon'], d.get('m_per_deg', 111318.84502145034)
        self.el = {e['id']: e for e in key['elements']}
        self.boundary = [tuple(p) for p in key['boundary']]
        self.tasks = key['tasks']

        self.gates = {}
        for g in key['gates']:
            r, gr = self.el[g['red_id']], self.el[g['green_id']]
            self.gates[g['label']] = dict(g, red=(r['x'], r['y']), green=(gr['x'], gr['y']))
        self.gate_last = {}

        self.obstacles = []
        for e in key['elements']:
            if e['kind'] == 'vessel':
                self.obstacles.append((e, 'poly', [tuple(p) for p in e['footprint']]))
            elif e.get('radius') and e['kind'] in ('sur_mark', 'a0', 'a2', 'indicator_buoy'):
                self.obstacles.append((e, 'circle', ((e['x'], e['y']), e['radius'])))
        self.contact = {}
        self.strikes = []

        self.windings = {}
        t2 = self.tasks.get('2')
        if t2:
            for ind in t2['indicators']:
                e = self.el[ind['id']]
                self.windings[ind['id']] = Winding((e['x'], e['y']), circle_radius)
        t3 = self.tasks.get('3')
        if t3:
            y = self.el[t3['yellow_id']]
            self.t3_winding = Winding((y['x'], y['y']), min(circle_radius, 0.8 * t3['distance']))

        self.t = None
        self.pose = None
        self.first_pose = None
        self.run_start = None
        self.terminated = None
        self.events = []

        self.t1 = {'g1': None, 'entry': None, 'g2_rev': None, 'exit': None}
        self.t2 = {'fwd_idx': 0, 'entered': None, 'rev_idx': 0, 'exited': None}
        self.t3 = {'start': None, 'end': None, 'winding': 0.0}
        self.t5 = {'inside': {}, 'docked': []}

        self.reports = []
        self.heartbeats = []
        self.gate_reports = []
        self.detections = []
        self.dockings = []
        self.protocol = {'frames': 0, 'errors': [], 'rate_violations': 0, 'seq_regressions': 0,
                         'missing_fields': 0, 'clock_skew': [], 'hb_pos_err': [], 'hb_tasks': {},
                         'hb_states': {}, 'peers': set()}
        self._recent = deque()
        self._last_seq = None

    # ------------------------------------------------------------------ helpers
    def xy(self, lat, lon):
        return ((lon - self.lon0) * self.m_per_deg * math.cos(math.radians(self.lat0)),
                (lat - self.lat0) * self.m_per_deg)

    def _event(self, text):
        self.events.append((self.t, text))

    @property
    def active(self):
        return self.terminated is None

    # ------------------------------------------------------------------ ground truth
    def on_pose(self, t, x, y, yaw):
        if not self.active:
            return
        prev = self.pose
        self.t, self.pose = t, (x, y, yaw)
        if self.first_pose is None:
            self.first_pose = (x, y)
            return
        if self.run_start is None:
            if math.hypot(x - self.first_pose[0], y - self.first_pose[1]) < 1.0:
                return
            self.run_start = t
            self._event('run started (boat moved 1 m from the start position)')
        if prev is None or math.hypot(x - prev[0], y - prev[1]) > 5.0:
            return

        self._check_gates((prev[0], prev[1]), (x, y))
        self._check_contacts(x, y, yaw)
        for w in self.windings.values():
            w.update(x, y)
        if self.t3['start'] is not None and self.t3['end'] is None:
            self.t3_winding.update(x, y)
        self._check_slips(x, y)

        if not geo.point_in_poly((x, y), self.boundary):
            self.terminated = t
            self._event('boat left the course boundary: run terminated')

    def _check_gates(self, p0, p1):
        for label, g in self.gates.items():
            if not geo.seg_intersect(p0, p1, g['red'], g['green']):
                continue
            last = self.gate_last.get(label)
            if last is not None and self.t - last < GATE_DEBOUNCE_S:
                continue
            self.gate_last[label] = self.t
            h = (math.cos(g['heading']), math.sin(g['heading']))
            fwd = (p1[0] - p0[0]) * h[0] + (p1[1] - p0[1]) * h[1] > 0
            self._event(f'crossed {label} {"forward" if fwd else "in reverse"}')
            self._on_gate(g['task'], label, fwd)

    def _on_gate(self, task, label, fwd):
        t = self.t
        if task == 1:
            s = self.t1
            if label == 'gate1' and fwd and s['entry'] is None:
                s['g1'] = t
            elif label == 'gate2' and fwd and s['g1'] is not None and s['entry'] is None:
                s['entry'] = t
                self._event('Task 1: entry gates complete')
            elif label == 'gate2' and not fwd and s['entry'] is not None and s['exit'] is None:
                s['g2_rev'] = t
            elif label == 'gate1' and not fwd and s['g2_rev'] is not None and s['exit'] is None:
                s['exit'] = t
                self._event('Task 1: exit gates complete')
        elif task == 2:
            s, order = self.t2, self.tasks['2']['gates']
            i = order.index(label)
            if fwd and s['entered'] is None:
                if i == s['fwd_idx']:
                    s['fwd_idx'] += 1
                elif i == 0:
                    s['fwd_idx'] = 1
                if s['fwd_idx'] == len(order):
                    s['entered'] = t
                    self._event('Task 2: entered the debris field through all channel gates')
            elif not fwd and s['entered'] is not None and s['exited'] is None:
                if i == len(order) - 1 - s['rev_idx']:
                    s['rev_idx'] += 1
                elif i == len(order) - 1:
                    s['rev_idx'] = 1
                if s['rev_idx'] == len(order):
                    s['exited'] = t
                    self._event('Task 2: exited through all channel gates')
        elif task == 3:
            s = self.t3
            if fwd and s['end'] is None:
                s['start'] = t
                self.t3_winding.reset()
                self._event('Task 3: timer started at the speed gate')
            elif not fwd and s['start'] is not None and s['end'] is None:
                if abs(self.t3_winding.best) >= SPEED_LOOP:
                    s['end'] = t
                    s['winding'] = self.t3_winding.best
                    self._event(f'Task 3: loop complete in {t - s["start"]:.1f} s '
                                f'({"counter-clockwise" if s["winding"] > 0 else "clockwise"})')

    def _check_contacts(self, x, y, yaw):
        hull = geo.rect(x, y, yaw, self.boat_length, self.boat_width)
        reach = math.hypot(self.boat_length, self.boat_width) / 2.0
        for e, kind, shape in self.obstacles:
            if math.hypot(e['x'] - x, e['y'] - y) > reach + 3.0:
                touching = False
            elif kind == 'circle':
                touching = geo.poly_circle(hull, shape[0], shape[1])
            else:
                touching = geo.poly_poly(hull, shape)
            st = self.contact.setdefault(e['id'], {'touching': False, 'released': None})
            if touching and not st['touching']:
                if st['released'] is None or self.t - st['released'] > STRIKE_REARM_S:
                    self.strikes.append({'t': self.t, 'id': e['id'], 'name': e['name'], 'task': e['task']})
                    self._event(f'STRIKE: contact with {e["name"]}')
            if st['touching'] and not touching:
                st['released'] = self.t
            st['touching'] = touching

    def _check_slips(self, x, y):
        t5 = self.tasks.get('5')
        if not t5:
            return
        for s in t5['slips']:
            k = (s['dock'], s['slip'])
            if geo.point_in_poly((x, y), [tuple(p) for p in s['polygon']]):
                since = self.t5['inside'].setdefault(k, self.t)
                if self.t - since >= self.dock_hold and k not in [d['slip'] for d in self.t5['docked']]:
                    self.t5['docked'].append({'t': self.t, 'slip': k, 'info': s})
                    self._event(f'Task 5: docked in {s["dock"]}{s["slip"]} ({s["indicator"]} indicator'
                                f'{", occupied" if s["occupied"] else ""})')
            else:
                self.t5['inside'].pop(k, None)

    # ------------------------------------------------------------------ reports
    def on_error(self, wall, message, peer):
        self.protocol['errors'].append({'wall': wall, 'peer': peer, 'error': message})

    def on_report(self, wall, report, peer):
        p = self.protocol
        p['frames'] += 1
        p['peers'].add(peer.rsplit(':', 1)[0])
        self._recent.append(wall)
        while self._recent and wall - self._recent[0] > 1.0:
            self._recent.popleft()
        if len(self._recent) > 5:
            p['rate_violations'] += 1
        if not report.team_id or not report.vehicle_id or not report.HasField('sent_at'):
            p['missing_fields'] += 1
        if report.HasField('sent_at'):
            p['clock_skew'].append(report.sent_at.seconds + report.sent_at.nanos * 1e-9 - wall)
        if self._last_seq is not None and report.seq < self._last_seq:
            p['seq_regressions'] += 1
        self._last_seq = report.seq

        body = report.WhichOneof('body')
        entry = {'wall': wall, 't': self.t, 'body': body, 'seq': report.seq}
        if not self.active:
            entry['ignored'] = 'after run termination'
            self.reports.append(entry)
            return
        if body == 'heartbeat':
            hb = report.heartbeat
            self.heartbeats.append(wall)
            task = pb.TaskType.Name(hb.current_task) if hb.current_task in pb.TaskType.values() else str(hb.current_task)
            state = pb.RobotState.Name(hb.state) if hb.state in pb.RobotState.values() else str(hb.state)
            p['hb_tasks'][task] = p['hb_tasks'].get(task, 0) + 1
            p['hb_states'][state] = p['hb_states'].get(state, 0) + 1
            if self.pose and hb.HasField('position') and hb.position.latitude:
                hx, hy = self.xy(hb.position.latitude, hb.position.longitude)
                p['hb_pos_err'].append(math.hypot(hx - self.pose[0], hy - self.pose[1]))
        elif body == 'gate_pass':
            gp = report.gate_pass
            xy = self.xy(gp.position.latitude, gp.position.longitude)
            gtype = pb.GateType.Name(gp.type) if gp.type in pb.GateType.values() else str(gp.type)
            self.gate_reports.append({'t': self.t, 'type': gtype, 'xy': xy})
            entry['detail'] = gtype
            self._event(f'report: GatePass {gtype}')
        elif body == 'object_detected':
            od = report.object_detected
            xy = self.xy(od.position.latitude, od.position.longitude)
            rec = {'t': self.t, 'type': pb.ObjectType.Name(od.object_type) if od.object_type in pb.ObjectType.values() else str(od.object_type),
                   'color': pb.Color.Name(od.color) if od.color in pb.Color.values() else str(od.color),
                   'color_value': od.color, 'xy': xy, 'object_id': od.object_id,
                   'task_context': pb.TaskType.Name(od.task_context) if od.task_context in pb.TaskType.values() else str(od.task_context)}
            near = self._nearest_object(xy)
            rec['nearest'] = near
            self.detections.append(rec)
            entry['detail'] = f'{rec["color"]} {rec["type"]}'
            self._event(f'report: ObjectDetected {rec["color"]} {rec["type"]}'
                        + (f' ({near[1]:.1f} m from {self.el[near[0]]["name"]})' if near else ''))
        elif body == 'docking':
            dk = report.docking
            self.dockings.append({'t': self.t, 'dock': dk.dock, 'slip': dk.slip})
            entry['detail'] = f'{dk.dock} {dk.slip}'
            self._event(f'report: Docking dock={dk.dock!r} slip={dk.slip!r}')
        self.reports.append(entry)

    def _nearest_object(self, xy):
        best = None
        for e in self.el.values():
            if 'x' not in e or e['kind'] in ('banner', 'dock_indicator', 'dock'):
                continue
            d = math.hypot(e['x'] - xy[0], e['y'] - xy[1])
            if best is None or d < best[1]:
                best = (e['id'], d)
        return best

    def _reported(self, eid, extra=None):
        e = self.el[eid]
        want = COLOR_NAMES.get(e['color'])
        for r in self.detections:
            if r['color_value'] != want:
                continue
            if math.hypot(r['xy'][0] - e['x'], r['xy'][1] - e['y']) <= self.report_tol:
                return True
            if extra and extra(r):
                return True
        return False

    # ------------------------------------------------------------------ scoring
    def score(self, wall_now=None):
        tasks = {}
        struck = {s['id'] for s in self.strikes}

        def item(key, desc, pts, earned, detail=''):
            return {'key': key, 'desc': desc, 'max': pts, 'points': earned, 'detail': detail}

        if '1' in self.tasks:
            s = self.t1
            g1, g2 = self.gates['gate1'], self.gates['gate2']
            entry_rep = any(r['type'] == 'GATE_ENTRY' and s['entry'] is not None and r['t'] >= s['entry'] - 2.0
                            and math.dist(r['xy'], g2['center']) <= self.gate_report_tol for r in self.gate_reports)
            exit_rep = any(r['type'] == 'GATE_EXIT' and s['exit'] is not None and r['t'] >= s['exit'] - 2.0
                           and math.dist(r['xy'], g1['center']) <= self.gate_report_tol for r in self.gate_reports)
            n_entry = sum(r['type'] == 'GATE_ENTRY' for r in self.gate_reports)
            n_exit = sum(r['type'] == 'GATE_EXIT' for r in self.gate_reports)
            tasks['1'] = [
                item('entry', 'Through both entry gates', 100, 100 if s['entry'] is not None else 0),
                item('entry_report', 'GatePass ENTRY report (time + position)', 100, 100 if entry_rep else 0,
                     f'{n_entry} received'),
                item('exit', 'Back through both gates (exit)', 200, 200 if s['exit'] is not None else 0),
                item('exit_report', 'GatePass EXIT report (time + position)', 100, 100 if exit_rep else 0,
                     f'{n_exit} received'),
            ]

        if '2' in self.tasks:
            t2, s = self.tasks['2'], self.t2
            items = [
                item('enter', 'Enter through all channel gates', 100, 100 if s['entered'] is not None else 0,
                     f'{s["fwd_idx"]}/{len(t2["gates"])} gates'),
                item('exit', 'Exit through all channel gates', 100, 100 if s['exited'] is not None else 0,
                     f'{s["rev_idx"]}/{len(t2["gates"])} gates'),
            ]
            for color, pts, desc in (('red', 100, 'Red indicator: avoid (no circle) + report location'),
                                     ('green', 200, 'Green indicator: circle 360 deg + report location')):
                ids = [i['id'] for i in t2['indicators'] if i['color'] == color]
                if not ids:
                    continue
                details, ok = [], True
                for i in ids:
                    rep = self._reported(i)
                    circ = abs(self.windings[i].best) >= FULL_CIRCLE
                    details.append(f'#{i}: reported={"yes" if rep else "no"} '
                                   f'circled={math.degrees(abs(self.windings[i].best)):.0f}deg'
                                   + (' STRUCK' if i in struck else ''))
                    if color == 'red':
                        ok &= rep and not circ and i not in struck
                    else:
                        ok &= rep and circ
                items.append(item(color, desc, pts, pts if ok else 0, '; '.join(details)))
            found = [i for i in t2['debris_ids'] if self._reported(i)]
            items.append(item('all_debris', 'Report locations of all debris (indicators + black buoys)', 200,
                              200 if len(found) == len(t2['debris_ids']) else 0,
                              f'{len(found)}/{len(t2["debris_ids"])} located within {self.report_tol:.0f} m'))
            tasks['2'] = items

        if '3' in self.tasks:
            t3, s = self.tasks['3'], self.t3
            done = s['end'] is not None
            T = (s['end'] - s['start']) if done else None
            direction = None if not done else ('ccw' if s['winding'] > 0 else 'cw')
            ind = self.el[t3['indicator_id']]
            color_rep = self._reported(t3['indicator_id'], extra=lambda r: r['task_context'] == 'TASK_SPEED_CHALLENGE')
            types = {r['type'] for r in self.gate_reports}
            time_rep = 'GATE_SPEED_START' in types and 'GATE_SPEED_END' in types
            running = ''
            if s['start'] is not None and not done and self.t is not None:
                running = f'running {self.t - s["start"]:.0f} s, {math.degrees(abs(self.t3_winding.best)):.0f} deg around'
            tasks['3'] = [
                item('loop', 'Through gate, around yellow buoy, back through gate', 200, 200 if done else 0, running),
                item('direction', f'Correct direction ({ind["color"]} -> {t3["required_direction"]})', 150,
                     150 if done and direction == t3['required_direction'] else 0, direction or ''),
                item('time', 'Completion time (250 - T)', 250, max(0.0, min(250.0, 250.0 - T)) if done else 0,
                     f'T = {T:.1f} s' if done else ''),
                item('report', 'Report indicator colour + SPEED_START/END times', 100,
                     100 if color_rep and time_rep else 0,
                     f'colour={"yes" if color_rep else "no"} times={"yes" if time_rep else "no"}'),
            ]

        if '5' in self.tasks:
            t5 = self.tasks['5']
            correct = {(c['dock'], c['slip']) for c in t5['correct']}
            best, best_desc = 0, ''
            for d in self.t5['docked']:
                info = d['info']
                k = d['slip']
                pts = 350 if k in correct else 200 if info['indicator'] == 'green' else 50 if not info['occupied'] else 0
                if pts > best:
                    best, best_desc = pts, f'{k[0]}{k[1]}'
            docked = {d['slip'] for d in self.t5['docked']}
            rep = any((_dock_letter(r['dock']), _slip_number(r['slip'])) in docked for r in self.dockings)
            target = ', '.join(f'{c["dock"]}{c["slip"]}' for c in t5['correct'])
            tasks['5'] = [
                item('dock', f'Dock: unoccupied 50 / green 200 / green + lowest number 350 (target {target})',
                     350, best, best_desc or ('docked: ' + ', '.join(f'{a}{b}' for a, b in docked) if docked else '')),
                item('report', 'Docking report matches the slip used', 100, 100 if rep else 0,
                     f'{len(self.dockings)} received'),
            ]

        n_strikes = len(self.strikes)
        penalty = -STRIKE_POINTS * min(n_strikes, MAX_STRIKES)
        total = sum(i['points'] for items in tasks.values() for i in items) + penalty

        hb = self._heartbeat_stats(wall_now)
        q = self._qualifying(tasks, hb)
        return {
            'tasks': tasks,
            'strikes': {'count': n_strikes, 'penalty': penalty, 'list': self.strikes},
            'total': round(total, 1),
            'max_total': sum(i['max'] for items in tasks.values() for i in items),
            'heartbeat': hb,
            'qualifying': q,
            'protocol': self._protocol_summary(),
            'run': {'started': self.run_start, 'sim_time': self.t, 'terminated': self.terminated,
                    'elapsed': (self.t - self.run_start) if self.run_start is not None and self.t is not None else 0.0},
            'warnings': self._warnings(hb),
        }

    def _heartbeat_stats(self, wall_now):
        hbs = self.heartbeats
        gaps = [b - a for a, b in zip(hbs, hbs[1:])]
        stats = {'count': len(hbs), 'mean_interval': sum(gaps) / len(gaps) if gaps else None,
                 'max_gap': max(gaps) if gaps else None,
                 'since_last': (wall_now - hbs[-1]) if hbs and wall_now else None}
        stats['ok'] = bool(len(hbs) >= 2 and stats['mean_interval'] <= 1.5 and stats['max_gap'] <= 3.0
                           and (stats['since_last'] is None or stats['since_last'] <= 3.0))
        return stats

    def _qualifying(self, tasks, hb):
        def pts(task, key):
            return next((i['points'] for i in tasks.get(task, []) if i['key'] == key), 0)
        core = {
            '1': pts('1', 'entry') > 0,
            '2': pts('2', 'enter') > 0 and pts('2', 'exit') > 0,
            '3': pts('3', 'loop') > 0,
            '5': pts('5', 'dock') >= 50,
        }
        advanced = {
            '1': pts('1', 'exit') > 0,
            '2': pts('2', 'red') > 0 or pts('2', 'green') > 0,
            '3': pts('3', 'direction') > 0,
            '5': pts('5', 'dock') >= 200,
        }
        core = {k: v for k, v in core.items() if k in tasks}
        advanced = {k: v for k, v in advanced.items() if k in tasks}
        n_core, n_adv = sum(core.values()), sum(advanced.values())
        return {'core': core, 'advanced': advanced, 'sequence': n_core >= 2, 'heartbeat': hb['ok'],
                'qualified': n_core >= 2 and n_adv >= 2 and hb['ok']}

    def _protocol_summary(self):
        p = self.protocol
        skew = p['clock_skew']
        err = p['hb_pos_err']
        return {'frames': p['frames'], 'errors': len(p['errors']), 'rate_violations': p['rate_violations'],
                'seq_regressions': p['seq_regressions'], 'missing_fields': p['missing_fields'],
                'max_clock_skew_s': max((abs(s) for s in skew), default=None),
                'heartbeat_position_error_m': (sum(err) / len(err)) if err else None,
                'heartbeat_tasks': dict(p['hb_tasks']), 'heartbeat_states': dict(p['hb_states']),
                'peers': sorted(p['peers']), 'gate_reports': len(self.gate_reports),
                'object_reports': len(self.detections), 'docking_reports': len(self.dockings)}

    def _warnings(self, hb):
        p, w = self.protocol, []
        n = hb['count']
        if self.run_start is not None and n == 0:
            w.append('No heartbeat received (RoboCommand requires 1 Hz heartbeats).')
        if n and set(p['hb_tasks']) <= {'TASK_UNKNOWN'}:
            w.append(f'Heartbeat current_task was TASK_UNKNOWN in all {n} heartbeats.')
        if n and 'STATE_AUTO' not in p['hb_states']:
            w.append('Heartbeat state never reported STATE_AUTO.')
        if p['hb_pos_err'] and sum(p['hb_pos_err']) / len(p['hb_pos_err']) > 3.0:
            w.append(f'Heartbeat positions are {sum(p["hb_pos_err"]) / len(p["hb_pos_err"]):.1f} m from '
                     'ground truth on average (check the SITL home / datum).')
        if p['clock_skew'] and max(abs(s) for s in p['clock_skew']) > 1.0:
            w.append('Report timestamps differ from the judge clock by more than 1 s (sync with NTP).')
        if p['rate_violations']:
            w.append(f'{p["rate_violations"]} reports exceeded the 5 messages/s limit.')
        if p['missing_fields']:
            w.append(f'{p["missing_fields"]} reports lacked team_id, vehicle_id or sent_at.')
        if p['errors']:
            w.append(f'{len(p["errors"])} framing/decoding errors (first: {p["errors"][0]["error"]}).')
        far = [r for r in self.detections if r['nearest'] and r['nearest'][1] > self.report_tol]
        if far:
            w.append(f'{len(far)} ObjectDetected reports were more than {self.report_tol:.0f} m from any course object.')
        return w


def render(score, key):
    """Plain-text scoreboard."""
    lines = []
    run = score['run']
    status = 'TERMINATED (boundary)' if run['terminated'] is not None else 'running' if run['started'] is not None else 'waiting for the boat to move'
    lines.append(f'RoboBoat 2026 sim judge | seed {key.get("seed")} | {status} | '
                 f'elapsed {run["elapsed"]:.0f} s (sim)')
    lines.append(f'TOTAL {score["total"]:.0f} / {score["max_total"]}   strikes {score["strikes"]["count"]} '
                 f'({score["strikes"]["penalty"]})')
    for task, items in score['tasks'].items():
        name = key['tasks'][task].get('name', '')
        sub = sum(i['points'] for i in items)
        lines.append(f'Task {task} {name}: {sub:.0f} / {sum(i["max"] for i in items)}')
        for i in items:
            mark = 'x' if i['points'] > 0 else ' '
            lines.append(f'  [{mark}] {i["points"]:>5.0f}/{i["max"]:<4} {i["desc"]}'
                         + (f'  ({i["detail"]})' if i['detail'] else ''))
    hb = score['heartbeat']
    lines.append(f'Heartbeat: {hb["count"]} received'
                 + (f', mean {hb["mean_interval"]:.2f} s, max gap {hb["max_gap"]:.1f} s' if hb['mean_interval'] else '')
                 + (' OK' if hb['ok'] else ' NOT OK'))
    q = score['qualifying']
    lines.append('Qualifying: core ' + ' '.join(f'T{k}{"+" if v else "-"}' for k, v in q['core'].items())
                 + ' | advanced ' + ' '.join(f'T{k}{"+" if v else "-"}' for k, v in q['advanced'].items())
                 + f' | two in sequence {"yes" if q["sequence"] else "no"} | '
                 + ('QUALIFIED' if q['qualified'] else 'not yet qualified'))
    pr = score['protocol']
    lines.append(f'Reports: {pr["frames"]} frames, {pr["gate_reports"]} gate, {pr["object_reports"]} object, '
                 f'{pr["docking_reports"]} docking, {pr["errors"]} errors')
    for w in score['warnings']:
        lines.append(f'  ! {w}')
    return '\n'.join(lines)
