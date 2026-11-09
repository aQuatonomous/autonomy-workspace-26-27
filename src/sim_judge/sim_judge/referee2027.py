"""RoboBoat 2027 referee: Core tier of Tasks 1-5 and the Proof of Readiness gates.

The 2027 rules so far are the Primer & Task Ideas: tasks and tiers, no scoring tables. Each Core
requirement is scored as its own item worth PLACEHOLDER points (listed in INTERPRETATIONS_2027) so
runs can be compared; replace them with the handbook's tables when it is published.

Tasks 4 and 5 start with a RoboCommand message. The answer key's `tasking` list says when; the
judge asks due_tasking() each tick and sends what is due on its tasking topic.
"""

import math

from . import geometry as geo
from .referee import FULL_CIRCLE, Referee, Winding, _dock_letter, _slip_number, score_item

PASS_RADIUS = 5.0          # a marker is "passed" when it goes from ahead to behind within this distance

INTERPRETATIONS_2027 = [
    'Rules: RoboBoat 2027 Primer & Task Ideas. Points are PLACEHOLDERS until the 2027 handbook.',
    'Task 1: ENTRY / EXIT are the blue-lit buoys. Circling = a full turn (~349 deg) within 10 m, '
    'clockwise around ENTRY, then counter-clockwise around EXIT after it.',
    'Task 1: a marker counts as passed when it moves from ahead of the boat to behind it within '
    f'{PASS_RADIUS:.0f} m; GREEN must then be on the boat\'s port side, RED on starboard. Only '
    'passes after the ENTRY circle count, and only the first pass of each marker.',
    'Task 1 reports: ENTRY / EXIT as GatePass ENTRY / EXIT reports within the gate tolerance of the '
    'ENTRY / EXIT buoy (the 2026 protocol has no blue colour); RED, GREEN and BLACK buoys as '
    'ObjectDetected with the colour, within the report tolerance.',
    'Task 2: start = within start_radius of the GREEN-lit numbered beacon; loop = a full turn '
    'around the target buoy (either direction) after the start; exit = leaving the field (hull of '
    'its buoys grown by exit_margin) after the loop. Time = start to exit.',
    'Task 3: docked = boat centre inside a slip for the dock hold time; Core = a GREEN slip.',
    'Task 4: delivered = boat centre within delivery_range of the assigned vessel for '
    'delivery_hold seconds after the DELIVERY message (no water/payload is simulated).',
    'Task 5: rallied = boat centre within rally_radius of the RALLY point after the RALLY message.',
    'Proof of Readiness: both gates forward in order; "no obstacle strikes" = no strike at all.',
    'Leaving the course boundary (boat centre) terminates the run; scoring freezes at that moment.',
]


class Referee2027(Referee):
    def __init__(self, key, tasking=True, **kw):
        self.tasking_enabled = tasking
        self.first_t = None
        super().__init__(key, **kw)

    def _init_tasks(self, circle_radius):
        T = self.tasks
        self.slip_task = '3'
        self.s1 = self.s2 = self.s4 = self.s5 = self.sR = None
        if '1' in T:
            e, x = self.el[T['1']['entry_id']], self.el[T['1']['exit_id']]
            self.w_entry = Winding((e['x'], e['y']), circle_radius)
            self.w_exit = Winding((x['x'], x['y']), circle_radius)
            self.s1 = {'entry': None, 'exit': None, 'passes': {}, 'ahead': {}}
        if '2' in T:
            g = self.el[T['2']['target_id']]
            self.w_target = Winding((g['x'], g['y']), circle_radius)
            self.field = [tuple(p) for p in T['2']['field']]
            self.s2 = {'start': None, 'loop': None, 'exit': None}
        if '4' in T:
            self.s4 = {'inside': {}, 'delivered': {}}
        if '5' in T:
            self.s5 = {'reached': None}
        if 'R' in T:
            self.sR = {'g1': None, 'done': None}
        self.tasking = list(self.key.get('tasking', []))
        self.issued = {}                                  # index in self.tasking -> sim time sent

    # ------------------------------------------------------------------ RoboCommand tasking
    def on_pose(self, t, x, y, yaw):
        if self.first_t is None:
            self.first_t = t
        super().on_pose(t, x, y, yaw)

    def due_tasking(self):
        """Messages whose time (sim seconds after the judge's first pose) has come; each once."""
        if not self.tasking_enabled or self.first_t is None or self.t is None or not self.active:
            return []
        out = []
        for i, m in enumerate(self.tasking):
            if i not in self.issued and self.t - self.first_t >= m['at_s']:
                self.issued[i] = self.t
                msg = dict(m['message'], seq=i + 1, issued_sim_t=round(self.t, 2))
                self._event(f'RoboCommand: sent {msg["type"]} (task {m["task"]})')
                out.append(msg)
        return out

    def issued_messages(self):
        return [dict(self.tasking[i]['message'], seq=i + 1, issued_sim_t=round(t, 2)) for i, t in sorted(self.issued.items())]

    def _issued_at(self, task):
        times = [t for i, t in self.issued.items() if self.tasking[i]['task'] == task]
        return min(times) if times else None

    # ------------------------------------------------------------------ tracking
    def _track(self, x, y, yaw):
        t = self.t
        if self.s1:
            s = self.s1
            self.w_entry.update(x, y)
            if s['entry'] is None and self.w_entry.cw <= -FULL_CIRCLE:
                s['entry'] = t
                self.w_exit.reset()
                self._event('Task 1: circled the ENTRY buoy clockwise')
            if s['entry'] is not None:
                self._check_passes(x, y, yaw)
                if s['exit'] is None:
                    self.w_exit.update(x, y)
                    if self.w_exit.ccw >= FULL_CIRCLE:
                        s['exit'] = t
                        self._event('Task 1: circled the EXIT buoy counter-clockwise')
        if self.s2:
            s, t2 = self.s2, self.tasks['2']
            g = self.el[t2['start_id']]
            if s['start'] is None and math.hypot(g['x'] - x, g['y'] - y) <= t2['start_radius']:
                s['start'] = t
                self.w_target.reset()
                self._event(f'Task 2: started at the green beacon {t2["start_number"]}')
            if s['start'] is not None and s['loop'] is None:
                self.w_target.update(x, y)
                if abs(self.w_target.best) >= FULL_CIRCLE:
                    s['loop'] = t
                    self._event(f'Task 2: looped the {t2["target_color"]} target buoy')
            if s['loop'] is not None and s['exit'] is None and not geo.point_in_poly((x, y), self.field):
                s['exit'] = t
                self._event(f'Task 2: left the field, {t - s["start"]:.1f} s after the start')
        if self.s4:
            sent = self._issued_at('4')
            if sent is not None:
                t4 = self.tasks['4']
                for v in t4['vessels']:
                    e = self.el[v['id']]
                    if math.hypot(e['x'] - x, e['y'] - y) <= t4['delivery_range']:
                        since = self.s4['inside'].setdefault(v['id'], t)
                        if t - since >= t4['delivery_hold'] and v['id'] not in self.s4['delivered']:
                            self.s4['delivered'][v['id']] = t
                            ok = 'assigned' if v['id'] in t4['assigned'] else 'NOT assigned'
                            self._event(f'Task 4: delivered to the {v["shape"]} vessel ({ok}), '
                                        f'{t - sent:.1f} s after the message')
                    else:
                        self.s4['inside'].pop(v['id'], None)
        if self.s5 and self.s5['reached'] is None:
            sent = self._issued_at('5')
            r = self.tasks['5']
            if sent is not None and math.hypot(r['rally']['x'] - x, r['rally']['y'] - y) <= r['rally_radius']:
                self.s5['reached'] = t
                self._event(f'Task 5: reached the rally point {t - sent:.1f} s after the message')

    def _check_passes(self, x, y, yaw):
        hx, hy = math.cos(yaw), math.sin(yaw)
        s = self.s1
        for m in self.tasks['1']['markers']:
            e = self.el[m['id']]
            rx, ry = e['x'] - x, e['y'] - y
            along = rx * hx + ry * hy
            prev = s['ahead'].get(m['id'])
            s['ahead'][m['id']] = along
            if (prev is not None and prev > 0.0 >= along and math.hypot(rx, ry) <= PASS_RADIUS
                    and m['id'] not in s['passes']):
                side = 'port' if hx * ry - hy * rx > 0 else 'starboard'
                s['passes'][m['id']] = side
                self._event(f'Task 1: passed {m["color"]} buoy #{m["id"]} on {side}'
                            + ('' if side == m['side'] else ' (WRONG SIDE)'))

    def _on_gate(self, task, label, fwd):
        if task != 'R' or not fwd or self.sR is None:
            return
        if label == 'gate1' and self.sR['done'] is None:
            self.sR['g1'] = self.t
        elif label == 'gate2' and self.sR['g1'] is not None and self.sR['done'] is None:
            self.sR['done'] = self.t
            self._event('Proof of Readiness: through both gates')

    # ------------------------------------------------------------------ scoring
    def _task_items(self):
        tasks = {}
        struck = {s['id'] for s in self.strikes}
        item = score_item
        T = self.tasks

        if self.s1:
            t1, s = T['1'], self.s1
            markers = t1['markers']
            right = [m for m in markers if s['passes'].get(m['id']) == m['side']]
            wrong = [m for m in markers if m['id'] in s['passes'] and s['passes'][m['id']] != m['side']]
            hit = [m for m in markers if m['id'] in struck]
            transit = s['entry'] is not None and len(right) == len(markers) and not hit
            entry, exit_ = self.el[t1['entry_id']], self.el[t1['exit_id']]
            entry_rep = any(r['type'] == 'GATE_ENTRY' and math.dist(r['xy'], (entry['x'], entry['y'])) <= self.gate_report_tol
                            for r in self.gate_reports)
            exit_rep = any(r['type'] == 'GATE_EXIT' and math.dist(r['xy'], (exit_['x'], exit_['y'])) <= self.gate_report_tol
                           for r in self.gate_reports)
            ids = [m['id'] for m in markers] + t1['black_ids']
            found = [i for i in ids if self._reported(i)]
            tasks['1'] = [
                item('entry', 'Circle the ENTRY buoy clockwise', 100, 100 if s['entry'] is not None else 0,
                     f'{math.degrees(abs(self.w_entry.cw)):.0f} deg clockwise'),
                item('transit', 'Transit: every GREEN to port, every RED to starboard, no contact', 200,
                     200 if transit else 0,
                     f'{len(right)}/{len(markers)} correct' + (f', {len(wrong)} wrong side' if wrong else '')
                     + (f', {len(hit)} struck' if hit else '')),
                item('exit', 'Circle the EXIT buoy counter-clockwise (after ENTRY)', 100,
                     100 if s['exit'] is not None else 0, f'{math.degrees(self.w_exit.ccw):.0f} deg counter-clockwise'),
                item('report_points', 'Report ENTRY and EXIT points (GatePass ENTRY / EXIT)', 50,
                     50 if entry_rep and exit_rep else 0,
                     f'entry={"yes" if entry_rep else "no"} exit={"yes" if exit_rep else "no"}'),
                item('report_buoys', 'Report the RED, GREEN and BLACK buoys (ObjectDetected)', 50,
                     50 if len(found) == len(ids) else 0, f'{len(found)}/{len(ids)} within {self.report_tol:.0f} m'),
            ]

        if self.s2:
            t2, s = T['2'], self.s2
            T_run = (s['exit'] - s['start']) if s['exit'] is not None else None
            running = ''
            if s['start'] is not None and s['exit'] is None and self.t is not None:
                running = f'running {self.t - s["start"]:.0f} s, {math.degrees(abs(self.w_target.best)):.0f} deg around'
            tasks['2'] = [
                item('start', f'Start at the GREEN beacon (number {t2["start_number"]})', 50,
                     50 if s['start'] is not None else 0),
                item('loop', f'Loop around the {t2["target_color"]} target buoy', 200,
                     200 if s['loop'] is not None else 0, running),
                item('exit', 'Exit the field after the loop', 100, 100 if s['exit'] is not None else 0),
                item('time', 'Sprint time, start to exit (PLACEHOLDER 250 - T)', 250,
                     max(0.0, min(250.0, 250.0 - T_run)) if T_run is not None else 0,
                     f'T = {T_run:.1f} s' if T_run is not None else ''),
            ]

        if '3' in T and 'slips' in T['3']:
            t3 = T['3']
            green = [d for d in self.t5['docked'] if d['info']['indicator'] == 'green' and not d['info']['occupied']]
            docked = {d['slip'] for d in self.t5['docked']}
            rep = any((_dock_letter(r['dock']), _slip_number(r['slip'])) in docked for r in self.dockings)
            target = ', '.join(f'{c["dock"]}{c["slip"]}' for c in t3['green']) or 'none'
            tasks['3'] = [
                item('dock', f'Dock in a GREEN slip (green: {target})', 200, 200 if green else 0,
                     ', '.join(f'{a}{b}' for a, b in sorted(docked))),
                item('report', 'Docking report matches the slip used', 100, 100 if rep else 0,
                     f'{len(self.dockings)} received'),
            ]

        if self.s4:
            t4, sent = T['4'], self._issued_at('4')
            items = []
            for vid in t4['assigned']:
                v = next(v for v in t4['vessels'] if v['id'] == vid)
                done = self.s4['delivered'].get(vid)
                items.append(item(f'deliver_{vid}', f'Deliver to the assigned {v["shape"]} vessel', 200,
                                  200 if done is not None else 0,
                                  'message not sent yet' if sent is None else
                                  f'{done - sent:.1f} s after the message' if done is not None else ''))
            wrong = [vid for vid in self.s4['delivered'] if vid not in t4['assigned']]
            if wrong:
                items[-1]['detail'] += f' (also stopped at {len(wrong)} other vessel(s))'
            tasks['4'] = items

        if self.s5:
            sent, reached = self._issued_at('5'), self.s5['reached']
            tasks['5'] = [item('rally', 'Break off and rally at the GPS point', 200, 200 if reached is not None else 0,
                               'message not sent yet' if sent is None else
                               f'{reached - sent:.1f} s after the message' if reached is not None else '')]

        if self.sR:
            done = self.sR['done'] is not None
            tasks['R'] = [
                item('gates', 'Through both gates', 100, 100 if done else 0),
                item('clean', 'No obstacle strikes', 100, 100 if done and not self.strikes else 0,
                     f'{len(self.strikes)} strike(s)'),
            ]
        return tasks

    def _qualifying(self, tasks, hb):
        core = {k: bool(items) and all(i['points'] > 0 for i in items if i['key'] not in ('time', 'report',
                                                                                       'report_points', 'report_buoys'))
                for k, items in tasks.items()}
        return {'core': core, 'advanced': {}, 'sequence': False, 'heartbeat': hb['ok'], 'qualified': False}
