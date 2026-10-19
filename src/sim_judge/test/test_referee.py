import json
import math
import os

import pytest

from sim_judge import report_compat as pb
from sim_judge.referee import Referee
from sim_judge.robocommand import FrameParser, decode, encode_frame

KEY = os.path.join(os.path.dirname(__file__), '..', '..', 'asv_wave_sim', 'gz-waves-models', 'worlds',
                   'roboboat2026_course.answer_key.json')


@pytest.fixture
def key():
    with open(KEY, encoding='utf-8') as f:
        return json.load(f)


class Driver:
    """Moves a point boat along waypoints at constant speed, feeding the referee at 20 Hz."""

    def __init__(self, ref, start, speed=1.5, dt=0.05):
        self.ref, self.p, self.speed, self.dt, self.t = ref, start, speed, dt, 0.0
        self.yaw = 0.0
        ref.on_pose(self.t, start[0], start[1], 0.0)

    def go(self, *points):
        for q in points:
            dx, dy = q[0] - self.p[0], q[1] - self.p[1]
            n = max(1, int(math.hypot(dx, dy) / (self.speed * self.dt)))
            self.yaw = math.atan2(dy, dx)
            for i in range(1, n + 1):
                self.t += self.dt
                self.ref.on_pose(self.t, self.p[0] + dx * i / n, self.p[1] + dy * i / n, self.yaw)
            self.p = q

    def hold(self, seconds):
        for _ in range(int(seconds / self.dt)):
            self.t += self.dt
            self.ref.on_pose(self.t, self.p[0], self.p[1], self.yaw)

    def latlng(self):
        lat = self.ref.lat0 + self.p[1] / self.ref.m_per_deg
        lon = self.ref.lon0 + self.p[0] / (self.ref.m_per_deg * math.cos(math.radians(self.ref.lat0)))
        return pb.LatLng(latitude=lat, longitude=lon)

    def report(self, **body):
        r = pb.Report(team_id='QUEU', vehicle_id='sim', seq=len(self.ref.reports), **body)
        r.sent_at.FromSeconds(int(self.t))
        self.ref.on_report(self.t, r, '127.0.0.1:1')


def items(score, task):
    return {i['key']: i['points'] for i in score['tasks'][task]}


def test_task1_entry_and_exit(key):
    ref = Referee(key)
    d = Driver(ref, (22.2, -23.9))
    d.go((8.0, -23.9))
    d.report(gate_pass=pb.GatePass(type=pb.GateType.GATE_ENTRY, position=d.latlng()))
    d.go((7.0, -23.5), (22.0, -23.5))
    d.report(gate_pass=pb.GatePass(type=pb.GateType.GATE_EXIT, position=d.latlng()))
    s = ref.score()
    assert items(s, '1') == {'entry': 100, 'entry_report': 100, 'exit': 200, 'exit_report': 100}
    assert s['strikes']['count'] == 0
    assert s['qualifying']['core']['1'] and s['qualifying']['advanced']['1']


def test_task3_ccw_loop_and_reports(key):
    assert key['tasks']['3']['required_direction'] == 'ccw'
    ref = Referee(key)
    d = Driver(ref, (-4.8, 3.0))
    d.go((-4.8, 8.0))
    d.report(gate_pass=pb.GatePass(type=pb.GateType.GATE_SPEED_START, position=d.latlng()))
    ind = ref.el[key['tasks']['3']['indicator_id']]
    d.report(object_detected=pb.ObjectDetected(
        object_type=pb.ObjectType.OBJECT_LIGHT_BEACON, color=pb.Color.COLOR_RED,
        position=pb.LatLng(latitude=ind['lat'], longitude=ind['lon']),
        task_context=pb.TaskType.TASK_SPEED_CHALLENGE))
    d.go((-3.0, 12.0), (-2.5, 20.6), (-3.5, 22.5), (-5.5, 22.5), (-6.8, 20.6), (-7.6, 18.0), (-7.6, 15.5),
         (-6.0, 12.5), (-5.0, 8.5), (-4.8, 4.0))
    d.report(gate_pass=pb.GatePass(type=pb.GateType.GATE_SPEED_END, position=d.latlng()))
    s = ref.score()
    got = items(s, '3')
    assert got['loop'] == 200 and got['direction'] == 150 and got['report'] == 100
    assert 0 < got['time'] < 250
    assert s['strikes']['count'] == 0, s['strikes']['list']


def test_task3_clockwise_loses_direction_points(key):
    ref = Referee(key)
    d = Driver(ref, (-4.8, 3.0))
    d.go((-4.8, 8.0), (-5.6, 12.5), (-6.0, 18.5), (-6.9, 21.5), (-4.5, 23.0), (-2.6, 20.6), (-3.0, 12.0), (-4.6, 4.0))
    got = items(ref.score(), '3')
    assert got['loop'] == 200 and got['direction'] == 0


def test_task5_dock_correct_slip(key):
    t5 = key['tasks']['5']
    target = t5['correct'][0]
    slip = next(s for s in t5['slips'] if s['dock'] == target['dock'] and s['slip'] == target['slip'])
    cx, cy = slip['center']
    ref = Referee(key)
    lane_y = 1.65
    d = Driver(ref, (cx - 6.0, lane_y))
    d.go((cx, lane_y), (cx, cy))
    d.hold(4.0)
    d.report(docking=pb.Docking(dock=target['dock'], slip=str(target['slip'])))
    got = items(ref.score(), '5')
    assert got == {'dock': 350, 'report': 100}


def test_strike_on_buoy_contact(key):
    ref = Referee(key)
    g = ref.gates['gate1']
    d = Driver(ref, (g['red'][0] + 3.0, g['red'][1]))
    d.go((g['red'][0] - 3.0, g['red'][1]))
    s = ref.score()
    assert s['strikes']['count'] == 1 and s['strikes']['penalty'] == -25


def test_boundary_terminates_run(key):
    ref = Referee(key)
    d = Driver(ref, (22.2, -23.9))
    d.go((22.2, -40.0), (10.0, -40.0))
    assert ref.terminated is not None
    assert items(ref.score(), '1')['entry'] == 0


def test_frame_parser_handles_split_and_garbage():
    r = pb.Report(team_id='QUEU', vehicle_id='v', seq=7,
                  heartbeat=pb.Heartbeat(state=pb.RobotState.STATE_AUTO, current_task=pb.TaskType.TASK_DOCKING))
    f = encode_frame(r)
    p = FrameParser()
    out = p.feed(b'xx' + f[:4]) + p.feed(f[4:] + f + b'$')
    assert len(out) == 2 and p.errors
    back = decode(out[1])
    assert back.seq == 7 and back.heartbeat.current_task == pb.TaskType.TASK_DOCKING
