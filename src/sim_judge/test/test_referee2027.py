import json
import math
import os

import pytest

from sim_judge import report_compat as pb
from sim_judge.referee2027 import Referee2027
from test_referee import Driver, items

WORLDS = os.path.join(os.path.dirname(__file__), '..', '..', 'asv_wave_sim', 'gz-waves-models', 'worlds')


def load(name):
    with open(os.path.join(WORLDS, name), encoding='utf-8') as f:
        return json.load(f)


@pytest.fixture
def key():
    return load('roboboat2027_course.answer_key.json')


def el(key, eid):
    return next(e for e in key['elements'] if e['id'] == eid)


def circle(center, radius, start_angle, turns, clockwise, steps=36):
    """Points on a circle around center, starting at start_angle, sweeping turns * 360 deg."""
    sgn = -1.0 if clockwise else 1.0
    n = int(steps * turns)
    return [(center[0] + radius * math.cos(start_angle + sgn * 2 * math.pi * k / steps),
             center[1] + radius * math.sin(start_angle + sgn * 2 * math.pi * k / steps)) for k in range(n + 1)]


def angle(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])


def offset_path(path, d):
    """The polyline shifted d m to its left (port)."""
    out = []
    for i, p in enumerate(path):
        a, b = (path[i], path[i + 1]) if i + 1 < len(path) else (path[i - 1], path[i])
        h = angle(a, b)
        out.append((p[0] - d * math.sin(h), p[1] + d * math.cos(h)))
    return out


def drive_task1(key, lateral=0.0):
    t1 = key['tasks']['1']
    entry, exit_ = el(key, t1['entry_id']), el(key, t1['exit_id'])
    path = [tuple(p) for p in t1['path']]
    ref = Referee2027(key)
    first_leg = angle(path[0], path[1])
    s = key['starts']['1']
    d = Driver(ref, (s['x'], s['y']))
    # Clockwise around ENTRY starting from behind it, then along the passage, then counter-clockwise
    # around EXIT and away.
    behind = first_leg + math.pi
    d.go(*circle((entry['x'], entry['y']), 2.5, behind, 1.1, clockwise=True))
    d.go(*offset_path(path[1:-1], lateral))
    last_leg = angle(path[-2], path[-1])
    d.go(*circle((exit_['x'], exit_['y']), 2.5, last_leg + math.pi, 1.1, clockwise=False))
    return ref, d


def test_task1_core(key):
    ref, d = drive_task1(key)
    s = ref.score()
    got = items(s, '1')
    assert got['entry'] == 100 and got['transit'] == 200 and got['exit'] == 100, s['tasks']['1']
    assert s['strikes']['count'] == 0
    assert s['qualifying']['core']['1']


def test_task1_wrong_side(key):
    # 3 m to port of the passage: the GREEN buoys (1.5 m to port of it) end up on starboard.
    ref, d = drive_task1(key, lateral=3.0)
    s = ref.score()
    t1 = {i['key']: i for i in s['tasks']['1']}
    assert t1['entry']['points'] == 100 and t1['transit']['points'] == 0
    assert 'wrong side' in t1['transit']['detail']


def test_task1_reports(key):
    ref, d = drive_task1(key)
    t1 = key['tasks']['1']

    def at(e):
        lat = ref.lat0 + e['y'] / ref.m_per_deg
        lon = ref.lon0 + e['x'] / (ref.m_per_deg * math.cos(math.radians(ref.lat0)))
        return pb.LatLng(latitude=lat, longitude=lon)
    entry, exit_ = el(key, t1['entry_id']), el(key, t1['exit_id'])
    d.report(gate_pass=pb.GatePass(type=pb.GateType.GATE_ENTRY, position=at(entry)))
    d.report(gate_pass=pb.GatePass(type=pb.GateType.GATE_EXIT, position=at(exit_)))
    colors = {'red': pb.Color.COLOR_RED, 'green': pb.Color.COLOR_GREEN, 'black': pb.Color.COLOR_BLACK}
    for eid in [m['id'] for m in t1['markers']] + t1['black_ids']:
        e = el(key, eid)
        d.report(object_detected=pb.ObjectDetected(object_type=pb.ObjectType.OBJECT_LIGHT_BEACON,
                                                   color=colors[e['color']], position=at(e)))
    got = items(ref.score(), '1')
    assert got['report_points'] == 50 and got['report_buoys'] == 50


def test_task2_core(key):
    t2 = key['tasks']['2']
    g, tgt = el(key, t2['start_id']), el(key, t2['target_id'])
    ref = Referee2027(key)
    s = key['starts']['2']
    d = Driver(ref, (s['x'], s['y']))
    d.go((g['x'] + 1.0, g['y']))
    a = angle((tgt['x'], tgt['y']), (g['x'], g['y']))
    d.go(*circle((tgt['x'], tgt['y']), 2.0, a, 1.1, clockwise=False))
    # Straight out of the field, away from its centre.
    cx = sum(p[0] for p in t2['field']) / len(t2['field'])
    cy = sum(p[1] for p in t2['field']) / len(t2['field'])
    h = angle((cx, cy), (tgt['x'], tgt['y']))
    d.go((tgt['x'] + 30 * math.cos(h), tgt['y'] + 30 * math.sin(h)))
    got = items(ref.score(), '2')
    assert got['start'] == 50 and got['loop'] == 200 and got['exit'] == 100 and got['time'] > 0, ref.events


def test_task2_loop_before_start_does_not_count(key):
    t2 = key['tasks']['2']
    tgt = el(key, t2['target_id'])
    ref = Referee2027(key)
    d = Driver(ref, (tgt['x'] + 2.0, tgt['y']))
    d.go(*circle((tgt['x'], tgt['y']), 2.0, 0.0, 1.1, clockwise=True))
    got = items(ref.score(), '2')
    assert got['start'] == 0 and got['loop'] == 0


def test_task3_green_slip(key):
    t3 = key['tasks']['3']
    slip = next(s for s in t3['slips'] if s['indicator'] == 'green' and not s['occupied'])
    ref = Referee2027(key)
    s = key['starts']['3']
    d = Driver(ref, (s['x'], s['y']))
    d.go(tuple(slip['center']))
    d.hold(4.0)
    got = items(ref.score(), '3')
    assert got['dock'] == 200


def test_task4_delivery_after_the_message(key):
    t4 = key['tasks']['4']
    v = el(key, t4['assigned'][0])
    s = key['starts']['course']
    ref = Referee2027(key)
    d = Driver(ref, (s['x'], s['y']))
    near = (v['x'] + 2.0 * math.cos(angle((v['x'], v['y']), (s['x'], s['y']))),
            v['y'] + 2.0 * math.sin(angle((v['x'], v['y']), (s['x'], s['y']))))
    # Parking at the vessel before RoboCommand asks does not count.
    d.go(near)
    d.hold(5.0)
    assert ref.due_tasking() and items(ref.score(), '4')[f'deliver_{v["id"]}'] == 0   # SPRINT_TARGET at 0 s
    at = next(m['at_s'] for m in key['tasking'] if m['task'] == '4')
    d.go((s['x'], s['y']))
    d.hold(at - d.t + 1.0)
    sent = ref.due_tasking()
    assert [m['type'] for m in sent] == ['DELIVERY']
    assert sent[0]['targets'] == [{'shape': t4['assigned_shapes'][0]}]
    assert ref.due_tasking() == []                     # each message once
    d.go(near)
    d.hold(4.0)
    assert items(ref.score(), '4')[f'deliver_{v["id"]}'] == 200


def test_task5_rally(key):
    r = key['tasks']['5']['rally']
    s = key['starts']['course']
    ref = Referee2027(key)
    d = Driver(ref, (s['x'], s['y']))
    at = next(m['at_s'] for m in key['tasking'] if m['task'] == '5')
    d.hold(at + 1.0)
    types = [m['type'] for m in ref.due_tasking()]
    assert 'RALLY' in types and 'DELIVERY' in types
    rally = next(m for m in ref.issued_messages() if m['type'] == 'RALLY')
    assert abs(rally['lat'] - r['lat']) < 1e-9
    d.go((r['x'] + 1.0, r['y']))
    assert items(ref.score(), '5')['rally'] == 200


def test_tasking_can_be_disabled(key):
    ref = Referee2027(key, tasking=False)
    d = Driver(ref, (0.0, 0.0))
    d.hold(300.0)
    assert ref.due_tasking() == []


def test_readiness_gates():
    key = load('roboboat2027_readiness.answer_key.json')
    assert key['layout'] == 'readiness' and set(key['tasks']) == {'R'}
    g1, g2 = (next(g for g in key['gates'] if g['label'] == lab) for lab in ('gate1', 'gate2'))
    ref = Referee2027(key)
    s = key['starts']['R']
    d = Driver(ref, (s['x'], s['y']))
    h = g2['heading']
    d.go(tuple(g1['center']), tuple(g2['center']),
         (g2['center'][0] + 3 * math.cos(h), g2['center'][1] + 3 * math.sin(h)))
    got = items(ref.score(), 'R')
    assert got == {'gates': 100, 'clean': 100}
