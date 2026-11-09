import json
import os

from sim_judge.viewer import OffsetEstimate, course_shapes

KEY = os.path.join(os.path.dirname(__file__), '..', '..', 'asv_wave_sim', 'gz-waves-models', 'worlds',
                   'roboboat2026_course.answer_key.json')


def test_course_shapes_cover_every_element_and_gate():
    with open(KEY, encoding='utf-8') as f:
        key = json.load(f)
    shapes = course_shapes(key)
    names = {s['name'] for s in shapes}
    for e in key['elements']:
        assert e['name'] in names, e['name']
    for g in key['gates']:
        line = next(s for s in shapes if s['name'] == g['label'])
        assert line['type'] == 'line' and len(line['points']) == 2
    assert any(s['name'] == 'boundary' for s in shapes)


def test_offset_starts_at_the_course_start_then_averages():
    est = OffsetEstimate((22.2, -23.9), keep=3)
    assert est.value() == (22.2, -23.9)
    est.add((32.0, -20.0), (10.0, 4.0))
    est.add((34.0, -18.0), (12.0, 6.0))
    assert est.value() == (22.0, -24.0)
    for _ in range(5):
        est.add((1.0, 1.0), (0.0, 0.0))
    assert est.value() == (1.0, 1.0)
