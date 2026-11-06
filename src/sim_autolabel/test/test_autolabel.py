import json
import os
import re
from types import SimpleNamespace

import cv2
import numpy as np
import pytest
import yaml

from sim_autolabel import export, upload
from sim_autolabel.record import Pairer, Recorder

SIM = os.path.join(os.path.dirname(__file__), '..', '..', 'asv_wave_sim')
WORLD = os.path.join(SIM, 'gz-waves-models', 'worlds', 'roboboat2026_course')


@pytest.fixture
def key():
    with open(WORLD + '.answer_key.json', encoding='utf-8') as f:
        return json.load(f)


def test_classes_yaml_uses_sim_labels(key):
    with open(export.CLASSES_YAML, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    for name in cfg['datasets']:
        classes, mapping, _ = export.load_dataset(export.CLASSES_YAML, name)
        assert set(mapping) <= set(key['labels']), name
        assert len(set(classes)) == len(classes)


def test_world_labels_every_element(key):
    with open(WORLD + '.sdf', encoding='utf-8') as f:
        world = f.read()
    blocks = {re.search(r'<name>([^<]*)</name>', b).group(1): b
              for b in re.findall(r'<include>.*?</include>', world, re.S)}
    blocks.update(re.findall(r'<model name="([^"]*)">(.*?)</model>', world, re.S))
    labelled = [e for e in key['elements'] if e.get('sim_label')]
    assert {e['kind'] for e in labelled} >= {'a0', 'a2', 'sur_mark', 'indicator_buoy', 'dock'}
    for e in labelled:
        assert f'<label>{key["labels"][e["sim_label"]]}</label>' in blocks[e['name']], e['name']
    # Vessels and banners label only their sign symbol / numeral, inside the model.
    assert not any(e['kind'] in ('vessel', 'banner') for e in labelled)
    for model, label in (('StationaryVesselYellow', 'vessel_sign_triangle'), ('StationaryVesselBlack', 'vessel_sign_cross'),
                         ('NumberBanner1', 'banner_1'), ('NumberBanner2', 'banner_2'), ('NumberBanner3', 'banner_3')):
        with open(os.path.join(SIM, 'gz-waves-models', 'models', model, 'model.sdf'), encoding='utf-8') as f:
            assert f'<label>{key["labels"][label]}</label>' in f.read(), model


def test_pairer_matches_by_stamp_in_any_order():
    got = []
    p = Pairer(lambda cam, st, img, box: got.append((cam, st, img, box)))
    p.add('camera1', 5, 'boxes', 'b5')
    p.add('camera0', 5, 'image', 'i5-cam0')
    p.add('camera1', 6, 'image', 'i6')
    p.add('camera1', 5, 'image', 'i5')
    assert got == [('camera1', 5, 'i5', 'b5')]


def _box(label, x0, y0, x1, y1):
    v = SimpleNamespace
    return v(label=label, box=v(min_corner=v(x=x0, y=y0), max_corner=v(x=x1, y=y1)))


def test_recorder_writes_image_and_boxes(tmp_path):
    w, h, step = 6, 4, 20                     # padded rows, as some encoders send
    rgb = np.full((h, step), 128, np.uint8)   # grey padding after each row
    rgb[:, :w * 3] = np.tile([255, 0, 0], w)   # the image itself is red
    img = SimpleNamespace(width=w, height=h, step=step, data=rgb.tobytes())
    rec = Recorder(str(tmp_path), {5: 'a2_red'}, empty_every=0, max_frames=0)
    rec.boat = {'x': 1.0, 'y': 2.0, 'yaw': 0.5}
    rec.on_frame('camera1', 1_500_000_000, img, SimpleNamespace(annotated_box=[_box(5, 1, 1, 3, 3)]))
    rec.on_frame('camera1', 2_000_000_000, img, SimpleNamespace(annotated_box=[]))   # empty: skipped
    files = sorted(os.listdir(tmp_path / 'camera1'))
    assert files == ['0000000001500000000.jpg', '0000000001500000000.json']
    frame = json.loads((tmp_path / 'camera1' / files[1]).read_text())
    assert frame['stamp'] == 1.5 and frame['boat']['x'] == 1.0
    assert frame['boxes'] == [{'label': 'a2_red', 'label_id': 5, 'x0': 1, 'y0': 1, 'x1': 3, 'y1': 3}]
    saved = cv2.imread(str(tmp_path / 'camera1' / files[0]))
    assert saved.shape == (h, w, 3) and saved[0, 0, 2] > 200 and saved[0, 0, 0] < 60   # BGR on disk
    assert rec.counts['a2_red'] == 1


def _session(root, name, frames):
    s = root / name
    (s / 'camera1').mkdir(parents=True)
    (s / 'session.json').write_text('{}')
    for stamp, boxes in frames:
        base = s / 'camera1' / f'{int(stamp * 1e9):019d}'
        cv2.imwrite(str(base) + '.jpg', np.zeros((100, 200, 3), np.uint8))
        base.with_suffix('.json').write_text(json.dumps(
            {'camera': 'camera1', 'stamp': stamp, 'width': 200, 'height': 100, 'boxes': boxes}))
    return s


def test_export_yolo_dataset(tmp_path):
    def b(label, x0, y0, x1, y1):
        return {'label': label, 'x0': x0, 'y0': y0, 'x1': x1, 'y1': y1}
    _session(tmp_path / 'rec', 's1', [
        # a0_green: 2 px wide in a 200 px frame = 6.4 px at 640, kept; a0_black: 0.5 px = 1.6 px, dropped
        (1.0, [b('a2_red', 40, 20, 60, 60), b('indicator_buoy_red', 0, 0, 50, 50), b('a0_green', 10, 10, 12, 30),
               b('a0_black', 100, 10, 100.5, 30)]),
        (2.0, [b('sur_mark_green', 190, 50, 230, 90)]),        # runs off the right edge: clipped
        (100.0, []),
    ])
    out = tmp_path / 'ds'
    rc = export.main([str(tmp_path / 'rec'), '--out', str(out), '--val-fraction', '0', '--preview', '1'])
    assert rc == 0
    data = yaml.safe_load((out / 'data.yaml').read_text())
    assert data['names'][5] == 'red_buoy' and data['nc'] == 9
    assert data['train'] == 'train/images' and data['val'] == 'valid/images'   # Roboflow's layout
    labels = {p.name: p.read_text().split('\n')[:-1] for p in (out / 'train' / 'labels').iterdir()}
    assert len(labels) == 3 and len(os.listdir(out / 'train' / 'images')) == 3
    assert labels['s1_camera1_0000000001000000000.txt'] == ['5 0.250000 0.400000 0.100000 0.400000',
                                                            '3 0.055000 0.200000 0.010000 0.200000']
    assert labels['s1_camera1_0000000002000000000.txt'] == ['4 0.975000 0.700000 0.050000 0.400000']
    assert labels['s1_camera1_0000000100000000000.txt'] == []
    summary = json.loads((out / 'export.json').read_text())
    assert summary['dropped'] == {'indicator_buoy_red (not in this dataset)': 1, 'a0_black (under 3.0 px at 640)': 1}
    assert len(os.listdir(out / 'preview')) == 1
    assert export.main([str(tmp_path / 'rec'), '--out', str(out)]) == 1           # refuses to overwrite
    assert export.main([str(tmp_path / 'rec'), '--out', str(out), '--force']) == 0


def test_val_split_is_by_block():
    vals = {export.is_val('/x/s1', 'camera1', t, 30.0, 0.5) for t in np.arange(0.0, 29.9, 0.5)}
    assert len(vals) == 1                     # one 30 s block never straddles train and val
    blocks = [export.is_val('/x/s1', 'camera1', 30.0 * k, 30.0, 0.5) for k in range(200)]
    assert 60 < sum(blocks) < 140


def test_numbers_dataset_filters_at_its_own_imgsz():
    classes, mapping, imgsz = export.load_dataset(export.CLASSES_YAML, 'numbers')
    dropped = export.collections.Counter()
    frame = {'width': 1920, 'height': 1200, 'boxes': [{'label': 'banner_2', 'x0': 0, 'y0': 0, 'x1': 5, 'y1': 50}]}
    assert imgsz == 960
    assert export.yolo_lines(frame, classes, mapping, imgsz, 3.0, dropped)[0] == []   # 2.5 px at 960
    assert export.yolo_lines(frame, classes, mapping, imgsz, 2.0, dropped)[0][0].startswith('1 ')


def test_upload_sends_every_image_with_split_and_tags(tmp_path, monkeypatch):
    _session(tmp_path / 'rec', '2026-11-05_10-00-00_seed4', [(1.0, []), (40.0, []), (80.0, []), (120.0, [])])
    out = tmp_path / 'ds'
    assert export.main([str(tmp_path / 'rec'), '--out', str(out), '--val-fraction', '0.5', '--val-block', '1']) == 0

    sent = []

    class Project:
        classes = {c: 10 for c in yaml.safe_load((out / 'data.yaml').read_text())['names']}

        def single_upload(self, **kw):
            sent.append(kw)

    class Roboflow:
        def __init__(self, api_key):
            assert api_key == 'k'

        def workspace(self, ws):
            return SimpleNamespace(project=lambda p: Project())

    monkeypatch.setitem(__import__('sys').modules, 'roboflow', SimpleNamespace(Roboflow=Roboflow))
    monkeypatch.setenv('ROBOFLOW_API_KEY', 'k')
    assert upload.main([str(out), '--workspace', 'aqua', '--project', 'main', '--tag', 'night']) == 0
    assert len(sent) == 4 and {s['split'] for s in sent} == {'train', 'valid'}
    for s in sent:
        assert os.path.basename(s['image_path'])[:-4] == os.path.basename(s['annotation_path'])[:-4]
        assert f'/{s["split"]}/' in s['image_path'].replace(os.sep, '/')
        assert s['tag_names'] == ['sim', 'sim_seed4', 'night'] and s['batch_name'] == 'sim ds'
    assert upload.main([str(out), '--workspace', 'aqua', '--project', 'main', '--limit', '1']) == 0
    assert len(sent) == 5

    Project.classes = {'Black Buoy': 3}                    # names that don't match: refuse
    assert upload.main([str(out), '--workspace', 'aqua', '--project', 'main']) == 1
    assert len(sent) == 5


def test_main_class_order_matches_the_stack():
    """classes.yaml, class_mapping.yaml, planning and the deployed model agree on ids 0-8."""
    import ast
    root = os.path.join(os.path.dirname(__file__), '..', '..', '..')
    classes, _, _ = export.load_dataset(export.CLASSES_YAML, 'main')
    assert classes == sorted(classes)                          # alphabetical, as the model was trained
    with open(os.path.join(root, 'computer_vision', 'class_mapping.yaml'), encoding='utf-8') as f:
        mapping = yaml.safe_load(f)
    assert [mapping['classes'][i] for i in range(9)] == classes
    assert all(mapping['class_name_to_id'][c] == i for i, c in enumerate(classes))
    with open(os.path.join(root, 'planning', 'Global', 'entities.py'), encoding='utf-8') as f:
        tree = ast.parse(f.read())
    table = next(ast.literal_eval(n.value) for n in tree.body if isinstance(n, ast.Assign)
                 and getattr(n.targets[0], 'id', '') == 'CLASS_ID_TO_ENTITY_TYPE')
    assert [table[i] for i in range(9)] == classes
    onnx = os.path.join(root, 'computer_vision', 'model_building_and_training', 'aqua_main.onnx')
    if os.path.isfile(onnx):
        with open(onnx, 'rb') as f:
            m = re.search(rb"names.{1,4}(\{0: '[^}]*\})", f.read())
        assert m and list(ast.literal_eval(m.group(1).decode()).values()) == classes
