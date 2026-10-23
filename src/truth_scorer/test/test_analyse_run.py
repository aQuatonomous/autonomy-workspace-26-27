"""Offline analysis, exercised end to end on a synthetic log.

analyse_run.py is the tool that decides what a slow, expensive sim run meant. If it quietly
mis-parses the log, the sweep optimises the wrong thing and the fitted confusion matrix is
garbage dressed up as a measurement. So the log format is round-tripped here rather than assumed.

No ROS and no sim: the log is a file, which is exactly the point of the JSONL.
"""

import json
import math
import random

import pytest

from truth_scorer.analyse_run import (
    build_parser,
    cmd_fit_confusion,
    cmd_replay,
    cmd_summarise,
    confusion_counts,
    offset_of,
    read_records,
    replay,
    split,
)
from truth_scorer.config_io import load_confusion_matrix
from truth_scorer.core.classes import index_of
from truth_scorer.core.config import ScorerConfig

RED_BUOY = 3
GREEN_BUOY = 1
START = {'x': 22.2, 'y': -23.9}


def write_log(path, n_frames=60, mislabel_every=0, with_offset=True, seed=7):
    """A synthetic run: two buoys 8 ft apart, a boat driving past, in the real frame offset."""
    rng = random.Random(seed)
    # Truth is recorded in the answer-key frame; the map frame is offset by START.
    truth = [
        {'id': 1, 'name': 'gate1_red', 'kind': 'sur_mark', 'colour': 'red',
         'x': START['x'] + 20.0, 'y': START['y'] + 0.0,
         'canonical_class_id': RED_BUOY, 'acceptable_class_ids': [3, 4, 9], 'point_like': True},
        {'id': 2, 'name': 'gate1_green', 'kind': 'sur_mark', 'colour': 'green',
         'x': START['x'] + 20.0, 'y': START['y'] + 2.4384,
         'canonical_class_id': GREEN_BUOY, 'acceptable_class_ids': [1, 2, 10],
         'point_like': True},
    ]
    lines = [{'type': 'meta', 'seed': 0, 'world': 'test.sdf', 'start': START,
              'match_distance_m': 2.0, 'opportunity_range_m': 30.0,
              'truth_elements': truth}]

    for i in range(n_frames):
        t = i * 0.4
        boat = {'east': i * 0.05, 'north': 1.2192, 'yaw': 0.0}
        lines.append({'type': 'frame', 't': t, 'boat': boat, 'objects': []})
        for element, cid in ((truth[0], RED_BUOY), (truth[1], GREEN_BUOY)):
            east = element['x'] - START['x'] + rng.gauss(0, 0.08)
            north = element['y'] - START['y'] + rng.gauss(0, 0.08)
            observed = cid
            if mislabel_every and i % mislabel_every == 0:
                observed = GREEN_BUOY if cid == RED_BUOY else RED_BUOY
            lines.append({
                'type': 'detection', 't': t, 'track_id': element['id'],
                'east': east, 'north': north,
                'range_m': math.hypot(east - boat['east'], north - boat['north']),
                'class_id': observed, 'class_score': 0.7, 'cv_age_s': 0.1,
                'truth_id': element['id'], 'truth_class_id': cid, 'truth_distance_m': 0.1,
            })

    if with_offset:
        lines.append({'type': 'offset', 'estimated': [START['x'], START['y']],
                      'expected': [START['x'], START['y']], 'source': 'test',
                      'disagreement_m': 0.0})

    with open(path, 'w', encoding='utf-8') as handle:
        for record in lines:
            handle.write(json.dumps(record) + '\n')
    return path


@pytest.fixture
def log(tmp_path):
    return write_log(tmp_path / 'run.jsonl')


# -- parsing -----------------------------------------------------------------------------------

def test_split_separates_the_record_kinds(log):
    meta, detections, frames = split(read_records(log))
    assert meta['seed'] == 0
    assert len(frames) == 60
    assert len(detections) == 120


def test_a_torn_final_line_does_not_lose_the_run(tmp_path, capsys):
    """A run killed with Ctrl+C mid-write leaves a half-written line. Discarding the whole file
    over it would throw away a sim run that costs minutes to reproduce."""
    path = write_log(tmp_path / 'run.jsonl')
    with open(path, 'a', encoding='utf-8') as handle:
        handle.write('{"type": "detection", "t": 99.0, "ea')
    records = read_records(path)
    meta, detections, frames = split(records)
    assert meta is not None and len(detections) == 120
    assert 'unparseable' in capsys.readouterr().err


def test_measured_offset_wins_over_the_assumed_one(tmp_path):
    path = write_log(tmp_path / 'run.jsonl', with_offset=False)
    meta, _, _ = split(read_records(path))
    assert offset_of(meta) == (START['x'], START['y']), 'falls back to the answer key start'

    with open(path, 'a', encoding='utf-8') as handle:
        handle.write(json.dumps({'type': 'offset', 'estimated': [1.0, 2.0]}) + '\n')
    meta, _, _ = split(read_records(path))
    assert offset_of(meta) == (1.0, 2.0), 'a measured offset must override the assumption'


# -- confusion fitting ---------------------------------------------------------------------------

def test_confusion_counts_tally_truth_against_observation(log):
    counts = confusion_counts(read_records(log))
    assert counts[RED_BUOY][RED_BUOY] == 60
    assert GREEN_BUOY not in counts[RED_BUOY]


def test_confusion_counts_see_mislabels(tmp_path):
    path = write_log(tmp_path / 'run.jsonl', mislabel_every=5)
    counts = confusion_counts(read_records(path))
    assert counts[RED_BUOY][GREEN_BUOY] == 12, 'one frame in five out of 60'
    assert counts[RED_BUOY][RED_BUOY] == 48


def test_clutter_is_excluded_rather_than_filed_as_unknown(tmp_path):
    """A detection with no truth nearby says nothing about class confusion, and counting it
    would teach the matrix that every real class is often reported as whatever clutter looks
    like."""
    path = write_log(tmp_path / 'run.jsonl')
    with open(path, 'a', encoding='utf-8') as handle:
        handle.write(json.dumps({'type': 'detection', 't': 1.0, 'class_id': 5,
                                 'truth_id': None, 'truth_class_id': None}) + '\n')
    counts = confusion_counts(read_records(path))
    assert 5 not in {obs for row in counts.values() for obs in row}


def test_fit_confusion_writes_a_loadable_matrix(tmp_path):
    path = write_log(tmp_path / 'run.jsonl', mislabel_every=5)
    out = tmp_path / 'fitted.yaml'
    args = build_parser().parse_args(
        ['fit-confusion', str(path), '-o', str(out), '--min-samples', '10'])
    assert cmd_fit_confusion(args) == 0

    matrix = load_confusion_matrix(str(out))
    row = matrix[index_of(RED_BUOY)]
    assert row[index_of(RED_BUOY)] > row[index_of(GREEN_BUOY)] > 0.0
    assert row[index_of(GREEN_BUOY)] > 0.1, 'the 20 % mislabel rate must show up in the fit'
    assert sum(row) == pytest.approx(1.0)


def test_fit_refuses_a_matrix_built_from_nothing(tmp_path, capsys):
    """A matrix fitted on a handful of frames is worse than the hand-built prior, because it
    carries false authority."""
    path = write_log(tmp_path / 'run.jsonl', n_frames=2)
    out = tmp_path / 'fitted.yaml'
    args = build_parser().parse_args(['fit-confusion', str(path), '-o', str(out)])
    assert cmd_fit_confusion(args) == 1
    assert not out.exists()
    assert 'refusing' in capsys.readouterr().err


def test_fit_merges_multiple_seeds(tmp_path):
    a = write_log(tmp_path / 'a.jsonl', mislabel_every=5, seed=1)
    b = write_log(tmp_path / 'b.jsonl', mislabel_every=0, seed=2)
    out = tmp_path / 'fitted.yaml'
    args = build_parser().parse_args(
        ['fit-confusion', str(a), str(b), '-o', str(out), '--min-samples', '10'])
    assert cmd_fit_confusion(args) == 0
    row = load_confusion_matrix(str(out))[index_of(RED_BUOY)]
    merged_error_rate = 12 / 120
    assert row[index_of(GREEN_BUOY)] < merged_error_rate + 0.05


# -- replay -------------------------------------------------------------------------------------

def test_replay_recovers_both_buoys(log):
    omap, meta = replay(read_records(log), ScorerConfig(), None)
    confident = omap.confident_objects(0.5)
    assert len(confident) == 2, 'the two gate buoys must not merge on replay either'
    norths = sorted(o.position.north for o in confident)
    assert norths[1] - norths[0] == pytest.approx(2.4384, abs=0.3)


def test_replay_groups_detections_into_frames(log):
    """If replay fed detections one at a time, each co-visible buoy would be charged an
    expected miss for every frame the other one was processed in, and both would decay."""
    omap, _ = replay(read_records(log), ScorerConfig(), None)
    assert omap.stats['frames'] == 60, 'one frame per timestamp, not one per detection'
    assert all(o.p_exist > 0.9 for o in omap.objects)


def test_replay_respects_overridden_tunables(log, tmp_path, capsys):
    base = build_parser().parse_args(['replay', str(log)])
    assert cmd_replay(base) == 0
    default_out = capsys.readouterr().out

    tweaked = build_parser().parse_args(
        ['replay', str(log), '--set', 'position.covariance_floor_m=2.0'])
    assert cmd_replay(tweaked) == 0
    tweaked_out = capsys.readouterr().out
    assert default_out != tweaked_out, 'a tunable change must move the numbers'
    assert 'NEES' in tweaked_out


def test_replay_rejects_an_unknown_tunable(log):
    args = build_parser().parse_args(['replay', str(log), '--set', 'position.not_a_knob=1.0'])
    with pytest.raises(SystemExit, match='unknown tunable'):
        cmd_replay(args)


def test_replay_rejects_a_non_numeric_value(log):
    args = build_parser().parse_args(
        ['replay', str(log), '--set', 'position.covariance_floor_m=banana'])
    with pytest.raises(SystemExit):
        cmd_replay(args)


def test_replay_without_poses_fails_loudly(tmp_path):
    """Silently replaying at the origin would produce plausible, meaningless numbers."""
    path = tmp_path / 'noposes.jsonl'
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write(json.dumps({'type': 'meta', 'seed': 0, 'truth_elements': []}) + '\n')
        handle.write(json.dumps({'type': 'frame', 't': 0.0, 'boat': None}) + '\n')
    with pytest.raises(SystemExit, match='no boat poses'):
        replay(read_records(path), ScorerConfig(), None)


# -- summarise ----------------------------------------------------------------------------------

def test_summarise_reports_what_is_in_the_log(log, capsys):
    args = build_parser().parse_args(['summarise', str(log)])
    assert cmd_summarise(args) == 0
    out = capsys.readouterr().out
    assert '120 logged detections' in out
    assert 'carrying a CV class: 120' in out


def test_summarise_warns_when_cv_never_scored_well(tmp_path, capsys):
    """The camera0 case. Without this warning, the next person tunes a class posterior that
    never received any evidence."""
    path = tmp_path / 'weak.jsonl'
    records = [json.loads(line) for line in open(write_log(tmp_path / 'run.jsonl'),
                                                 encoding='utf-8')]
    for r in records:
        if r.get('type') == 'detection':
            r['class_score'] = 0.08
    with open(path, 'w', encoding='utf-8') as handle:
        for r in records:
            handle.write(json.dumps(r) + '\n')

    args = build_parser().parse_args(['summarise', str(path)])
    assert cmd_summarise(args) == 0
    assert 'no detection ever scored above 0.3' in capsys.readouterr().out
