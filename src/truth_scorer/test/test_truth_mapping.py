"""Truth mapping, checked against the real generated answer keys.

These tests read the actual course files in the repo. If the course generator grows a new element
kind, the scorer must learn about it rather than silently scoring it as a perception failure --
which would look identical to a broken detector and cost an afternoon to diagnose.
"""

import json
import pathlib

import pytest

from truth_scorer.core.classes import CLASS_NAMES, UNKNOWN_ID
from truth_scorer.core.truth_mapping import (
    TRUTH_MAP,
    label_for,
    load_truth_elements,
    unmapped_kinds,
)

# test/ -> truth_scorer/ -> src/
SRC_DIR = pathlib.Path(__file__).resolve().parents[2]
COURSE_DIR = SRC_DIR / 'asv_wave_sim' / 'course' / 'generated'
WORLDS_DIR = SRC_DIR / 'asv_wave_sim' / 'gz-waves-models' / 'worlds'


def _answer_keys():
    found = []
    if COURSE_DIR.is_dir():
        found.extend(sorted(COURSE_DIR.glob('*/*.answer_key.json')))
    if WORLDS_DIR.is_dir():
        found.extend(sorted(WORLDS_DIR.glob('*.answer_key.json')))
    return found


def test_the_real_answer_keys_were_actually_found():
    """Guards the skipif above. A silently skipped validation suite is worse than none, because
    it reads as a pass."""
    assert _answer_keys(), (
        f'no answer keys found under {COURSE_DIR} or {WORLDS_DIR}; the course-validation tests '
        'below would silently skip')


def test_mapping_only_references_real_classes():
    valid = set(CLASS_NAMES) | {UNKNOWN_ID}
    for key, label in TRUTH_MAP.items():
        assert label.canonical_class_id in valid, f'{key} has a bogus canonical class'
        assert label.acceptable_class_ids <= valid, f'{key} has bogus acceptable classes'
        assert label.canonical_class_id in label.acceptable_class_ids, (
            f'{key}: canonical class is not in its own acceptable set')


def test_colours_are_from_a_known_vocabulary():
    allowed = {'red', 'green', 'black', 'yellow', None}
    for key, label in TRUTH_MAP.items():
        assert label.colour in allowed, f'{key} has unexpected colour {label.colour!r}'


def test_label_lookup_handles_missing_colour():
    assert label_for('dock', None) is not None
    assert label_for('banner', None) is not None
    assert label_for('not_a_real_kind', 'red') is None


@pytest.mark.skipif(not _answer_keys(), reason='no generated course found in the repo')
def test_every_element_in_every_generated_course_is_mapped():
    problems = []
    for path in _answer_keys():
        key = json.loads(path.read_text(encoding='utf-8'))
        missing = unmapped_kinds(key)
        if missing:
            problems.append(f'{path.parent.name}: {missing}')
    assert not problems, 'unmapped answer-key kinds:\n  ' + '\n  '.join(problems)


@pytest.mark.skipif(not _answer_keys(), reason='no generated course found in the repo')
def test_loads_the_real_answer_key():
    path = _answer_keys()[0]
    key = json.loads(path.read_text(encoding='utf-8'))
    elements = load_truth_elements(key)

    assert len(elements) == len(key['elements'])
    assert all(e.scoreable for e in elements)
    # The course is mostly buoys, so most elements must be point-like or position metrics
    # will be computed over almost nothing.
    assert sum(e.point_like for e in elements) > len(elements) / 2


@pytest.mark.skipif(not _answer_keys(), reason='no generated course found in the repo')
def test_gate_buoys_resolve_to_opposite_colours():
    """A gate the scorer cannot tell apart by colour is a gate it cannot drive through."""
    path = _answer_keys()[0]
    key = json.loads(path.read_text(encoding='utf-8'))
    by_id = {e.element_id: e for e in load_truth_elements(key)}

    gates = key.get('gates') or []
    assert gates, 'answer key has no gates'
    for gate in gates:
        red = by_id[gate['red_id']]
        green = by_id[gate['green_id']]
        assert red.label.colour == 'red'
        assert green.label.colour == 'green'
        assert red.label.acceptable_class_ids.isdisjoint(green.label.acceptable_class_ids), (
            f"gate {gate.get('label')}: red and green share acceptable classes, so a correct "
            'answer for one would score as correct for the other')


def test_class_correctness_accepts_the_equivalence_group():
    """red_buoy vs red_pole_buoy is an aspect-ratio distinction the answer key does not encode,
    so scoring it as an error would measure our bookkeeping, not the perception."""
    label = label_for('sur_mark', 'red')
    assert 3 in label.acceptable_class_ids    # red_buoy
    assert 4 in label.acceptable_class_ids    # red_pole_buoy
    assert 1 not in label.acceptable_class_ids   # green_buoy is a genuine error


def test_colour_correctness_is_strict():
    from truth_scorer.core.truth_mapping import TruthElement
    label = label_for('sur_mark', 'red')
    e = TruthElement(1, 'x', 1, 'sur_mark', 'red', 0.0, 0.0, label)
    assert e.colour_is_correct('red')
    assert not e.colour_is_correct('green')
    assert not e.colour_is_correct(None)
