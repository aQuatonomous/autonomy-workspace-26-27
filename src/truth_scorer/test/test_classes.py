"""The class vocabulary must not drift from computer_vision/class_mapping.yaml.

core/classes.py hard-codes the mapping so the scoring maths has no file dependencies. That is a
deliberate trade: the cost is that the copy can go stale, so the staleness is tested directly.
"""

import pathlib

import pytest
import yaml

from truth_scorer.core.classes import (
    CLASS_COLOUR,
    CLASS_IDS,
    CLASS_NAMES,
    CONFUSABLE_GROUPS,
    NUM_CLASSES,
    UNKNOWN_ID,
    class_colour,
    class_name,
    index_of,
)

REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
MAPPING_PATH = REPO_ROOT / 'computer_vision' / 'class_mapping.yaml'


def test_mapping_file_exists():
    assert MAPPING_PATH.is_file(), (
        f'{MAPPING_PATH} not found; the consistency test below would silently skip')


def test_class_names_match_the_yaml():
    data = yaml.safe_load(MAPPING_PATH.read_text(encoding='utf-8'))
    from_yaml = {int(k): str(v) for k, v in data['classes'].items()}
    assert CLASS_NAMES == from_yaml, (
        'core/classes.py has drifted from computer_vision/class_mapping.yaml')


def test_buoy_colours_match_the_yaml():
    data = yaml.safe_load(MAPPING_PATH.read_text(encoding='utf-8'))
    name_to_id = {str(v): int(k) for k, v in data['classes'].items()}
    for name, colour in (data.get('buoy_colors') or {}).items():
        cid = name_to_id[name]
        assert class_colour(cid) == colour, f'{name}: colour drifted from the YAML'


def test_unknown_is_present_and_last():
    assert CLASS_IDS[-1] == UNKNOWN_ID
    assert NUM_CLASSES == len(CLASS_NAMES) + 1


def test_indices_are_dense_and_unique():
    indices = [index_of(c) for c in CLASS_IDS]
    assert sorted(indices) == list(range(NUM_CLASSES))


def test_unrecognised_ids_fall_into_unknown():
    assert index_of(9999) == index_of(UNKNOWN_ID)
    assert class_name(9999) == 'unknown'


def test_every_class_has_a_colour_entry():
    for cid in CLASS_IDS:
        assert cid in CLASS_COLOUR, f'class {cid} has no colour entry'


def test_confusable_groups_cover_every_class_exactly_once():
    seen = [c for group in CONFUSABLE_GROUPS for c in group]
    assert len(seen) == len(set(seen)), 'a class appears in two confusable groups'
    assert set(seen) == set(CLASS_NAMES), 'confusable groups do not cover the vocabulary'


@pytest.mark.parametrize('cid,expected', [(3, 'red'), (4, 'red'), (1, 'green'),
                                          (0, 'black'), (5, 'yellow'), (7, None)])
def test_spot_check_colours(cid, expected):
    assert class_colour(cid) == expected
