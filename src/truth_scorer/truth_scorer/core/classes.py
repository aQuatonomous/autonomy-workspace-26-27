"""The class vocabulary, mirrored from computer_vision/class_mapping.yaml.

Kept as a literal here (rather than read from the YAML at import time) so that core/ has no file
dependencies and the unit tests are hermetic. `load_class_mapping` can override it at runtime if
the YAML ever drifts; `test_classes.py` checks the two agree.
"""

from __future__ import annotations

from typing import Dict, List, Optional

UNKNOWN_ID = 255
UNKNOWN_NAME = 'unknown'

# class_id -> class_name, matching computer_vision/class_mapping.yaml exactly.
CLASS_NAMES: Dict[int, str] = {
    0: 'black_buoy',
    1: 'green_buoy',
    2: 'green_pole_buoy',
    3: 'red_buoy',
    4: 'red_pole_buoy',
    5: 'yellow_buoy',
    6: 'cross',
    7: 'dock',
    8: 'triangle',
    9: 'red_indicator_buoy',
    10: 'green_indicator_buoy',
    11: 'yellow_supply_drop',
    12: 'black_supply_drop',
    20: 'digit_1',
    21: 'digit_2',
    22: 'digit_3',
}

# Stable ordering for the posterior vector. The trailing UNKNOWN_ID is a real category: it absorbs
# "something is here that is none of the above", which is what keeps the posterior honest when the
# CV model is shown an object it was never trained on.
CLASS_IDS: List[int] = sorted(CLASS_NAMES) + [UNKNOWN_ID]
NUM_CLASSES: int = len(CLASS_IDS)
CLASS_INDEX: Dict[int, int] = {cid: i for i, cid in enumerate(CLASS_IDS)}

# Colour is what the RoboBoat handbook actually scores, so it gets first-class treatment.
CLASS_COLOUR: Dict[int, Optional[str]] = {
    0: 'black',
    1: 'green',
    2: 'green',
    3: 'red',
    4: 'red',
    5: 'yellow',
    6: None,
    7: None,
    8: None,
    9: 'red',
    10: 'green',
    11: 'yellow',
    12: 'black',
    20: None,
    21: None,
    22: None,
    UNKNOWN_ID: None,
}


def class_name(class_id: int) -> str:
    return CLASS_NAMES.get(int(class_id), UNKNOWN_NAME)


def class_colour(class_id: int) -> Optional[str]:
    return CLASS_COLOUR.get(int(class_id))


def colour_is_expressible(colour: Optional[str], canonical_class_id: int) -> bool:
    """Does the class the detector is *expected* to emit for this element carry its colour?

    Asked against the canonical class, not the acceptable set. 'red' is expressible in general
    -- red_buoy is red -- but a Task 5 dock indicator is a red *cross*, and the canonical answer
    for it is `cross`, which is colourless in class_mapping.yaml. A detector doing its job
    perfectly still reports no colour for that element.

    The acceptable set does list the red buoy classes, but only as scoring leniency; it is not
    what a correct detector emits, so it is the wrong thing to ask. Elements that fail this test
    are counted separately rather than scored as colour misses: counting them would drag colour
    accuracy down *and* pin it, since no improvement in CV could ever move them.
    """
    if colour is None:
        return False
    return CLASS_COLOUR.get(int(canonical_class_id)) == colour


def index_of(class_id: int) -> int:
    """Posterior-vector index for a class id. Unrecognised ids fall into the unknown slot."""
    return CLASS_INDEX.get(int(class_id), CLASS_INDEX[UNKNOWN_ID])


# Groups of classes that a vision model realistically confuses with one another. Used to build the
# default confusion prior: within-group confusion is far more likely than across-group.
CONFUSABLE_GROUPS: List[List[int]] = [
    [1, 2, 10],      # green buoy / green pole buoy / green indicator
    [3, 4, 9],       # red buoy / red pole buoy / red indicator
    [0, 12],         # black buoy / black supply drop
    [5, 11],         # yellow buoy / yellow supply drop
    [6, 8],          # cross / triangle (both flat dock placards)
    [20, 21, 22],    # the three digits
    [7],             # dock
]
