"""Answer-key vocabulary -> CV class vocabulary.

The course generator describes objects by (kind, colour) -- 'a0' is a Polyform A-0 ropehold ball,
'a2' a Polyform A-2, 'sur_mark' a surface navigation marker, and so on. The CV model knows a
different vocabulary (red_buoy, red_pole_buoy, ...). The two do not line up one-to-one, and
pretending they do would silently depress every class metric.

The honest mapping is therefore one-to-many: each truth element gets a *canonical* class plus a
set of classes that should be accepted as correct. 'red_buoy' versus 'red_pole_buoy' is a
distinction the CV model makes on bounding-box aspect ratio and the answer key simply does not
encode, so scoring it as an error would be measuring our own bookkeeping, not the perception.

Colour, which the handbook actually scores and which the key does record unambiguously, is
reported separately and is the number to trust.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, FrozenSet, List, Optional

from .classes import UNKNOWN_ID


@dataclass(frozen=True)
class TruthLabel:
    """What a course element 'is', in terms the scorer can be graded against."""

    canonical_class_id: int
    acceptable_class_ids: FrozenSet[int]
    colour: Optional[str]
    # Extended structures (docks, banners on a dock face, moored vessels) are not point objects.
    # A LiDAR cluster centroid on a 3 m dock is not meaningfully "wrong" by 1.5 m, so position
    # error and existence recall for these are reported separately rather than mixed in.
    point_like: bool = True
    notes: str = ''


def _label(canonical: int, acceptable: List[int], colour: Optional[str],
           point_like: bool = True, notes: str = '') -> TruthLabel:
    return TruthLabel(canonical, frozenset(acceptable), colour, point_like, notes)


# Colour-generic buoy equivalence sets: round buoy / pole buoy / indicator of the same colour.
_RED_BUOYS = [3, 4, 9]       # red_buoy, red_pole_buoy, red_indicator_buoy
_GREEN_BUOYS = [1, 2, 10]    # green_buoy, green_pole_buoy, green_indicator_buoy
_BLACK_BUOYS = [0, 12]       # black_buoy, black_supply_drop
_YELLOW_BUOYS = [5, 11]      # yellow_buoy, yellow_supply_drop

TRUTH_MAP: Dict[tuple, TruthLabel] = {
    # --- sur_mark: the large navigation markers used for gates -------------------------------
    ('sur_mark', 'red'): _label(3, _RED_BUOYS, 'red'),
    ('sur_mark', 'green'): _label(1, _GREEN_BUOYS, 'green'),

    # --- a0: Polyform A-0 ropehold, the small 0.10 m ball buoys -------------------------------
    ('a0', 'red'): _label(3, _RED_BUOYS, 'red'),
    ('a0', 'green'): _label(1, _GREEN_BUOYS, 'green'),
    ('a0', 'black'): _label(0, _BLACK_BUOYS, 'black'),
    ('a0', 'yellow'): _label(5, _YELLOW_BUOYS, 'yellow'),

    # --- a2: Polyform A-2, the larger 0.18 m buoys --------------------------------------------
    ('a2', 'red'): _label(3, _RED_BUOYS, 'red'),
    ('a2', 'green'): _label(1, _GREEN_BUOYS, 'green'),
    ('a2', 'black'): _label(0, _BLACK_BUOYS, 'black'),
    ('a2', 'yellow'): _label(5, _YELLOW_BUOYS, 'yellow'),

    # --- indicator_buoy: the 0.305 m colour indicators for tasks 2/3 --------------------------
    ('indicator_buoy', 'red'): _label(9, _RED_BUOYS, 'red'),
    ('indicator_buoy', 'green'): _label(10, _GREEN_BUOYS, 'green'),

    # --- task 5 marina ------------------------------------------------------------------------
    # DockIndicator placards carry the cross/triangle shapes as well as a colour, so both the
    # shape classes and the colour-matched buoy classes are defensible readings.
    ('dock_indicator', 'red'): _label(6, [6, 8] + _RED_BUOYS, 'red',
                                      notes='shape placard; colour is the reliable signal'),
    ('dock_indicator', 'green'): _label(6, [6, 8] + _GREEN_BUOYS, 'green',
                                        notes='shape placard; colour is the reliable signal'),
    ('banner', None): _label(20, [20, 21, 22], None, point_like=False,
                             notes='digit banner on the dock face; digit identity from the '
                                   'number model, not the main detector'),
    ('dock', None): _label(7, [7], None, point_like=False,
                           notes='extended structure; centroid position error is not meaningful'),

    # --- stationary vessels (task 2 debris, task 3 obstacle, task 5 slip occupants) -----------
    ('vessel', 'yellow'): _label(11, _YELLOW_BUOYS + [7], 'yellow', point_like=False,
                                 notes='extended hull'),
    ('vessel', 'black'): _label(12, _BLACK_BUOYS + [7], 'black', point_like=False,
                                notes='extended hull'),
}


def label_for(kind: str, colour: Optional[str]) -> Optional[TruthLabel]:
    """Look up the truth label for an answer-key element. None means 'not scoreable'."""
    key = (str(kind), colour if colour else None)
    label = TRUTH_MAP.get(key)
    if label is not None:
        return label
    # Unknown colour for a known kind: fall back to a colour-free entry if one exists.
    return TRUTH_MAP.get((str(kind), None))


@dataclass
class TruthElement:
    """One answer-key element, resolved into the answer-key (course) frame."""

    element_id: int
    name: str
    task: Optional[int]
    kind: str
    colour: Optional[str]
    east: float          # answer-key frame
    north: float
    label: Optional[TruthLabel]
    role: Optional[str] = None
    number: Optional[int] = None
    raw: dict = field(default_factory=dict)

    @property
    def scoreable(self) -> bool:
        return self.label is not None

    @property
    def point_like(self) -> bool:
        return bool(self.label and self.label.point_like)

    def class_is_correct(self, class_id: int) -> bool:
        if self.label is None:
            return False
        return int(class_id) in self.label.acceptable_class_ids

    def colour_is_correct(self, colour: Optional[str]) -> bool:
        if self.label is None or self.label.colour is None:
            return False
        return colour == self.label.colour

    @property
    def canonical_class_id(self) -> int:
        return self.label.canonical_class_id if self.label else UNKNOWN_ID


def load_truth_elements(answer_key: dict) -> List[TruthElement]:
    """Parse the `elements` array of a generated course answer key.

    Coordinates stay in the answer-key frame; converting to the robot's map frame is the
    evaluator's job, because the offset between them has to be measured at run time.
    """
    out: List[TruthElement] = []
    for e in (answer_key or {}).get('elements', []):
        kind = str(e.get('kind', ''))
        colour = e.get('color')
        out.append(TruthElement(
            element_id=int(e.get('id', -1)),
            name=str(e.get('name', '')),
            task=e.get('task'),
            kind=kind,
            colour=colour,
            east=float(e.get('x', 0.0)),
            north=float(e.get('y', 0.0)),
            label=label_for(kind, colour),
            role=e.get('role'),
            number=e.get('number'),
            raw=e,
        ))
    return out


def unmapped_kinds(answer_key: dict) -> List[tuple]:
    """(kind, colour) pairs present in the key that we have no label for.

    Surfaced loudly in the report: a silently unmapped kind looks exactly like a perception
    failure, and that is a very expensive hour to spend debugging.
    """
    missing = set()
    for e in (answer_key or {}).get('elements', []):
        kind, colour = str(e.get('kind', '')), e.get('color')
        if label_for(kind, colour) is None:
            missing.add((kind, colour))
    return sorted(missing, key=str)
