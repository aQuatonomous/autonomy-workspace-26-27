"""YAML loading for the scorer's tunables and confusion prior.

Deliberately lives *outside* core/: core does the maths and touches no files, so it stays
unit-testable with nothing installed. This module is the only place that knows about disk, and it
imports no ROS either, so analyse_run.py can use it offline.

Every loader here fails loudly. A config file that silently half-applies is worse than no config
file, because the run looks fine and the numbers are wrong.
"""

from __future__ import annotations

import os
from typing import List, Optional, Sequence

import yaml

from .core.classes import CLASS_IDS, NUM_CLASSES, index_of
from .core.config import ScorerConfig
from .core.confusion import _normalise_in_place, default_confusion_matrix


def load_scorer_config(path: Optional[str]) -> ScorerConfig:
    """Load ScorerConfig from a plain nested YAML. Empty path or missing file -> defaults."""
    if not path:
        return ScorerConfig()
    if not os.path.isfile(path):
        raise FileNotFoundError(f'scorer config file not found: {path}')
    with open(path, 'r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    # ScorerConfig.from_dict raises on unknown sections/keys, which is the point.
    return ScorerConfig.from_dict(data or {})


def load_confusion_matrix(path: Optional[str]) -> List[List[float]]:
    """Load a confusion prior, or build the hand-made default if there is no file.

    File format (what save_confusion_matrix writes):

        class_ids: [0, 1, ..., 255]     # row/column order, must match core.classes.CLASS_IDS
        matrix:
          - [...]                       # NUM_CLASSES rows of NUM_CLASSES floats

    The class_ids list is not decoration: it is the check that a matrix fitted before the class
    vocabulary changed cannot be silently applied in the wrong order afterwards.
    """
    if not path or not os.path.isfile(path):
        return default_confusion_matrix()
    with open(path, 'r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}

    if data.get('use_default'):
        return default_confusion_matrix(**(data.get('default_params') or {}))

    ids = data.get('class_ids')
    if ids is not None and [int(i) for i in ids] != list(CLASS_IDS):
        raise ValueError(
            f'{path}: class_ids do not match core.classes.CLASS_IDS. The matrix was fitted '
            'against a different class vocabulary and applying it would permute the rows.')

    matrix = data.get('matrix')
    if not matrix:
        raise ValueError(f'{path}: no "matrix" key and no "use_default: true"')
    if len(matrix) != NUM_CLASSES or any(len(r) != NUM_CLASSES for r in matrix):
        raise ValueError(
            f'{path}: matrix must be {NUM_CLASSES}x{NUM_CLASSES}, got '
            f'{len(matrix)}x{len(matrix[0]) if matrix else 0}')

    out = [[float(v) for v in row] for row in matrix]
    for row in out:
        if any(v < 0.0 for v in row):
            raise ValueError(f'{path}: negative probability in confusion matrix')
        _normalise_in_place(row)
    return out


def save_confusion_matrix(path: str, matrix: Sequence[Sequence[float]], note: str = '') -> None:
    """Write a fitted matrix back out in the format load_confusion_matrix reads."""
    from .core.classes import class_name

    lines = [
        '# Confusion prior: P(CV reports class j | the object really is class i).',
        '#',
        '# Row and column order is core.classes.CLASS_IDS, pinned below so a vocabulary change',
        '# cannot silently permute this matrix.',
    ]
    if note:
        lines.extend('# ' + part for part in note.splitlines())
    lines.append('')
    lines.append('class_ids: [' + ', '.join(str(c) for c in CLASS_IDS) + ']')
    lines.append('matrix:')
    for cid in CLASS_IDS:
        row = matrix[index_of(cid)]
        body = ', '.join(f'{v:.6f}' for v in row)
        lines.append(f'  - [{body}]   # true = {class_name(cid)}')
    lines.append('')
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(lines))
