"""Resolve the colcon workspace root so CV asset paths work from any clone location."""
from __future__ import annotations

import os
from pathlib import Path
from typing import Optional


def _path_from_env() -> Optional[Path]:
    raw = os.environ.get('AUTONOMY_WS')
    if not raw:
        return None
    p = Path(raw).expanduser().resolve()
    return p if p.is_dir() else None


def _is_workspace_root(p: Path) -> bool:
    return (p / 'computer_vision').is_dir() and (p / 'src').is_dir()


def resolve_workspace_root(start: Optional[Path] = None) -> Path:
    """Workspace = directory containing top-level ``src/`` and ``computer_vision/``."""
    env_path = _path_from_env()
    if env_path is not None and _is_workspace_root(env_path):
        return env_path
    if start is None:
        start = Path(__file__).resolve().parent
    p = start.resolve()
    for _ in range(14):
        if _is_workspace_root(p):
            return p
        parent = p.parent
        if parent == p:
            break
        p = parent
    if env_path is not None:
        return env_path
    for legacy in (
        Path.home() / 'Repos' / 'School' / 'autonomy-ws-25-26',
        Path.home() / 'autonomy-ws-25-26',
    ):
        if legacy.is_dir():
            return legacy.resolve()
    return Path.home() / 'Repos' / 'School' / 'autonomy-ws-25-26'


def computer_vision_root() -> Path:
    return resolve_workspace_root() / 'computer_vision'


def default_engine_path() -> str:
    return str(computer_vision_root() / 'model_building_and_training' / 'model.engine')


def default_number_engine_path() -> str:
    return str(computer_vision_root() / 'model_building_and_training' / 'number_detection.engine')


def default_class_mapping_path() -> str:
    return str(computer_vision_root() / 'class_mapping.yaml')
