"""Structural guard: core/ must never import ROS.

The entire testing strategy depends on the scoring maths being runnable with no sourced ROS
environment, no colcon build and no simulator. That property is easy to destroy with one
convenience import, and the breakage shows up far from the cause -- so it is asserted directly.
"""

import ast
import pathlib
import subprocess
import sys

CORE_DIR = pathlib.Path(__file__).resolve().parent.parent / 'truth_scorer' / 'core'

FORBIDDEN_ROOTS = {
    'rclpy', 'rclcpp', 'ament_index_python', 'rosidl_runtime_py', 'tf2_ros', 'tf2_py',
    'std_msgs', 'sensor_msgs', 'geometry_msgs', 'builtin_interfaces', 'nav_msgs',
    'visualization_msgs', 'pointcloud_filters', 'global_frame', 'truth_scorer_msgs',
    'rosgraph_msgs', 'rclpy.node',
}


def _core_modules():
    return sorted(CORE_DIR.glob('*.py'))


def test_core_directory_is_not_empty():
    assert _core_modules(), f'no modules found in {CORE_DIR}'


def test_no_ros_imports_anywhere_in_core():
    offenders = []
    for path in _core_modules():
        tree = ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    root = alias.name.split('.')[0]
                    if root in FORBIDDEN_ROOTS:
                        offenders.append(f'{path.name}:{node.lineno} import {alias.name}')
            elif isinstance(node, ast.ImportFrom):
                if node.level:      # relative import, always fine
                    continue
                root = (node.module or '').split('.')[0]
                if root in FORBIDDEN_ROOTS:
                    offenders.append(f'{path.name}:{node.lineno} from {node.module}')
    assert not offenders, 'ROS imports found in core/:\n  ' + '\n  '.join(offenders)


def test_core_imports_cleanly_in_a_bare_interpreter():
    """Belt and braces: actually import every core module in a subprocess with ROS stripped from
    the path, so a dynamic or lazily-nested import cannot sneak past the AST check."""
    modules = [p.stem for p in _core_modules() if p.stem != '__init__']
    package_root = str(CORE_DIR.parent.parent)
    code = (
        'import sys\n'
        f'sys.path.insert(0, {package_root!r})\n'
        "sys.path = [p for p in sys.path if 'ros' not in p.lower()]\n"
        f'sys.path.insert(0, {package_root!r})\n'
        + '\n'.join(f'import truth_scorer.core.{m}' for m in modules)
        + '\nprint("ok")\n'
    )
    result = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True)
    assert result.returncode == 0, (
        f'core/ failed to import without ROS:\n{result.stdout}\n{result.stderr}')
    assert 'ok' in result.stdout


def test_core_does_not_read_files_at_import_time():
    """Importing the scoring maths must not depend on a YAML sitting in the right place."""
    offenders = []
    for path in _core_modules():
        tree = ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
        for node in tree.body:      # module level only
            for inner in ast.walk(node):
                if isinstance(inner, ast.Call) and isinstance(inner.func, ast.Name):
                    if inner.func.id == 'open':
                        offenders.append(f'{path.name}:{inner.lineno}')
    assert not offenders, f'module-level file reads in core/: {offenders}'
