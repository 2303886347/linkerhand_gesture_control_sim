"""锁定 L30/O6 单手 Gazebo 快捷入口的型号和手侧。"""

import importlib.util
from pathlib import Path

import pytest


LAUNCH_DIR = Path(__file__).parents[1] / 'launch'


def _launch_arguments(filename):
    path = LAUNCH_DIR / filename
    spec = importlib.util.spec_from_file_location(
        filename.replace('.', '_'), path
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    launch_description = module.generate_launch_description()
    include = list(launch_description.entities)[0]
    return dict(include.launch_arguments)


@pytest.mark.parametrize(
    ('filename', 'model_id', 'side'),
    [
        ('mediapipe_gazebo_left.launch.py', 'l30', 'left'),
        ('mediapipe_gazebo_right.launch.py', 'l30', 'right'),
        ('mediapipe_gazebo_o6_left.launch.py', 'o6', 'left'),
        ('mediapipe_gazebo_o6_right.launch.py', 'o6', 'right'),
    ],
)
def test_shortcut_selects_expected_model_and_side(filename, model_id, side):
    arguments = _launch_arguments(filename)

    assert arguments['model_id'] == model_id
    assert arguments['side'] == side
