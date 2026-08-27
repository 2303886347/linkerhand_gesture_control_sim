"""锁定多型号 RViz 快捷入口与实际左右型号组合。"""

import importlib.util
from pathlib import Path

from launch import LaunchContext
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
    ('filename', 'left_model', 'right_model'),
    [
        ('mediapipe_rviz_l30_both.launch.py', 'l30', 'l30'),
        ('mediapipe_rviz_o6_both.launch.py', 'o6', 'o6'),
        ('mediapipe_rviz_l30_o6.launch.py', 'l30', 'o6'),
        ('mediapipe_rviz_o6_l30.launch.py', 'o6', 'l30'),
    ],
)
def test_shortcut_selects_expected_model_pair(
    filename, left_model, right_model
):
    arguments = _launch_arguments(filename)

    assert arguments['left_model'] == left_model
    assert arguments['right_model'] == right_model


@pytest.mark.parametrize(
    'filename',
    [
        'mediapipe_gazebo_l30_left.launch.py',
        'mediapipe_gazebo_l30_right.launch.py',
        'mediapipe_gazebo_o6_left.launch.py',
        'mediapipe_gazebo_o6_right.launch.py',
    ],
)
def test_gazebo_shortcuts_include_registered_control_launch(filename):
    path = LAUNCH_DIR / filename
    spec = importlib.util.spec_from_file_location(
        filename.replace('.', '_'), path
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    launch_description = module.generate_launch_description()
    include = list(launch_description.entities)[0]
    included_description = (
        include.launch_description_source.get_launch_description(
            LaunchContext()
        )
    )

    assert 'linkerhand_gazebo_control' in include.launch_description_source.location
    assert included_description.entities
