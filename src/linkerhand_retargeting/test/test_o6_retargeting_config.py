"""锁定 O6 左右手的六自由度融合标定配置。"""

import math
from pathlib import Path

import pytest
import yaml


CONFIG_DIR = Path(__file__).parents[1] / 'config'


def _parameters(side):
    data = yaml.safe_load(
        (CONFIG_DIR / f'retargeting_o6_{side}.yaml').read_text(
            encoding='utf-8'
        )
    )
    return data['/**']['ros__parameters']


@pytest.mark.parametrize('side', ['left', 'right'])
def test_o6_configuration_drives_exactly_six_active_joints(side):
    parameters = _parameters(side)
    mapping = parameters['mapping']

    assert parameters['model_id'] == 'o6'
    assert parameters['model_side'] == side
    assert parameters['accepted_hand'] == side
    assert set(mapping) == {
        'thumb_cmc_yaw',
        'thumb_cmc_pitch',
        'index_mcp_pitch',
        'middle_mcp_pitch',
        'ring_mcp_pitch',
        'pinky_mcp_pitch',
    }


@pytest.mark.parametrize('side', ['left', 'right'])
def test_o6_four_fingers_use_independent_mcp_pip_fusion(side):
    mapping = _parameters(side)['mapping']

    for finger in ('index', 'middle', 'ring', 'pinky'):
        settings = mapping[f'{finger}_mcp_pitch']
        assert settings['sources'] == [
            f'{finger}_mcp_flexion',
            f'{finger}_pip_flexion',
        ]
        assert settings['source_weights'] == pytest.approx([0.35, 0.65])
        assert settings['output_min'] == pytest.approx(0.0)
        assert settings['output_max'] == pytest.approx(90.0)


def test_o6_thumb_outputs_respect_profile_safe_limits():
    left = _parameters('left')['mapping']
    right = _parameters('right')['mapping']

    assert left['thumb_cmc_pitch']['output_max'] == pytest.approx(
        math.degrees(min(0.58, 1.08 / 2.29)), abs=0.01
    )
    assert right['thumb_cmc_pitch']['output_max'] == pytest.approx(
        math.degrees(min(0.58, 1.08 / 1.86)), abs=0.01
    )
    assert left['thumb_cmc_yaw']['output_max'] == pytest.approx(
        math.degrees(1.30), abs=0.01
    )
    assert right['thumb_cmc_yaw']['output_max'] == pytest.approx(
        math.degrees(1.36), abs=0.01
    )


@pytest.mark.parametrize(
    ('side', 'input_min', 'input_max', 'output_max'),
    [
        ('left', 30.0, 80.0, math.degrees(1.30)),
        ('right', 30.0, 75.0, math.degrees(1.36)),
    ],
)
def test_o6_thumb_abduction_uses_measured_endpoints(
    side, input_min, input_max, output_max
):
    settings = _parameters(side)['mapping']['thumb_cmc_yaw']

    assert settings['sources'] == ['thumb_cmc_flexion']
    assert settings['source_weights'] == pytest.approx([1.0])
    assert settings['input_min'] == pytest.approx(input_min)
    assert settings['input_max'] == pytest.approx(input_max)
    assert settings['output_min'] == pytest.approx(0.0)
    assert settings['output_max'] == pytest.approx(output_max, abs=0.01)
    assert settings['invert'] is False

    def map_endpoint(value):
        ratio = (value - settings['input_min']) / (
            settings['input_max'] - settings['input_min']
        )
        return settings['output_min'] + ratio * (
            settings['output_max'] - settings['output_min']
        )

    assert map_endpoint(input_min) == pytest.approx(0.0)
    assert map_endpoint(input_max) == pytest.approx(output_max, abs=0.01)
