"""验证多型号 RViz 编排对四种左右组合的解析。"""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from linkerhand_retargeting.bringup import resolve_rviz_hand_spec


PACKAGE_SHARE = Path(__file__).parents[1]


@pytest.mark.parametrize(
    ('left_model', 'right_model', 'left_config', 'right_config'),
    [
        ('l30', 'l30', 'retargeting_left.yaml', 'retargeting_right.yaml'),
        ('o6', 'o6', 'retargeting_o6_left.yaml', 'retargeting_o6_right.yaml'),
        ('l30', 'o6', 'retargeting_left.yaml', 'retargeting_o6_right.yaml'),
        ('o6', 'l30', 'retargeting_o6_left.yaml', 'retargeting_right.yaml'),
    ],
)
def test_all_registered_dual_hand_model_combinations(
    left_model, right_model, left_config, right_config
):
    left = resolve_rviz_hand_spec(left_model, 'left', PACKAGE_SHARE)
    right = resolve_rviz_hand_spec(right_model, 'right', PACKAGE_SHARE)

    assert left.profile.model_id == left_model
    assert left.profile.side == 'left'
    assert left.parameters_file.is_file()
    assert left.parameters_file.name == left_config
    assert ET.fromstring(left.robot_description).get('name') == (
        f'linkerhand_{left_model}_left'
    )

    assert right.profile.model_id == right_model
    assert right.profile.side == 'right'
    assert right.parameters_file.is_file()
    assert right.parameters_file.name == right_config
    assert ET.fromstring(right.robot_description).get('name') == (
        f'linkerhand_{right_model}_right'
    )


def test_explicit_parameters_file_overrides_model_default(tmp_path):
    custom_parameters = tmp_path / 'personal_o6_left.yaml'
    custom_parameters.write_text('/**:\n  ros__parameters: {}\n', encoding='utf-8')

    spec = resolve_rviz_hand_spec(
        'o6', 'left', PACKAGE_SHARE, custom_parameters
    )

    assert spec.parameters_file == custom_parameters
    assert spec.uses_profile_defaults is False


def test_missing_explicit_parameters_file_fails_clearly(tmp_path):
    with pytest.raises(FileNotFoundError, match='显式指定的重定向配置不存在'):
        resolve_rviz_hand_spec(
            'o6',
            'right',
            PACKAGE_SHARE,
            tmp_path / 'missing.yaml',
        )


def test_missing_default_parameters_falls_back_to_profile(tmp_path):
    spec = resolve_rviz_hand_spec('o6', 'left', tmp_path)

    assert spec.parameters_file is None
    assert spec.uses_profile_defaults is True
