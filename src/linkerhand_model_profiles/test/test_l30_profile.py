import math

import pytest

from linkerhand_model_profiles import (
    ProfileNotFoundError,
    expand_joint_positions,
    load_model_profile,
)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_l30_profile_matches_validated_joint_structure(side):
    profile = load_model_profile('l30', side)

    assert profile.model_id == 'l30'
    assert profile.side == side
    assert len(profile.active_joints) == 17
    assert len(profile.mimic_joints) == 4
    assert profile.locked_joints == {'thumb_cmc_roll': 0.0}
    assert len(profile.full_joints) == 22
    assert profile.root_link == 'base_footprint'
    assert profile.joint_limits['wrist_pitch'] == pytest.approx((-1.05, 1.05))


def test_l30_left_and_right_keep_existing_gazebo_inertial_scales():
    assert load_model_profile('l30', 'left').gazebo_inertial_scale == pytest.approx(
        1.0 / 7.6
    )
    assert load_model_profile('l30', 'right').gazebo_inertial_scale == pytest.approx(1.0)


def test_expand_joint_positions_applies_l30_mimic_and_locked_joints():
    profile = load_model_profile('l30', 'left')
    positions = {joint: index / 100.0 for index, joint in enumerate(profile.active_joints)}

    expanded = expand_joint_positions(profile, positions)

    assert expanded['thumb_cmc_roll'] == pytest.approx(0.0)
    for mimic_joint, settings in profile.mimic_joints.items():
        assert expanded[mimic_joint] == pytest.approx(positions[settings.source])


def test_l30_mapping_defaults_preserve_previous_code_values():
    profile = load_model_profile('l30', 'left')

    index = profile.mapping_defaults['index_pip']
    assert index.source_angle == 'index_pip_flexion'
    assert index.input_min == pytest.approx(0.2617994)
    assert index.input_max == pytest.approx(1.3089969)
    assert index.output_max == pytest.approx(1.57)
    thumb = profile.mapping_defaults['thumb_mcp']
    assert thumb.input_min == pytest.approx(math.radians(5.0), abs=1.0e-6)
    assert thumb.output_max == pytest.approx(math.radians(85.0), abs=1.0e-6)


def test_unknown_model_is_rejected():
    with pytest.raises(ProfileNotFoundError, match='未注册型号'):
        load_model_profile('unknown', 'left')
