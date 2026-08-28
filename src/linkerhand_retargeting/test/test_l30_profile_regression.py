"""用迁移前的固定常量验证 L30 profile 不改变数值映射。"""

import pytest

from linkerhand_model_profiles import load_model_profile
from linkerhand_retargeting.mapping import JointMapping


LEGACY_ACTIVE_JOINTS = (
    'wrist_pitch',
    'pinky_mcp_roll',
    'pinky_mcp_pitch',
    'pinky_pip',
    'ring_mcp_roll',
    'ring_mcp_pitch',
    'ring_pip',
    'middle_mcp_roll',
    'middle_mcp_pitch',
    'middle_pip',
    'index_mcp_roll',
    'index_mcp_pitch',
    'index_pip',
    'thumb_cmc_yaw',
    'thumb_cmc_pitch',
    'thumb_mcp',
    'thumb_dip',
)


LEGACY_DEFAULT_MAPPINGS = {
    'wrist_pitch': ('', 0.0, 1.0, 0.0, 0.0, False, 0.0, 0.0),
    'pinky_mcp_roll': ('', 0.0, 1.0, 0.0, 0.0, False, 0.0, 0.0),
    'pinky_mcp_pitch': (
        'pinky_mcp_flexion', 0.0, 1.20, 0.0, 1.40, False, 0.0, 0.0
    ),
    'pinky_pip': (
        'pinky_pip_flexion', 0.3490659, 1.3962634, 0.0, 1.57, False, 0.0, 0.0
    ),
    'ring_mcp_roll': ('', 0.0, 1.0, 0.0, 0.0, False, 0.0, 0.0),
    'ring_mcp_pitch': (
        'ring_mcp_flexion', 0.0, 1.20, 0.0, 1.40, False, 0.0, 0.0
    ),
    'ring_pip': (
        'ring_pip_flexion', 0.3490659, 1.3962634, 0.0, 1.57, False, 0.0, 0.0
    ),
    'middle_mcp_roll': ('', 0.0, 1.0, 0.0, 0.0, False, 0.0, 0.0),
    'middle_mcp_pitch': (
        'middle_mcp_flexion', 0.0, 1.20, 0.0, 1.40, False, 0.0, 0.0
    ),
    'middle_pip': (
        'middle_pip_flexion', 0.5235988, 1.4835299, 0.0, 1.57, False, 0.0, 0.0
    ),
    'index_mcp_roll': ('', 0.0, 1.0, 0.0, 0.0, False, 0.0, 0.0),
    'index_mcp_pitch': (
        'index_mcp_flexion', 0.0, 1.20, 0.0, 1.40, False, 0.0, 0.0
    ),
    'index_pip': (
        'index_pip_flexion', 0.2617994, 1.3089969, 0.0, 1.57, False, 0.0, 0.0
    ),
    'thumb_cmc_yaw': ('', 0.0, 1.0, 0.0, 0.0, False, 0.0, 0.0),
    'thumb_cmc_pitch': ('', 0.0, 1.0, 0.0, 0.0, False, 0.0, 0.0),
    'thumb_mcp': (
        'thumb_mcp_flexion', 0.0872665, 0.6108652, 0.0, 1.4835299, False, 0.0, 0.0
    ),
    'thumb_dip': (
        'thumb_ip_flexion', 0.0, 1.50, 0.0, 1.40, False, 0.0, 0.0
    ),
}


def _mapping_from_tuple(joint, values, limits):
    source, input_min, input_max, output_min, output_max, invert, fixed, _ = values
    return JointMapping(
        target_joint=joint,
        source_angle=source,
        input_min=input_min,
        input_max=input_max,
        output_min=output_min,
        output_max=output_max,
        joint_min=limits[0],
        joint_max=limits[1],
        invert=invert,
        fixed_position=fixed,
    )


def _mapping_from_profile(profile, joint):
    defaults = profile.mapping_defaults[joint]
    return JointMapping(
        target_joint=joint,
        source_angle=defaults.source_angle,
        input_min=defaults.input_min,
        input_max=defaults.input_max,
        output_min=defaults.output_min,
        output_max=defaults.output_max,
        joint_min=profile.joint_limits[joint][0],
        joint_max=profile.joint_limits[joint][1],
        invert=defaults.invert,
        fixed_position=defaults.fixed_position,
    )


@pytest.mark.parametrize('side', ['left', 'right'])
def test_l30_profile_keeps_legacy_joint_order_and_limits(side):
    profile = load_model_profile('l30', side)

    assert profile.active_joints == LEGACY_ACTIVE_JOINTS
    expected_limits = {
        'wrist_pitch': (-1.05, 1.05),
        'pinky_mcp_roll': (-0.26, 0.26),
        'pinky_mcp_pitch': (0.0, 1.57),
        'pinky_pip': (0.0, 1.57),
        'ring_mcp_roll': (-0.26, 0.26),
        'ring_mcp_pitch': (0.0, 1.57),
        'ring_pip': (0.0, 1.57),
        'middle_mcp_roll': (-0.26, 0.26),
        'middle_mcp_pitch': (0.0, 1.57),
        'middle_pip': (0.0, 1.57),
        'index_mcp_roll': (-0.26, 0.26),
        'index_mcp_pitch': (0.0, 1.57),
        'index_pip': (0.0, 1.57),
        'thumb_cmc_yaw': (0.0, 1.57),
        'thumb_cmc_pitch': (-0.39, 0.39),
        'thumb_mcp': (0.0, 1.57),
        'thumb_dip': (0.0, 1.57),
    }
    for joint, limits in expected_limits.items():
        assert profile.joint_limits[joint] == pytest.approx(limits)


@pytest.mark.parametrize('sample_ratio', [-0.25, 0.0, 0.5, 1.0, 1.25])
def test_l30_profile_mapping_matches_frozen_legacy_defaults(sample_ratio):
    profile = load_model_profile('l30', 'left')

    for joint in LEGACY_ACTIVE_JOINTS:
        legacy_values = LEGACY_DEFAULT_MAPPINGS[joint]
        legacy = _mapping_from_tuple(joint, legacy_values, profile.joint_limits[joint])
        current = _mapping_from_profile(profile, joint)
        sample = legacy.input_min + sample_ratio * (
            legacy.input_max - legacy.input_min
        )

        assert current.source_angle == legacy.source_angle
        assert current.map_angle(sample) == pytest.approx(
            legacy.map_angle(sample), abs=1.0e-12
        )
