"""验证 O6 官方模型的自由度、联动关系和 ROS 2 资源完整性。"""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest

from linkerhand_model_profiles import expand_joint_positions, load_model_profile


O6_ACTIVE_JOINTS = (
    'thumb_cmc_yaw',
    'thumb_cmc_pitch',
    'index_mcp_pitch',
    'middle_mcp_pitch',
    'ring_mcp_pitch',
    'pinky_mcp_pitch',
)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_o6_profile_matches_official_six_dof_structure(side):
    profile = load_model_profile('o6', side)

    assert profile.model_id == 'o6'
    assert profile.side == side
    assert profile.active_joints == O6_ACTIVE_JOINTS
    assert len(profile.mimic_joints) == 5
    assert not profile.locked_joints
    assert len(profile.full_joints) == 11
    assert profile.root_link == 'base_footprint'


def test_o6_keeps_official_side_specific_thumb_mimic_ratios():
    left = load_model_profile('o6', 'left')
    right = load_model_profile('o6', 'right')

    assert left.mimic_joints['thumb_ip'].multiplier == pytest.approx(2.29)
    assert right.mimic_joints['thumb_ip'].multiplier == pytest.approx(1.86)


def test_o6_left_thumb_control_limit_respects_mimic_joint_limit():
    profile = load_model_profile('o6', 'left')

    # 主动关节上限为 0.58 rad，但 2.29 倍联动会先触及 IP 的 1.08 rad 上限。
    expected_upper = 1.08 / 2.29
    assert profile.urdf_joint_limits['thumb_cmc_pitch'] == pytest.approx(
        (0.0, 0.58)
    )
    assert profile.joint_limits['thumb_cmc_pitch'] == pytest.approx(
        (0.0, expected_upper)
    )
    assert math.degrees(expected_upper) == pytest.approx(27.02, abs=0.01)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_o6_expansion_applies_all_official_mimic_relations(side):
    profile = load_model_profile('o6', side)
    positions = {
        joint: profile.joint_limits[joint][1] * 0.5
        for joint in profile.active_joints
    }

    expanded = expand_joint_positions(profile, positions)

    assert set(expanded) == set(profile.full_joints)
    for joint, settings in profile.mimic_joints.items():
        expected = (
            positions[settings.source] * settings.multiplier + settings.offset
        )
        assert expanded[joint] == pytest.approx(expected)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_o6_installed_urdf_resolves_every_mesh(side):
    package = f'linkerhand_o6_{side}_description'
    package_share = Path(get_package_share_directory(package))
    profile = load_model_profile('o6', side)
    robot = ET.parse(profile.urdf_path).getroot()
    meshes = robot.findall('.//mesh')

    assert len(meshes) == 24
    resolved_paths = set()
    for mesh in meshes:
        prefix = f'package://{package}/'
        filename = mesh.get('filename', '')
        assert filename.startswith(prefix)
        resolved_path = package_share / filename.removeprefix(prefix)
        assert resolved_path.is_file()
        resolved_paths.add(resolved_path)
    assert len(resolved_paths) == 12
