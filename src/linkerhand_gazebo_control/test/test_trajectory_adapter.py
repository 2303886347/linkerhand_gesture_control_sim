import math

import pytest
from sensor_msgs.msg import JointState

from linkerhand_gazebo_control.joints import CONTROLLED_JOINTS
from linkerhand_gazebo_control.trajectory_adapter import build_trajectory
from linkerhand_model_profiles import load_model_profile
from linkerhand_retargeting.joints import MIMIC_JOINTS


def make_target():
    message = JointState()
    message.name = [
        joint
        for joint in CONTROLLED_JOINTS
        if joint != 'thumb_cmc_roll' and joint not in MIMIC_JOINTS
    ]
    message.position = [float(index) / 100.0 for index in range(len(message.name))]
    return message


def test_build_trajectory_adds_locked_joint_and_duration():
    trajectory = build_trajectory(make_target(), 0.15)

    assert trajectory.joint_names == list(CONTROLLED_JOINTS)
    assert len(trajectory.points) == 1
    point = trajectory.points[0]
    locked_index = trajectory.joint_names.index('thumb_cmc_roll')
    assert point.positions[locked_index] == pytest.approx(0.0)
    for mimic, source in MIMIC_JOINTS.items():
        mimic_index = trajectory.joint_names.index(mimic)
        source_index = trajectory.joint_names.index(source)
        assert point.positions[mimic_index] == pytest.approx(
            point.positions[source_index]
        )
    assert point.time_from_start.sec == 0
    assert point.time_from_start.nanosec == 150_000_000


def test_build_trajectory_rejects_missing_driven_joint():
    message = make_target()
    message.name.pop()
    message.position.pop()

    with pytest.raises(ValueError, match='缺少控制关节'):
        build_trajectory(message, 0.15)


def test_build_trajectory_rejects_invalid_arrays_and_values():
    message = make_target()
    message.position.pop()
    with pytest.raises(ValueError, match='长度不一致'):
        build_trajectory(message, 0.15)

    message = make_target()
    message.position[0] = math.nan
    with pytest.raises(ValueError, match='非有限'):
        build_trajectory(message, 0.15)


@pytest.mark.parametrize(
    ('side', 'thumb_multiplier'),
    [('left', 2.29), ('right', 1.86)],
)
def test_o6_trajectory_expands_six_active_joints(side, thumb_multiplier):
    profile = load_model_profile('o6', side)
    message = JointState()
    message.name = list(profile.active_joints)
    message.position = [0.2, 0.3, 0.4, 0.5, 0.6, 0.7]

    trajectory = build_trajectory(message, 0.15, profile)
    point = trajectory.points[0]
    positions = dict(zip(trajectory.joint_names, point.positions))

    assert trajectory.joint_names == list(profile.full_joints)
    assert len(point.positions) == 11
    assert positions['thumb_ip'] == pytest.approx(
        positions['thumb_cmc_pitch'] * thumb_multiplier
    )
    for finger in ('index', 'middle', 'ring', 'pinky'):
        assert positions[f'{finger}_dip'] == pytest.approx(
            positions[f'{finger}_mcp_pitch'] * 0.89
        )
