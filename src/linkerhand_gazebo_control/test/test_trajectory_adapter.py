import math

import pytest
from sensor_msgs.msg import JointState

from linkerhand_gazebo_control.joints import CONTROLLED_JOINTS
from linkerhand_gazebo_control.trajectory_adapter import build_trajectory
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
