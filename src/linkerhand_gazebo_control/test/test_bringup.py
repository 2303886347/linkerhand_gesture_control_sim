"""验证参数式 Gazebo 入口按手侧生成隔离话题。"""

import pytest

from linkerhand_gazebo_control.bringup import resolve_gazebo_topics


@pytest.mark.parametrize('side', ['left', 'right'])
def test_empty_topics_follow_selected_side(side):
    topics = resolve_gazebo_topics(side)

    assert topics.pose == f'/{side}/mediapipe/hand_pose'
    assert topics.angles == f'/{side}/mediapipe/human_joint_angles'
    assert topics.debug_image == f'/{side}/mediapipe/debug_image'
    assert topics.target_joint == f'/{side}/linkerhand/target_joint_states'
    assert topics.status == f'/{side}/linkerhand/retargeting_status'


def test_explicit_topics_are_preserved():
    topics = resolve_gazebo_topics(
        'right',
        pose_topic='/custom/pose',
        target_joint_topic='/custom/target',
    )

    assert topics.pose == '/custom/pose'
    assert topics.target_joint == '/custom/target'
    assert topics.angles == '/right/mediapipe/human_joint_angles'


def test_invalid_side_is_rejected():
    with pytest.raises(ValueError, match='left 或 right'):
        resolve_gazebo_topics('both')
