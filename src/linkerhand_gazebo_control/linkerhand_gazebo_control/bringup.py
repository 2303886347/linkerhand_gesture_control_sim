"""解析单手 Gazebo 管线按手侧隔离的话题。"""

from dataclasses import dataclass


@dataclass(frozen=True)
class GazeboTopics:
    pose: str
    angles: str
    debug_image: str
    target_joint: str
    status: str


def resolve_gazebo_topics(
    side,
    pose_topic='',
    angles_topic='',
    debug_image_topic='',
    target_joint_topic='',
    status_topic='',
):
    """为空的话题按 left/right 自动生成，显式话题保持不变。"""
    normalized_side = str(side).strip().lower()
    if normalized_side not in {'left', 'right'}:
        raise ValueError('side 只能是 left 或 right')

    def selected(value, suffix):
        value = str(value).strip()
        return value or f'/{normalized_side}/{suffix}'

    return GazeboTopics(
        pose=selected(pose_topic, 'mediapipe/hand_pose'),
        angles=selected(angles_topic, 'mediapipe/human_joint_angles'),
        debug_image=selected(debug_image_topic, 'mediapipe/debug_image'),
        target_joint=selected(
            target_joint_topic, 'linkerhand/target_joint_states'
        ),
        status=selected(status_topic, 'linkerhand/retargeting_status'),
    )
