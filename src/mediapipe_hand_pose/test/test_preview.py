from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import pytest

from mediapipe_hand_pose.hand_pose_node import MediaPipeHandPoseNode


def test_mirror_preview_flips_image_without_changing_input():
    node = SimpleNamespace(mirror_preview=True)
    frame = np.asarray([[[1, 2, 3], [4, 5, 6]]], dtype=np.uint8)

    debug_frame, debug_landmarks = MediaPipeHandPoseNode._prepare_debug_frame(
        node, frame
    )

    assert debug_landmarks is None
    assert debug_frame.tolist() == [[[4, 5, 6], [1, 2, 3]]]
    assert frame.tolist() == [[[1, 2, 3], [4, 5, 6]]]


def test_disabled_mirror_keeps_image_direction():
    node = SimpleNamespace(mirror_preview=False)
    frame = np.asarray([[[1, 2, 3], [4, 5, 6]]], dtype=np.uint8)

    debug_frame, _ = MediaPipeHandPoseNode._prepare_debug_frame(node, frame)

    assert debug_frame.tolist() == frame.tolist()
    assert debug_frame is not frame


def test_angle_calculation_prefers_world_landmarks():
    image_points = np.zeros((21, 3))
    world_points = np.ones((21, 3))
    expected = {'index_pip_flexion': 0.5}

    with patch(
        'mediapipe_hand_pose.hand_pose_node.calculate_joint_angles',
        return_value=expected,
    ) as calculate:
        angles, used_fallback = (
            MediaPipeHandPoseNode._calculate_angles_with_fallback(
                image_points, world_points
            )
        )

    assert angles == expected
    assert used_fallback is False
    calculate.assert_called_once_with(world_points)


def test_angle_calculation_falls_back_to_image_landmarks():
    image_points = np.zeros((21, 3))
    world_points = np.ones((21, 3))
    expected = {'index_pip_flexion': 0.5}

    with patch(
        'mediapipe_hand_pose.hand_pose_node.calculate_joint_angles',
        side_effect=[ValueError('world failed'), expected],
    ) as calculate:
        angles, used_fallback = (
            MediaPipeHandPoseNode._calculate_angles_with_fallback(
                image_points, world_points
            )
        )

    assert angles == expected
    assert used_fallback is True
    assert calculate.call_count == 2


def test_angle_calculation_reports_both_failures():
    with patch(
        'mediapipe_hand_pose.hand_pose_node.calculate_joint_angles',
        side_effect=[ValueError('world failed'), ValueError('image failed')],
    ):
        with pytest.raises(ValueError, match='图像关键点也失败'):
            MediaPipeHandPoseNode._calculate_angles_with_fallback(
                np.zeros((21, 3)), np.ones((21, 3))
            )


def test_angle_summary_contains_o6_thumb_calibration_values():
    frame = np.zeros((240, 640, 3), dtype=np.uint8)
    angles = {
        'thumb_cmc_abduction': 0.1,
        'thumb_cmc_flexion': 0.2,
        'thumb_mcp_flexion': 0.3,
    }

    with patch('mediapipe_hand_pose.hand_pose_node.cv2.putText') as put_text:
        MediaPipeHandPoseNode._draw_angle_summary(frame, angles)

    labels = [call.args[1] for call in put_text.call_args_list]
    assert any(label.startswith('Thumb Abd:') for label in labels)
    assert any(label.startswith('Thumb CMC:') for label in labels)
    assert any(label.startswith('Thumb MCP:') for label in labels)
