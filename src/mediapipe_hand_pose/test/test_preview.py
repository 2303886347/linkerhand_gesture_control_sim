from types import SimpleNamespace

import numpy as np

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
