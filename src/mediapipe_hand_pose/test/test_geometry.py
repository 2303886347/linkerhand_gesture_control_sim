import math

import numpy as np

from mediapipe_hand_pose.geometry import angle_between, flexion_angle


def test_angle_between_orthogonal_vectors():
    assert math.isclose(
        angle_between(np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0])),
        math.pi / 2.0,
    )


def test_straight_joint_has_zero_flexion():
    assert math.isclose(
        flexion_angle(
            np.array([-1.0, 0.0, 0.0]),
            np.array([0.0, 0.0, 0.0]),
            np.array([1.0, 0.0, 0.0]),
        ),
        0.0,
    )


def test_right_angle_joint_has_ninety_degree_flexion():
    assert math.isclose(
        flexion_angle(
            np.array([-1.0, 0.0, 0.0]),
            np.array([0.0, 0.0, 0.0]),
            np.array([0.0, 1.0, 0.0]),
        ),
        math.pi / 2.0,
    )
