import numpy as np

from mediapipe_hand_pose.one_euro_filter import OneEuroFilter


def test_first_sample_passes_through():
    filter_ = OneEuroFilter()
    value = np.asarray([1.0, 2.0, 3.0])

    output = filter_.filter(value, 1.0)

    assert np.allclose(output, value)


def test_constant_signal_converges_smoothly():
    filter_ = OneEuroFilter(min_cutoff=0.8, beta=0.0)
    filter_.filter(np.asarray([0.0]), 0.0)

    first = filter_.filter(np.asarray([1.0]), 0.1)[0]
    second = filter_.filter(np.asarray([1.0]), 0.2)[0]

    assert 0.0 < first < second < 1.0


def test_beta_reduces_delay_for_fast_motion():
    fixed_filter = OneEuroFilter(min_cutoff=0.8, beta=0.0)
    adaptive_filter = OneEuroFilter(min_cutoff=0.8, beta=0.5)
    fixed_filter.filter(np.asarray([0.0]), 0.0)
    adaptive_filter.filter(np.asarray([0.0]), 0.0)

    fixed_output = fixed_filter.filter(np.asarray([1.0]), 0.05)[0]
    adaptive_output = adaptive_filter.filter(np.asarray([1.0]), 0.05)[0]

    assert adaptive_output > fixed_output


def test_reset_accepts_new_pose_immediately():
    filter_ = OneEuroFilter()
    filter_.filter(np.asarray([0.0]), 0.0)
    filter_.filter(np.asarray([1.0]), 0.1)
    filter_.reset()

    output = filter_.filter(np.asarray([5.0]), 1.0)

    assert np.allclose(output, np.asarray([5.0]))


def test_filter_reduces_static_measurement_jitter():
    filter_ = OneEuroFilter(min_cutoff=0.8, beta=0.3)
    measurements = np.asarray(
        [0.0, 0.03, -0.02, 0.025, -0.015, 0.02, -0.01, 0.015]
    )
    outputs = [
        filter_.filter(np.asarray([value]), index / 15.0)[0]
        for index, value in enumerate(measurements)
    ]

    assert np.std(outputs[2:]) < np.std(measurements[2:])
