import pytest

from linkerhand_retargeting.filters import (
    JointDeadbandHysteresis,
    exponential_moving_average,
    move_towards,
)


def make_deadband(**overrides):
    parameters = {
        'initial_value': 0.0,
        'start_threshold': 1.5,
        'stop_threshold': 0.5,
        'settle_frames': 3,
    }
    parameters.update(overrides)
    return JointDeadbandHysteresis(**parameters)


def test_deadband_holds_small_stationary_jitter():
    deadband = make_deadband()

    assert deadband.update(0.4) == pytest.approx(0.0)
    assert deadband.update(-0.7) == pytest.approx(0.0)
    assert deadband.update(1.49) == pytest.approx(0.0)
    assert deadband.moving is False


def test_deadband_starts_at_outer_threshold_and_follows_motion():
    deadband = make_deadband()

    assert deadband.update(1.5) == pytest.approx(1.5)
    assert deadband.moving is True
    assert deadband.update(2.4) == pytest.approx(2.4)


def test_deadband_relocks_after_stable_window():
    deadband = make_deadband()

    deadband.update(2.0)
    deadband.update(2.2)
    locked_value = (2.0 + 2.2 + 2.3) / 3.0
    assert deadband.update(2.3) == pytest.approx(locked_value)
    assert deadband.moving is False
    assert deadband.update(2.7) == pytest.approx(locked_value)


def test_deadband_does_not_relock_while_window_is_still_moving():
    deadband = make_deadband()

    deadband.update(2.0)
    deadband.update(2.4)
    deadband.update(2.8)
    assert deadband.moving is True
    assert deadband.update(3.2) == pytest.approx(3.2)


def test_deadband_reset_replaces_locked_center():
    deadband = make_deadband()
    deadband.update(2.0)

    deadband.reset(-1.0)

    assert deadband.output == pytest.approx(-1.0)
    assert deadband.moving is False
    assert deadband.update(0.0) == pytest.approx(-1.0)


@pytest.mark.parametrize(
    'overrides',
    [
        {'start_threshold': 0.0},
        {'start_threshold': float('nan')},
        {'stop_threshold': -0.1},
        {'start_threshold': 0.5, 'stop_threshold': 0.5},
        {'settle_frames': 1},
    ],
)
def test_deadband_rejects_invalid_configuration(overrides):
    with pytest.raises(ValueError):
        make_deadband(**overrides)


def test_exponential_moving_average():
    assert exponential_moving_average(0.0, 1.0, 0.25) == pytest.approx(0.25)
    assert exponential_moving_average(0.25, 1.0, 0.25) == pytest.approx(0.4375)


def test_filter_alpha_is_clamped():
    assert exponential_moving_average(0.2, 1.0, -1.0) == pytest.approx(0.2)
    assert exponential_moving_average(0.2, 1.0, 2.0) == pytest.approx(1.0)


def test_move_towards_limits_positive_and_negative_steps():
    assert move_towards(0.0, 1.0, 0.2) == pytest.approx(0.2)
    assert move_towards(1.0, 0.0, 0.2) == pytest.approx(0.8)
    assert move_towards(0.0, 0.1, 0.2) == pytest.approx(0.1)


def test_non_positive_step_disables_velocity_limit():
    assert move_towards(0.0, 1.0, 0.0) == pytest.approx(1.0)
