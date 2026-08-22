import pytest

from linkerhand_retargeting.filters import exponential_moving_average, move_towards


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
