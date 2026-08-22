import pytest

from linkerhand_retargeting.mapping import (
    JointMapping,
    angle_to_radians,
    radians_to_angle,
)


def make_mapping(**overrides):
    values = {
        'target_joint': 'test_joint',
        'source_angle': 'test_angle',
        'input_min': 0.0,
        'input_max': 2.0,
        'output_min': 0.0,
        'output_max': 1.0,
        'joint_min': 0.0,
        'joint_max': 0.8,
    }
    values.update(overrides)
    return JointMapping(**values)


def test_linear_mapping_and_joint_limit():
    mapping = make_mapping()

    assert mapping.map_angle(0.0) == pytest.approx(0.0)
    assert mapping.map_angle(1.0) == pytest.approx(0.5)
    assert mapping.map_angle(2.0) == pytest.approx(0.8)


def test_input_is_clamped_before_mapping():
    mapping = make_mapping(joint_max=2.0)

    assert mapping.map_angle(-1.0) == pytest.approx(0.0)
    assert mapping.map_angle(3.0) == pytest.approx(1.0)


def test_inverted_mapping():
    mapping = make_mapping(invert=True, joint_max=2.0)

    assert mapping.map_angle(0.0) == pytest.approx(1.0)
    assert mapping.map_angle(2.0) == pytest.approx(0.0)


def test_fixed_joint_is_clamped():
    mapping = make_mapping(
        source_angle='', fixed_position=2.0, joint_min=-0.5, joint_max=0.5
    )

    assert mapping.map_angle(123.0) == pytest.approx(0.5)


def test_invalid_input_range_raises_error():
    mapping = make_mapping(input_min=1.0, input_max=1.0)

    with pytest.raises(ValueError):
        mapping.map_angle(1.0)


def test_degree_configuration_converts_to_radians():
    assert angle_to_radians(180.0, 'deg') == pytest.approx(3.141592653589793)
    assert radians_to_angle(3.141592653589793, 'deg') == pytest.approx(180.0)
    assert angle_to_radians(1.25, 'rad') == pytest.approx(1.25)


def test_invalid_angle_unit_raises_error():
    with pytest.raises(ValueError):
        angle_to_radians(1.0, 'grad')
