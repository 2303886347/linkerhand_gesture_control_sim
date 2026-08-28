from pathlib import Path

import pytest
import yaml

from linkerhand_calibration.calibration import (
    POSE_FIST,
    POSE_OPEN,
    POSE_THUMB_IN,
    POSE_THUMB_OUT,
    attach_profile_metadata,
    build_personal_calibration,
    build_profile_metadata,
    calibration_endpoints,
    combine_mapping_input,
    driven_joints,
    extract_profile_metadata,
    load_parameters,
    required_sources,
    summarize_pose_samples,
    validate_calibration_ranges,
    write_calibration,
)
from linkerhand_calibration.sampling import PoseSamplingResult


RETARGETING_CONFIG = (
    Path(__file__).resolve().parents[2]
    / 'linkerhand_retargeting'
    / 'config'
)


def load_template(model_id, side):
    filename = (
        f'retargeting_{side}.yaml'
        if model_id == 'l30'
        else f'retargeting_{model_id}_{side}.yaml'
    )
    return load_parameters(RETARGETING_CONFIG / filename)


def make_pose(parameters, value_by_source):
    return {
        source: float(value_by_source.get(source, 10.0))
        for source in required_sources(parameters)
    }


def test_o6_mapping_input_uses_existing_mcp_pip_weights():
    _, parameters = load_template('o6', 'left')
    settings = parameters['mapping']['index_mcp_pitch']

    value = combine_mapping_input(
        settings,
        {'index_mcp_flexion': 20.0, 'index_pip_flexion': 60.0},
    )

    assert value == pytest.approx(46.0)


def test_pose_summary_uses_median_against_single_outlier():
    _, parameters = load_template('o6', 'left')
    samples = [
        make_pose(parameters, {}),
        make_pose(parameters, {}),
        make_pose(
            parameters,
            {
                'index_mcp_flexion': 150.0,
                'index_pip_flexion': 150.0,
            },
        ),
    ]

    summary = summarize_pose_samples(parameters, samples)

    assert summary['index_mcp_pitch'] == pytest.approx(10.0)


@pytest.mark.parametrize(
    ('model_id', 'side', 'expected_joint_count'),
    [
        ('l30', 'left', 10),
        ('l30', 'right', 10),
        ('o6', 'left', 6),
        ('o6', 'right', 6),
    ],
)
def test_all_registered_templates_expose_expected_driven_joints(
    model_id, side, expected_joint_count
):
    _, parameters = load_template(model_id, side)

    assert len(driven_joints(parameters)) == expected_joint_count
    assert required_sources(parameters)


def test_personal_calibration_updates_only_input_endpoints():
    document, parameters = load_template('o6', 'left')
    before = yaml.safe_load(yaml.safe_dump(document))
    open_pose = {
        joint: 10.0 + index
        for index, joint in enumerate(driven_joints(parameters))
    }
    fist_pose = {
        joint: 70.0 + index
        for index, joint in enumerate(driven_joints(parameters))
    }
    thumb_out = dict(open_pose)
    thumb_in = dict(fist_pose)
    thumb_out['thumb_cmc_yaw'] = 25.0
    thumb_in['thumb_cmc_yaw'] = 75.0

    result = build_personal_calibration(
        document,
        {
            POSE_OPEN: open_pose,
            POSE_FIST: fist_pose,
            POSE_THUMB_OUT: thumb_out,
            POSE_THUMB_IN: thumb_in,
        },
    )
    result_parameters = result['/**']['ros__parameters']

    assert result_parameters['mapping']['thumb_cmc_yaw']['input_min'] == 25.0
    assert result_parameters['mapping']['thumb_cmc_yaw']['input_max'] == 75.0
    assert result_parameters['mapping']['index_mcp_pitch']['input_min'] == 12.0
    assert result_parameters['mapping']['index_mcp_pitch']['input_max'] == 72.0
    template_parameters = before['/**']['ros__parameters']
    assert result_parameters['filter_alpha'] == template_parameters[
        'filter_alpha'
    ]
    for joint in driven_joints(parameters):
        assert result_parameters['mapping'][joint]['output_min'] == (
            before['/**']['ros__parameters']['mapping'][joint]['output_min']
        )
        assert result_parameters['mapping'][joint]['output_max'] == (
            before['/**']['ros__parameters']['mapping'][joint]['output_max']
        )


def test_calibration_rejects_reversed_or_too_small_range():
    document, parameters = load_template('l30', 'left')
    open_pose = {joint: 30.0 for joint in driven_joints(parameters)}
    fist_pose = {joint: 32.0 for joint in driven_joints(parameters)}

    with pytest.raises(ValueError, match='活动范围过小'):
        build_personal_calibration(
            document,
            {POSE_OPEN: open_pose, POSE_FIST: fist_pose},
            minimum_range_deg=5.0,
        )


def test_written_yaml_can_be_loaded_as_ros_parameters(tmp_path):
    document, parameters = load_template('l30', 'right')
    open_pose = {joint: 5.0 for joint in driven_joints(parameters)}
    fist_pose = {joint: 80.0 for joint in driven_joints(parameters)}
    result = build_personal_calibration(
        document,
        {POSE_OPEN: open_pose, POSE_FIST: fist_pose},
    )

    path = write_calibration(tmp_path / 'personal.yaml', result)
    loaded_document, loaded_parameters = load_parameters(path)

    assert loaded_document == result
    assert loaded_parameters['model_id'] == 'l30'
    assert loaded_parameters['model_side'] == 'right'


@pytest.mark.parametrize(
    ('model_id', 'side'),
    [
        ('l30', 'left'),
        ('l30', 'right'),
        ('o6', 'left'),
        ('o6', 'right'),
    ],
)
def test_personal_calibration_can_be_generated_for_every_registered_hand(
    model_id, side, tmp_path
):
    document, parameters = load_template(model_id, side)
    open_pose = {joint: 10.0 for joint in driven_joints(parameters)}
    fist_pose = {joint: 70.0 for joint in driven_joints(parameters)}
    thumb_out = dict(open_pose)
    thumb_in = dict(open_pose)
    if 'thumb_cmc_yaw' in driven_joints(parameters):
        thumb_out['thumb_cmc_yaw'] = 25.0
        thumb_in['thumb_cmc_yaw'] = 75.0

    result = build_personal_calibration(
        document,
        {
            POSE_OPEN: open_pose,
            POSE_FIST: fist_pose,
            POSE_THUMB_OUT: thumb_out,
            POSE_THUMB_IN: thumb_in,
        },
    )
    path = write_calibration(
        tmp_path / f'{model_id}_{side}.yaml', result
    )
    _, loaded = load_parameters(path)

    assert loaded['model_id'] == model_id
    assert loaded['model_side'] == side
    assert set(loaded['mapping']) == set(parameters['mapping'])


def test_o6_thumb_abduction_uses_out_and_in_pose_endpoints():
    _document, parameters = load_template('o6', 'right')
    open_pose = {joint: 10.0 for joint in driven_joints(parameters)}
    fist_pose = {joint: 70.0 for joint in driven_joints(parameters)}
    thumb_out = dict(open_pose)
    thumb_in = dict(open_pose)
    thumb_out['thumb_cmc_yaw'] = 12.0
    thumb_in['thumb_cmc_yaw'] = 38.0

    endpoints = calibration_endpoints(parameters, {
        POSE_OPEN: open_pose,
        POSE_FIST: fist_pose,
        POSE_THUMB_OUT: thumb_out,
        POSE_THUMB_IN: thumb_in,
    })

    assert endpoints['thumb_cmc_yaw'] == {
        'minimum_pose': POSE_THUMB_OUT,
        'maximum_pose': POSE_THUMB_IN,
        'input_min': 12.0,
        'input_max': 38.0,
        'range': 26.0,
    }


def test_joint_range_validation_reports_specific_reversed_joint():
    _document, parameters = load_template('l30', 'left')
    open_pose = {joint: 10.0 for joint in driven_joints(parameters)}
    fist_pose = {joint: 70.0 for joint in driven_joints(parameters)}
    fist_pose['index_pip'] = 5.0

    _endpoints, invalid = validate_calibration_ranges(
        parameters,
        {POSE_OPEN: open_pose, POSE_FIST: fist_pose},
        minimum_range_deg=5.0,
    )

    assert len(invalid) == 1
    assert invalid[0]['joint'] == 'index_pip'
    assert invalid[0]['reason'] == 'range_too_small_or_reversed'


def test_profile_metadata_stays_outside_ros_parameter_tree():
    document, _parameters = load_template('o6', 'left')
    result = PoseSamplingResult(
        success=True,
        summary={'index_mcp_pitch': 10.0},
        source_summary={'index_mcp_flexion': 12.0},
        spreads={'index_mcp_pitch': 1.25},
        valid_frames=20,
        total_frames=21,
        valid_ratio=20 / 21,
    )
    metadata = build_profile_metadata(
        'desk camera',
        'o6',
        'left',
        '/dev/video2',
        {POSE_OPEN: result},
        {'minimum_range_deg': 5.0},
    )

    with_metadata = attach_profile_metadata(document, metadata)

    assert 'calibration_profile' in with_metadata['/**']['ros__parameters']
    assert extract_profile_metadata(with_metadata)['name'] == 'desk camera'
    assert extract_profile_metadata(with_metadata)['samples'][POSE_OPEN][
        'valid_frames'
    ] == 20
    extract_profile_metadata,
