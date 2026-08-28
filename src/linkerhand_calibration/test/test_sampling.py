import math
from pathlib import Path

import pytest

from linkerhand_calibration.calibration import load_parameters, required_sources
from linkerhand_calibration.sampling import (
    FAIL_INSUFFICIENT_SAMPLES,
    FAIL_LOW_VALID_RATIO,
    FAIL_UNSTABLE,
    INVALID_HAND_CLIPPED,
    PoseSampleCollector,
    evaluate_pose_samples,
    extract_valid_sample,
    percentile,
)


CONFIG_DIR = (
    Path(__file__).resolve().parents[2]
    / 'linkerhand_retargeting'
    / 'config'
)


def load_template(model_id='o6', side='left'):
    filename = (
        f'retargeting_{side}.yaml'
        if model_id == 'l30'
        else f'retargeting_{model_id}_{side}.yaml'
    )
    return load_parameters(CONFIG_DIR / filename)[1]


def make_pose(parameters, value_deg=20.0, **overrides):
    names = list(required_sources(parameters))
    values = [math.radians(float(value_deg)) for _ in names]
    pose = {
        'detected': True,
        'handedness': 'left',
        'confidence': 0.9,
        'landmarks_visible': True,
        'joint_names': tuple(names),
        'joint_angles': tuple(values),
    }
    pose.update(overrides)
    return pose


def test_percentile_uses_linear_interpolation():
    assert percentile([0.0, 10.0], 90.0) == pytest.approx(9.0)
    assert percentile([0.0, 10.0], 10.0) == pytest.approx(1.0)


def test_pose_validation_converts_required_angles_to_degrees():
    parameters = load_template()
    sample, reason = extract_valid_sample(
        parameters, make_pose(parameters, 30.0), 'left', 0.5
    )

    assert reason == ''
    assert sample['index_mcp_flexion'] == pytest.approx(30.0)


def test_pose_validation_rejects_clipped_hand():
    parameters = load_template()
    sample, reason = extract_valid_sample(
        parameters,
        make_pose(parameters, landmarks_visible=False),
        'left',
    )

    assert sample is None
    assert reason == INVALID_HAND_CLIPPED


def test_o6_stability_uses_runtime_mcp_pip_fusion():
    parameters = load_template()
    sources = required_sources(parameters)
    samples = []
    for index in range(20):
        sample = {source: 20.0 for source in sources}
        sample['index_mcp_flexion'] = 20.0 + index * 0.1
        sample['index_pip_flexion'] = 40.0 + index * 0.1
        samples.append(sample)

    result = evaluate_pose_samples(
        parameters,
        samples,
        total_frames=20,
        maximum_spread_deg=8.0,
    )

    assert result.success
    assert result.summary['index_mcp_pitch'] == pytest.approx(33.95)


def test_sampling_rejects_insufficient_valid_frames_with_reason():
    parameters = load_template()
    collector = PoseSampleCollector(parameters, 'left')
    for _ in range(10):
        collector.add_pose(make_pose(parameters))
    for _ in range(10):
        collector.add_pose(make_pose(parameters, detected=False))

    result = collector.finish(minimum_samples=15)

    assert not result.success
    assert result.reason_code == FAIL_INSUFFICIENT_SAMPLES
    assert result.dominant_invalid_reason == 'no_hand'


def test_sampling_rejects_low_valid_ratio_after_minimum_count():
    parameters = load_template()
    collector = PoseSampleCollector(parameters, 'left')
    for _ in range(15):
        collector.add_pose(make_pose(parameters))
    for _ in range(15):
        collector.add_pose(make_pose(parameters, detected=False))

    result = collector.finish(minimum_samples=15, minimum_valid_ratio=0.7)

    assert not result.success
    assert result.reason_code == FAIL_LOW_VALID_RATIO
    assert result.valid_ratio == pytest.approx(0.5)


def test_sampling_rejects_motion_using_p90_p10():
    parameters = load_template(model_id='l30')
    sources = required_sources(parameters)
    samples = []
    for index in range(20):
        sample = {source: 20.0 for source in sources}
        sample['index_pip_flexion'] = float(index * 2)
        samples.append(sample)

    result = evaluate_pose_samples(
        parameters,
        samples,
        total_frames=20,
        maximum_spread_deg=8.0,
    )

    assert not result.success
    assert result.reason_code == FAIL_UNSTABLE
    assert result.reason_detail == 'index_pip'
    assert result.spreads['index_pip'] > 8.0


def test_single_outlier_does_not_dominate_stability_metric():
    parameters = load_template(model_id='l30')
    sources = required_sources(parameters)
    samples = [{source: 20.0 for source in sources} for _ in range(20)]
    samples[-1]['index_pip_flexion'] = 120.0

    result = evaluate_pose_samples(
        parameters,
        samples,
        total_frames=20,
        maximum_spread_deg=8.0,
    )

    assert result.success
