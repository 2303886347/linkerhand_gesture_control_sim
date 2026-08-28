from pathlib import Path

import pytest

from linkerhand_calibration.calibration import (
    attach_profile_metadata,
    build_profile_metadata,
    load_parameters,
    write_calibration,
)
from linkerhand_calibration.profile_store import (
    load_profile,
    profile_filename,
    scan_profiles,
    unique_profile_path,
    validate_profile_name,
)


RETARGETING_CONFIG = (
    Path(__file__).resolve().parents[2]
    / 'linkerhand_retargeting'
    / 'config'
)


def template(model_id, side):
    filename = (
        f'retargeting_{side}.yaml'
        if model_id == 'l30'
        else f'retargeting_{model_id}_{side}.yaml'
    )
    return load_parameters(RETARGETING_CONFIG / filename)[0]


@pytest.mark.parametrize('name', ['', '   ', 'bad/name', 'bad\\name'])
def test_profile_name_rejects_empty_or_path_like_values(name):
    with pytest.raises(ValueError):
        validate_profile_name(name)


def test_unicode_profile_name_has_stable_safe_filename():
    filename = profile_filename('o6', 'right', '书桌摄像头')

    assert filename.startswith('o6_right_profile_')
    assert filename.endswith('.yaml')


def test_unique_profile_path_never_overwrites_existing_file(tmp_path):
    first = unique_profile_path(tmp_path, 'l30', 'left', 'demo')
    first.touch()
    second = unique_profile_path(tmp_path, 'l30', 'left', 'demo')

    assert first.name == 'l30_left_demo.yaml'
    assert second.name == 'l30_left_demo_2.yaml'


def test_scan_profiles_filters_model_side_and_reads_metadata(tmp_path):
    left_document = attach_profile_metadata(
        template('o6', 'left'),
        build_profile_metadata(
            'O6 left desk', 'o6', 'left', '/dev/video0', {}, {}
        ),
    )
    right_document = attach_profile_metadata(
        template('o6', 'right'),
        build_profile_metadata(
            'O6 right desk', 'o6', 'right', '/dev/video0', {}, {}
        ),
    )
    left_path = write_calibration(tmp_path / 'left.yaml', left_document)
    write_calibration(tmp_path / 'right.yaml', right_document)
    (tmp_path / 'broken.yaml').write_text('not: [yaml', encoding='utf-8')

    profiles, errors = scan_profiles('o6', 'left', directory=tmp_path)

    assert len(profiles) == 1
    assert profiles[0].path == left_path
    assert profiles[0].name == 'O6 left desk'
    assert len(errors) == 1


def test_legacy_ros_parameter_yaml_can_be_imported(tmp_path):
    path = write_calibration(tmp_path / 'legacy.yaml', template('l30', 'right'))

    profile = load_profile(path, external=True)

    assert profile.legacy is True
    assert profile.external is True
    assert profile.model_id == 'l30'
    assert profile.side == 'right'
