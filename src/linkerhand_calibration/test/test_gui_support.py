from pathlib import Path

import pytest

from linkerhand_calibration.gui_support import (
    PerceptionSettings,
    VerificationSettings,
    build_perception_command,
    build_verification_command,
    calibration_topics,
    list_video_devices,
    normalize_model,
    normalize_side,
)


@pytest.mark.parametrize(
    ('side', 'expected_prefix'),
    [('left', '/left/calibration'), ('right', '/right/calibration')],
)
def test_calibration_topics_are_isolated_by_side(side, expected_prefix):
    topics = calibration_topics(side)

    assert topics == {
        'pose': f'{expected_prefix}/hand_pose',
        'angles': f'{expected_prefix}/human_joint_angles',
        'debug_image': f'{expected_prefix}/debug_image',
    }


def test_perception_command_reuses_gui_topics_without_preview_window():
    program, arguments = build_perception_command(PerceptionSettings(
        side='right',
        device='/dev/video2',
        width=1280,
        height=720,
        camera_fps=30.0,
        processing_fps=20.0,
        mirror_preview=False,
    ))

    assert Path(program).name == 'setsid'
    assert Path(arguments[0]).name == 'ros2'
    assert arguments[1:4] == (
        'launch', 'mediapipe_hand_pose', 'pipeline.launch.py'
    )
    assert 'device:=/dev/video2' in arguments
    assert 'target_hand:=right' in arguments
    assert 'pose_topic:=/right/calibration/hand_pose' in arguments
    assert 'debug_image_topic:=/right/calibration/debug_image' in arguments
    assert 'mediapipe_show_preview:=false' in arguments
    assert 'mirror_preview:=false' in arguments


@pytest.mark.parametrize(
    ('mode', 'launch_file'),
    [
        ('rviz', 'mediapipe_rviz_single.launch.py'),
        ('gazebo', 'mediapipe_gazebo.launch.py'),
    ],
)
def test_verification_command_uses_selected_profile_and_camera(
    mode, launch_file, tmp_path
):
    profile = tmp_path / 'personal.yaml'
    profile.touch()

    program, arguments = build_verification_command(VerificationSettings(
        mode=mode,
        model_id='O6',
        side='Right',
        parameters_file=str(profile),
        device='/dev/video4',
        width=1280,
        height=720,
        camera_fps=30.0,
        processing_fps=12.0,
        mirror_preview=False,
    ))

    assert Path(program).name == 'setsid'
    assert Path(arguments[0]).name == 'ros2'
    assert arguments[1:4] == ('launch', 'linkerhand_bringup', launch_file)
    assert 'model_id:=o6' in arguments
    assert 'side:=right' in arguments
    assert f'parameters_file:={profile.resolve()}' in arguments
    assert 'device:=/dev/video4' in arguments
    assert 'width:=1280' in arguments
    assert 'height:=720' in arguments
    assert 'processing_fps:=12' in arguments
    assert 'mediapipe_show_preview:=true' in arguments
    assert 'mirror_preview:=false' in arguments


def test_verification_command_rejects_missing_profile(tmp_path):
    with pytest.raises(FileNotFoundError):
        build_verification_command(VerificationSettings(
            mode='rviz',
            model_id='l30',
            side='left',
            parameters_file=str(tmp_path / 'missing.yaml'),
            device='/dev/video0',
        ))


def test_video_devices_use_natural_numeric_order(tmp_path):
    for name in ('video10', 'video2', 'video1'):
        (tmp_path / name).touch()

    devices = list_video_devices(str(tmp_path / 'video*'))

    assert tuple(Path(value).name for value in devices) == (
        'video1', 'video2', 'video10'
    )


@pytest.mark.parametrize(('value', 'expected'), [('L30', 'l30'), ('O6', 'o6')])
def test_model_normalization(value, expected):
    assert normalize_model(value) == expected


@pytest.mark.parametrize(('value', 'expected'), [('LEFT', 'left'), ('Right', 'right')])
def test_side_normalization(value, expected):
    assert normalize_side(value) == expected


@pytest.mark.parametrize(
    ('function', 'value'),
    [(normalize_model, 'unknown'), (normalize_side, 'both')],
)
def test_invalid_profile_values_are_rejected(function, value):
    with pytest.raises(ValueError):
        function(value)
