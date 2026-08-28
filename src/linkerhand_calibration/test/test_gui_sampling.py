import math
import os
import time

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import pytest

pytest.importorskip('PySide2')

from linkerhand_calibration.calibration import required_sources
from linkerhand_calibration.calibration_gui import CalibrationWindow
from linkerhand_calibration.sampling import PoseSamplingResult
from linkerhand_calibration.profile_store import load_profile


def make_valid_pose(window, value_deg=20.0):
    names = required_sources(window.template_parameters)
    return {
        'detected': True,
        'handedness': str(window.side_combo.currentData()),
        'confidence': 0.9,
        'processing_time_ms': 10.0,
        'joint_names': names,
        'joint_angles': tuple(math.radians(value_deg) for _ in names),
        'landmarks_visible': True,
    }


@pytest.fixture(scope='session')
def qt_app():
    from PySide2 import QtWidgets

    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture(scope='session')
def ros_context():
    import rclpy

    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def window(qt_app, ros_context):
    instance = CalibrationWindow()
    yield instance
    instance.shutdown()


def make_ready(window):
    now = time.monotonic()
    window.connection_state = 'connected'
    window.last_pose_time = now
    window.valid_detection_started = now - 1.0
    window.latest_pose = make_valid_pose(window)
    window._update_sample_controls()


def finish_success(window, value_deg=20.0):
    make_ready(window)
    window._start_sampling()
    pose = make_valid_pose(window, value_deg)
    for _ in range(20):
        window._on_pose_received(pose)
    window._finish_sampling()


def test_sampling_success_advances_without_auto_start(window):
    finish_success(window)

    assert window.pose_attempt_states['open_hand'] == 'sample_success'
    assert window.pose_list.currentRow() == 1
    assert window.sampling_pose is None
    assert '20/20' in window.sample_feedback_label.text()


def test_failed_resample_keeps_previous_success(window):
    finish_success(window)
    window.pose_list.setCurrentRow(0)
    make_ready(window)
    old_result = window.pose_results['open_hand']
    window._start_sampling()
    for _ in range(20):
        pose = make_valid_pose(window)
        pose['detected'] = False
        window._on_pose_received(pose)
    window._finish_sampling()

    assert window.pose_results['open_hand'] is old_result
    assert window.pose_attempt_states['open_hand'] == 'sample_success'
    assert '原有成功结果已保留' in window.sample_feedback_label.text()


def test_cancelled_first_sample_returns_to_not_sampled(window):
    make_ready(window)
    window._start_sampling()
    window._cancel_sampling()

    assert window.pose_attempt_states['open_hand'] == 'not_sampled'
    assert '已取消' in window.sample_feedback_label.text()


def successful_result(summary):
    return PoseSamplingResult(
        success=True,
        summary=summary,
        source_summary={},
        spreads={joint: 1.0 for joint in summary},
        valid_frames=20,
        total_frames=20,
        valid_ratio=1.0,
    )


def complete_l30_pose_results(window):
    joints = tuple(window.template_parameters['mapping'])
    driven = tuple(
        joint
        for joint in joints
        if window.template_parameters['mapping'][joint].get('source')
    )
    open_summary = {joint: 10.0 for joint in driven}
    fist_summary = {joint: 70.0 for joint in driven}
    window.pose_results = {
        'open_hand': successful_result(open_summary),
        'closed_fist': successful_result(fist_summary),
        'thumb_adducted': successful_result(open_summary),
        'thumb_abducted': successful_result(open_summary),
    }
    window.pose_attempt_states = {
        pose: 'sample_success' for pose in window.pose_order
    }
    window.samples_dirty = True
    window._refresh_pose_items()
    window._refresh_profile_results()


def test_complete_pose_set_enables_profile_generation(window):
    complete_l30_pose_results(window)

    assert window.generate_button.isEnabled()
    assert window.result_table.rowCount() == 10
    assert '可以生成个人配置' in window.profile_feedback_label.text()


def test_reversed_joint_range_disables_profile_generation(window):
    complete_l30_pose_results(window)
    old_result = window.pose_results['closed_fist']
    reversed_summary = dict(old_result.summary)
    reversed_summary['index_pip'] = 5.0
    window.pose_results['closed_fist'] = successful_result(reversed_summary)
    window._refresh_profile_results()

    assert not window.generate_button.isEnabled()
    assert '联合范围检查未通过' in window.profile_feedback_label.text()
    statuses = [
        window.result_table.item(row, 4).text()
        for row in range(window.result_table.rowCount())
    ]
    assert '范围过小/方向异常' in statuses


def test_new_profile_is_written_and_selected(
    window, tmp_path, monkeypatch
):
    import linkerhand_calibration.calibration_gui as calibration_gui

    complete_l30_pose_results(window)
    window.profile_name_edit.setText('pytest desk')
    monkeypatch.setattr(
        calibration_gui, 'DEFAULT_PROFILE_DIRECTORY', tmp_path
    )
    monkeypatch.setattr(
        calibration_gui.QtWidgets.QMessageBox,
        'information',
        lambda *_args, **_kwargs: calibration_gui.QtWidgets.QMessageBox.Ok,
    )

    window._save_profile()

    assert window.last_saved_profile_path.exists()
    assert window.profile_path_edit.text() == str(window.last_saved_profile_path)
    document, parameters = calibration_gui.load_parameters(
        window.last_saved_profile_path
    )
    assert parameters['mapping']['index_pip']['input_min'] == 10.0
    assert parameters['mapping']['index_pip']['input_max'] == 70.0
    assert document['/**']['ros__parameters'][
        'calibration_profile'
    ]['name'] == 'pytest desk'
    assert window.samples_dirty is False


def test_save_as_does_not_overwrite_selected_profile(
    window, tmp_path, monkeypatch
):
    import linkerhand_calibration.calibration_gui as calibration_gui

    complete_l30_pose_results(window)
    window.profile_name_edit.setText('first')
    monkeypatch.setattr(
        calibration_gui, 'DEFAULT_PROFILE_DIRECTORY', tmp_path
    )
    monkeypatch.setattr(
        calibration_gui.QtWidgets.QMessageBox,
        'information',
        lambda *_args, **_kwargs: calibration_gui.QtWidgets.QMessageBox.Ok,
    )
    window._save_profile()
    first_path = window.last_saved_profile_path

    complete_l30_pose_results(window)
    window.profile_name_edit.setText('second')
    monkeypatch.setattr(
        calibration_gui.QtWidgets.QFileDialog,
        'getSaveFileName',
        lambda *_args, **_kwargs: (str(tmp_path / 'second.yaml'), ''),
    )
    window._save_profile_as()
    second_path = window.last_saved_profile_path

    assert first_path.exists()
    assert second_path.exists()
    assert first_path != second_path


def test_repository_template_cannot_be_updated(window):
    complete_l30_pose_results(window)
    template_path = window._template_path('l30', 'left')
    window.current_profile = load_profile(template_path, external=True)
    window._update_profile_controls()

    assert not window.generate_button.isEnabled()
    assert window.save_as_button.isEnabled()
    assert '只读' in window.generate_button.toolTip()


def select_saved_profile(window):
    window.current_profile = load_profile(
        window._template_path('l30', 'left'), external=True
    )
    window.samples_dirty = False
    window._update_verification_controls()


def test_saved_profile_enables_both_verification_modes(window, tmp_path):
    select_saved_profile(window)
    camera = tmp_path / 'video0'
    camera.touch()
    window.camera_combo.setCurrentText(str(camera))

    assert window.rviz_button.isEnabled()
    assert window.gazebo_button.isEnabled()
    assert window.rviz_button.text() == 'RViz 验证'
    assert window.gazebo_button.text() == 'Gazebo 验证'


def test_unsaved_resampling_disables_verification(window):
    select_saved_profile(window)
    window.samples_dirty = True
    window._update_verification_controls()

    assert not window.rviz_button.isEnabled()
    assert not window.gazebo_button.isEnabled()
    assert '尚未保存' in window.verification_feedback_label.text()


def test_connected_verification_releases_camera_before_launch(
    window, monkeypatch, tmp_path
):
    select_saved_profile(window)
    camera = tmp_path / 'video0'
    camera.touch()
    window.camera_combo.setCurrentText(str(camera))
    window.connection_state = 'connected'
    calls = []
    monkeypatch.setattr(
        window, '_disconnect_perception', lambda: calls.append('disconnect')
    )
    monkeypatch.setattr(
        window,
        '_start_verification_process',
        lambda: calls.append('start'),
    )

    window._request_verification('rviz')

    assert calls == ['disconnect']
    assert window.verification_state == 'starting'
    assert window.pending_verification_mode == 'rviz'
    assert window.verification_restore_connection is True


def test_disconnected_verification_starts_without_camera_handoff(
    window, monkeypatch, tmp_path
):
    select_saved_profile(window)
    camera = tmp_path / 'video0'
    camera.touch()
    window.camera_combo.setCurrentText(str(camera))
    window.connection_state = 'disconnected'
    calls = []
    monkeypatch.setattr(
        window,
        '_start_verification_process',
        lambda: calls.append('start'),
    )

    window._request_verification('gazebo')

    assert calls == ['start']
    assert window.verification_restore_connection is False


def test_verification_cleanup_restores_only_previous_connection(
    window, monkeypatch
):
    from linkerhand_calibration import calibration_gui

    calls = []
    monkeypatch.setattr(window, '_connect_perception', lambda: calls.append('connect'))
    monkeypatch.setattr(
        calibration_gui.QtCore.QTimer,
        'singleShot',
        lambda _delay, callback: callback(),
    )
    window.verification_state = 'running'
    window.verification_mode = 'rviz'
    window.verification_restore_connection = True

    window._finish_verification_cleanup('RViz')

    assert calls == ['connect']
    assert window.verification_state == 'idle'
    assert '正在恢复' in window.verification_feedback_label.text()

    calls.clear()
    window.verification_state = 'running'
    window.verification_mode = 'gazebo'
    window.verification_restore_connection = False
    window._finish_verification_cleanup('Gazebo')

    assert calls == []
    assert '验证已结束' in window.verification_feedback_label.text()


def test_verification_qprocess_can_be_stopped_cleanly(
    window, monkeypatch, tmp_path, qt_app
):
    import shutil
    from linkerhand_calibration import calibration_gui

    select_saved_profile(window)
    camera = tmp_path / 'video0'
    camera.touch()
    window.camera_combo.setCurrentText(str(camera))
    window.connection_state = 'disconnected'
    setsid = shutil.which('setsid') or 'setsid'
    monkeypatch.setattr(
        calibration_gui,
        'build_verification_command',
        lambda _settings: (setsid, ('/bin/sleep', '30')),
    )

    window._request_verification('rviz')
    deadline = time.monotonic() + 2.0
    while (
        window.verification_state != 'running'
        and time.monotonic() < deadline
    ):
        qt_app.processEvents()
        time.sleep(0.01)

    assert window.verification_state == 'running'
    assert window.verification_process is not None
    assert window.rviz_button.text() == '停止 RViz 验证'

    window._stop_verification()
    deadline = time.monotonic() + 3.0
    while window.verification_state != 'idle' and time.monotonic() < deadline:
        qt_app.processEvents()
        time.sleep(0.01)

    assert window.verification_state == 'idle'
    assert window.verification_process is None
    assert '验证已结束' in window.verification_feedback_label.text()
