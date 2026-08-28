"""Linker Hand 个人标定 Qt 上位机。"""

from collections import deque
import math
import os
from pathlib import Path
import signal
import sys
import time

from ament_index_python.packages import get_package_share_directory
from hand_pose_msgs.msg import HandPose
from PySide2 import QtCore, QtGui, QtWidgets
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image

from linkerhand_calibration.calibration import (
    POSE_FIST,
    POSE_OPEN,
    POSE_THUMB_IN,
    POSE_THUMB_OUT,
    attach_profile_metadata,
    build_personal_calibration,
    build_profile_metadata,
    calibration_endpoints,
    extract_profile_metadata,
    load_parameters,
    validate_calibration_ranges,
    write_calibration,
)
from linkerhand_calibration.gui_support import (
    PerceptionSettings,
    VerificationSettings,
    build_perception_command,
    build_verification_command,
    calibration_topics,
    list_video_devices,
)
from linkerhand_calibration.profile_store import (
    DEFAULT_PROFILE_DIRECTORY,
    load_profile,
    scan_profiles,
    unique_profile_path,
    validate_profile_name,
)
from linkerhand_calibration.sampling import (
    FAIL_INSUFFICIENT_SAMPLES,
    FAIL_LOW_VALID_RATIO,
    FAIL_NO_MESSAGES,
    FAIL_UNSTABLE,
    INVALID_HAND_CLIPPED,
    INVALID_INCOMPLETE_ANGLES,
    INVALID_LOW_CONFIDENCE,
    INVALID_NO_HAND,
    INVALID_WRONG_HAND,
    PoseSampleCollector,
    extract_valid_sample,
)


TRANSLATIONS = {
    'zh': {
        'window_title': 'Linker Hand 个人标定',
        'app_title': 'Linker Hand 个人标定',
        'model': '型号',
        'side': '手侧',
        'left': '左手',
        'right': '右手',
        'camera': '摄像头',
        'refresh': '刷新设备',
        'connect': '连接',
        'disconnect': '断开',
        'connecting': '正在连接',
        'disconnecting': '正在断开',
        'advanced': '高级设置',
        'width': '宽度',
        'height': '高度',
        'camera_fps': '摄像头 FPS',
        'processing_fps': '识别 FPS',
        'mirror': '镜像预览',
        'sample_duration': '采样时长',
        'confidence_threshold': '置信度阈值',
        'minimum_samples': '最少有效帧',
        'minimum_valid_ratio': '最低有效率',
        'maximum_spread': '最大波动',
        'minimum_range': '最小活动范围',
        'preview_wait': '选择设备后点击“连接”',
        'preview_starting': '正在等待 MediaPipe 画面…',
        'preview_stopped': '感知管线已断开',
        'status': '检测状态',
        'connection': '连接',
        'target_hand': '目标手',
        'recognized_hand': '识别结果',
        'detection': '检测',
        'confidence': '置信度',
        'image_fps': '画面 FPS',
        'not_connected': '未连接',
        'waiting_image': '等待画面',
        'connected': '已连接',
        'unknown': '未知',
        'no_hand': '未检测到目标手',
        'stabilizing': '正在稳定',
        'stable': '稳定',
        'pose_group': '标定姿态',
        'pose_open': '张开手掌',
        'pose_fist': '自然握拳',
        'pose_thumb_in': '拇指收拢',
        'pose_thumb_out': '拇指展开',
        'pose_open_help': '自然张开手掌并伸直五指。',
        'pose_fist_help': '自然握拳，保持手腕稳定。',
        'pose_thumb_in_help': '保持四指伸直，将拇指收拢到掌侧。',
        'pose_thumb_out_help': '保持四指伸直，将拇指完全展开。',
        'not_sampled': '未采样',
        'sampling': '正在采样',
        'sample_success': '采样成功',
        'sample_failed': '采样失败',
        'ready_to_sample': '检测稳定，可以开始采样。',
        'wait_stable': '请将目标手完整放入画面，并等待检测稳定。',
        'ready_no_hand': '未检测到目标手，无法开始采样。',
        'ready_wrong_hand': '当前识别手侧与目标手不符。',
        'ready_low_confidence': '当前识别置信度不足，请调整光线或手掌位置。',
        'ready_hand_clipped': '请将整只手放入画面，避免关键点贴近或超出边缘。',
        'ready_incomplete_angles': '当前部分关节角缺失，请调整手掌朝向。',
        'sampling_hold': '正在采样，请保持当前手势不动。',
        'sample_cancelled': '已取消本次采样。',
        'sample_cancelled_kept': '已取消本次重采，原有成功结果保持不变。',
        'start_sample': '开始采样',
        'resample': '重新采样',
        'cancel_sample': '取消采样',
        'generate': '生成配置',
        'update_profile': '更新当前配置',
        'save_as': '另存为新配置',
        'profile_group': '个人配置',
        'profile': '配置档案',
        'new_profile': '新建个人配置',
        'profile_name': '配置名称',
        'profile_path': '文件路径',
        'import_profile': '导入 YAML',
        'result_group': '关节范围结果',
        'result_joint': '驱动关节',
        'result_min_pose': '最小姿态',
        'result_min': '最小角',
        'result_max_pose': '最大姿态',
        'result_max': '最大角',
        'result_range': '范围',
        'result_status': '检查',
        'result_pending': '等待四姿态采样完成',
        'result_ok': '通过',
        'result_invalid': '范围过小/方向异常',
        'profile_ready': '四姿态和全部关节范围检查通过，可以生成个人配置。',
        'profile_incomplete': '完成四个姿态采样后才能生成个人配置。',
        'profile_invalid': '联合范围检查未通过，请根据结果表重新采样对应姿态。',
        'profile_saved_title': '配置已保存',
        'profile_saved': '个人配置已保存：\n{path}',
        'profile_updated': '个人配置已更新：\n{path}',
        'profile_save_failed_title': '保存失败',
        'profile_save_failed': '无法生成个人配置：\n{reason}',
        'profile_overwrite_title': '确认更新配置',
        'profile_overwrite': '将使用本次四姿态采样结果更新：\n{path}\n\n确定继续吗？',
        'profile_save_as_title': '另存为个人配置',
        'profile_save_filter': 'ROS 2 参数 YAML (*.yaml)',
        'profile_import_title': '导入个人配置',
        'profile_import_filter': 'ROS 2 参数 YAML (*.yaml *.yml)',
        'profile_import_failed_title': '导入失败',
        'profile_import_failed': '无法导入该配置：\n{reason}',
        'profile_mismatch': '该配置属于 {model}/{side}，当前选择为 {current_model}/{current_side}。',
        'profile_external': '外部',
        'profile_legacy': '旧格式',
        'profile_name_invalid': '配置名称无效：{reason}',
        'profile_path_default': '保存后将在这里显示完整路径',
        'profile_read_only': '项目默认模板为只读，请使用“另存为新配置”。',
        'verify_rviz': 'RViz 验证',
        'verify_gazebo': 'Gazebo 验证',
        'stop_rviz': '停止 RViz 验证',
        'stop_gazebo': '停止 Gazebo 验证',
        'verification_starting': '正在释放摄像头并启动 {mode} 验证…',
        'verification_running': '{mode} 验证正在运行。点击“停止验证”可完整退出摄像头和仿真进程。',
        'verification_stopping': '正在停止 {mode} 验证…',
        'verification_finished': '{mode} 验证已结束。',
        'verification_finished_reconnecting': '{mode} 验证已结束，正在恢复标定摄像头连接…',
        'verification_failed_title': '验证启动失败',
        'verification_failed': '无法启动 {mode} 验证：\n{reason}',
        'verification_profile_required': '请先选择或生成一份已保存的个人配置。',
        'verification_unsaved': '当前采样结果尚未保存，请先更新或另存配置。',
        'verification_ready': '当前个人配置已就绪，可以启动 RViz 或 Gazebo 验证。',
        'device_missing_title': '摄像头不可用',
        'device_missing': '摄像头设备不存在：\n{device}',
        'start_failed_title': '连接失败',
        'start_failed': '无法启动摄像头与 MediaPipe：\n{reason}',
        'process_exited': '感知管线异常退出，退出码 {code}',
        'template_failed_title': '标定模板不可用',
        'template_failed': '无法加载 {model}/{side} 标定模板：\n{reason}',
        'sample_success_detail': '采样成功：{valid}/{total} 有效帧（{ratio:.0%}），最大波动 {spread:.1f}°。',
        'sample_no_messages': '采样失败：没有收到姿态数据，请检查摄像头与 MediaPipe。',
        'sample_few_frames': '采样失败：只有 {valid}/{total} 个有效帧，至少需要 {required} 帧。{hint}',
        'sample_low_ratio': '采样失败：有效帧率为 {ratio:.0%}，至少需要 {required:.0%}。{hint}',
        'sample_unstable': '采样失败：{joint} 波动 {spread:.1f}°，允许上限为 {limit:.1f}°。',
        'sample_old_kept': ' 原有成功结果已保留。',
        'invalid_no_hand': '采样期间经常未检测到目标手。',
        'invalid_wrong_hand': '采样期间识别到的手侧不匹配。',
        'invalid_low_confidence': '采样期间识别置信度偏低。',
        'invalid_hand_clipped': '采样期间手部关键点超出画面，请让整只手可见。',
        'invalid_incomplete_angles': '采样期间部分关节角缺失。',
        'unsaved_title': '未保存的标定结果',
        'unsaved_close': '当前已有未保存的姿态采样结果，确定关闭并放弃吗？',
        'unsaved_profile': '切换型号或手侧会放弃当前姿态采样结果，确定继续吗？',
    },
    'en': {
        'window_title': 'Linker Hand Personal Calibration',
        'app_title': 'Linker Hand Personal Calibration',
        'model': 'Model',
        'side': 'Hand',
        'left': 'Left',
        'right': 'Right',
        'camera': 'Camera',
        'refresh': 'Refresh',
        'connect': 'Connect',
        'disconnect': 'Disconnect',
        'connecting': 'Connecting',
        'disconnecting': 'Disconnecting',
        'advanced': 'Advanced settings',
        'width': 'Width',
        'height': 'Height',
        'camera_fps': 'Camera FPS',
        'processing_fps': 'Detection FPS',
        'mirror': 'Mirror preview',
        'sample_duration': 'Sample time',
        'confidence_threshold': 'Confidence',
        'minimum_samples': 'Minimum frames',
        'minimum_valid_ratio': 'Valid ratio',
        'maximum_spread': 'Max variation',
        'minimum_range': 'Minimum range',
        'preview_wait': 'Select a device and click Connect',
        'preview_starting': 'Waiting for MediaPipe video…',
        'preview_stopped': 'Perception pipeline disconnected',
        'status': 'Detection status',
        'connection': 'Connection',
        'target_hand': 'Target hand',
        'recognized_hand': 'Recognized',
        'detection': 'Detection',
        'confidence': 'Confidence',
        'image_fps': 'Video FPS',
        'not_connected': 'Disconnected',
        'waiting_image': 'Waiting for video',
        'connected': 'Connected',
        'unknown': 'Unknown',
        'no_hand': 'Target hand not detected',
        'stabilizing': 'Stabilizing',
        'stable': 'Stable',
        'pose_group': 'Calibration poses',
        'pose_open': 'Open hand',
        'pose_fist': 'Closed fist',
        'pose_thumb_in': 'Thumb adducted',
        'pose_thumb_out': 'Thumb abducted',
        'pose_open_help': 'Open your palm naturally and extend all fingers.',
        'pose_fist_help': 'Make a natural fist and keep your wrist steady.',
        'pose_thumb_in_help': 'Keep four fingers extended and move the thumb inward.',
        'pose_thumb_out_help': 'Keep four fingers extended and fully spread the thumb.',
        'not_sampled': 'Not sampled',
        'sampling': 'Sampling',
        'sample_success': 'Sampled',
        'sample_failed': 'Failed',
        'ready_to_sample': 'Detection is stable. Ready to sample.',
        'wait_stable': 'Keep the complete target hand in view and wait for stable detection.',
        'ready_no_hand': 'The target hand is not detected.',
        'ready_wrong_hand': 'The detected hand does not match the target side.',
        'ready_low_confidence': 'Detection confidence is too low. Adjust lighting or hand position.',
        'ready_hand_clipped': 'Keep the complete hand inside the frame and away from the edges.',
        'ready_incomplete_angles': 'Some joint angles are missing. Adjust the palm orientation.',
        'sampling_hold': 'Sampling. Hold the current pose still.',
        'sample_cancelled': 'The current sample was cancelled.',
        'sample_cancelled_kept': 'Resampling cancelled. The previous result was kept.',
        'start_sample': 'Start sampling',
        'resample': 'Resample',
        'cancel_sample': 'Cancel',
        'generate': 'Generate profile',
        'update_profile': 'Update profile',
        'save_as': 'Save as new',
        'profile_group': 'Personal profiles',
        'profile': 'Profile',
        'new_profile': 'New personal profile',
        'profile_name': 'Profile name',
        'profile_path': 'File path',
        'import_profile': 'Import YAML',
        'result_group': 'Joint range results',
        'result_joint': 'Driven joint',
        'result_min_pose': 'Minimum pose',
        'result_min': 'Minimum',
        'result_max_pose': 'Maximum pose',
        'result_max': 'Maximum',
        'result_range': 'Range',
        'result_status': 'Check',
        'result_pending': 'Waiting for all four pose samples',
        'result_ok': 'Pass',
        'result_invalid': 'Too small / reversed',
        'profile_ready': 'All poses and joint ranges passed. The profile can be generated.',
        'profile_incomplete': 'Complete all four pose samples before generating a profile.',
        'profile_invalid': 'The combined range check failed. Resample the indicated poses.',
        'profile_saved_title': 'Profile saved',
        'profile_saved': 'Personal profile saved:\n{path}',
        'profile_updated': 'Personal profile updated:\n{path}',
        'profile_save_failed_title': 'Save failed',
        'profile_save_failed': 'Could not generate the personal profile:\n{reason}',
        'profile_overwrite_title': 'Confirm profile update',
        'profile_overwrite': 'This sample set will update:\n{path}\n\nContinue?',
        'profile_save_as_title': 'Save personal profile as',
        'profile_save_filter': 'ROS 2 parameter YAML (*.yaml)',
        'profile_import_title': 'Import personal profile',
        'profile_import_filter': 'ROS 2 parameter YAML (*.yaml *.yml)',
        'profile_import_failed_title': 'Import failed',
        'profile_import_failed': 'Could not import this profile:\n{reason}',
        'profile_mismatch': 'This profile is for {model}/{side}; the current selection is {current_model}/{current_side}.',
        'profile_external': 'external',
        'profile_legacy': 'legacy',
        'profile_name_invalid': 'Invalid profile name: {reason}',
        'profile_path_default': 'The full path will appear here after saving',
        'profile_read_only': 'Repository default templates are read-only. Use Save as new.',
        'verify_rviz': 'Verify in RViz',
        'verify_gazebo': 'Verify in Gazebo',
        'stop_rviz': 'Stop RViz verification',
        'stop_gazebo': 'Stop Gazebo verification',
        'verification_starting': 'Releasing the camera and starting {mode} verification…',
        'verification_running': '{mode} verification is running. Click Stop verification to exit the camera and simulation processes cleanly.',
        'verification_stopping': 'Stopping {mode} verification…',
        'verification_finished': '{mode} verification finished.',
        'verification_finished_reconnecting': '{mode} verification finished. Reconnecting the calibration camera…',
        'verification_failed_title': 'Verification failed to start',
        'verification_failed': 'Could not start {mode} verification:\n{reason}',
        'verification_profile_required': 'Select or generate a saved personal profile first.',
        'verification_unsaved': 'The current samples are not saved. Update or save the profile first.',
        'verification_ready': 'The current personal profile is ready for RViz or Gazebo verification.',
        'device_missing_title': 'Camera unavailable',
        'device_missing': 'Camera device does not exist:\n{device}',
        'start_failed_title': 'Connection failed',
        'start_failed': 'Could not start the camera and MediaPipe:\n{reason}',
        'process_exited': 'Perception pipeline exited unexpectedly (code {code})',
        'template_failed_title': 'Calibration template unavailable',
        'template_failed': 'Could not load the {model}/{side} template:\n{reason}',
        'sample_success_detail': 'Sampled: {valid}/{total} valid frames ({ratio:.0%}), max variation {spread:.1f}°.',
        'sample_no_messages': 'Sampling failed: no pose data was received. Check the camera and MediaPipe.',
        'sample_few_frames': 'Sampling failed: {valid}/{total} valid frames; at least {required} are required. {hint}',
        'sample_low_ratio': 'Sampling failed: valid ratio was {ratio:.0%}; at least {required:.0%} is required. {hint}',
        'sample_unstable': 'Sampling failed: {joint} varied by {spread:.1f}°; the limit is {limit:.1f}°.',
        'sample_old_kept': ' The previous successful result was kept.',
        'invalid_no_hand': 'The target hand was often not detected during sampling.',
        'invalid_wrong_hand': 'The detected hand side did not match during sampling.',
        'invalid_low_confidence': 'Detection confidence was too low during sampling.',
        'invalid_hand_clipped': 'Part of the hand left the frame during sampling.',
        'invalid_incomplete_angles': 'Some joint angles were missing during sampling.',
        'unsaved_title': 'Unsaved calibration results',
        'unsaved_close': 'There are unsaved pose samples. Close and discard them?',
        'unsaved_profile': 'Changing the model or hand will discard current samples. Continue?',
    },
}


POSE_UI_KEYS = {
    POSE_OPEN: 'pose_open',
    POSE_FIST: 'pose_fist',
    POSE_THUMB_IN: 'pose_thumb_in',
    POSE_THUMB_OUT: 'pose_thumb_out',
}

JOINT_LABELS = {
    'thumb_cmc_yaw': ('拇指侧摆', 'Thumb abduction'),
    'thumb_cmc_pitch': ('拇指屈曲', 'Thumb flexion'),
    'thumb_mcp': ('拇指 MCP', 'Thumb MCP'),
    'thumb_dip': ('拇指 DIP', 'Thumb DIP'),
    'index_mcp_pitch': ('食指 MCP', 'Index MCP'),
    'index_pip': ('食指 PIP', 'Index PIP'),
    'middle_mcp_pitch': ('中指 MCP', 'Middle MCP'),
    'middle_pip': ('中指 PIP', 'Middle PIP'),
    'ring_mcp_pitch': ('无名指 MCP', 'Ring MCP'),
    'ring_pip': ('无名指 PIP', 'Ring PIP'),
    'pinky_mcp_pitch': ('小拇指 MCP', 'Pinky MCP'),
    'pinky_pip': ('小拇指 PIP', 'Pinky PIP'),
}


class VideoPreview(QtWidgets.QLabel):
    """保持图像比例的预览控件。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._image = None
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.setMinimumSize(640, 480)
        self.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding,
            QtWidgets.QSizePolicy.Expanding,
        )
        self.setObjectName('videoPreview')

    def set_image(self, image):
        self._image = image
        self.setText('')
        self._render_image()

    def clear_image(self, text):
        self._image = None
        self.setPixmap(QtGui.QPixmap())
        self.setText(text)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._render_image()

    def _render_image(self):
        if self._image is None or self._image.isNull():
            return
        pixmap = QtGui.QPixmap.fromImage(self._image)
        self.setPixmap(pixmap.scaled(
            self.size(),
            QtCore.Qt.KeepAspectRatio,
            QtCore.Qt.SmoothTransformation,
        ))


def qimage_from_ros_image(message):
    """直接把常见 ROS 图像编码转为 QImage，避免在 Qt 进程加载 cv2。"""
    formats = {
        'bgr8': QtGui.QImage.Format_BGR888,
        'rgb8': QtGui.QImage.Format_RGB888,
        'rgba8': QtGui.QImage.Format_RGBA8888,
        'bgra8': QtGui.QImage.Format_ARGB32,
        'mono8': QtGui.QImage.Format_Grayscale8,
    }
    encoding = str(message.encoding).strip().lower()
    image_format = formats.get(encoding)
    if image_format is None:
        raise ValueError(f'Qt 预览不支持 ROS 图像编码：{message.encoding}')

    width = int(message.width)
    height = int(message.height)
    step = int(message.step)
    if width <= 0 or height <= 0 or step <= 0:
        raise ValueError(
            f'ROS 图像尺寸无效：{width}x{height}, step={step}'
        )
    data = bytes(message.data)
    if len(data) < step * height:
        raise ValueError(
            f'ROS 图像数据不足：需要 {step * height} 字节，实际 {len(data)} 字节'
        )
    return QtGui.QImage(
        data,
        width,
        height,
        step,
        image_format,
    ).copy()


class RosSubscriberThread(QtCore.QThread):
    """在后台线程中执行 ROS 回调，避免阻塞 Qt 主线程。"""

    image_received = QtCore.Signal(object)
    pose_received = QtCore.Signal(object)
    failed = QtCore.Signal(str)

    def __init__(self, side, parent=None):
        super().__init__(parent)
        self.side = side
        self._running = True
        self._node = None

    def stop(self):
        self._running = False

    def run(self):
        topics = calibration_topics(self.side)
        node = None
        try:
            node = rclpy.create_node(
                f'linkerhand_calibration_gui_{self.side}_{os.getpid()}'
            )
            self._node = node

            def image_callback(message):
                try:
                    image = qimage_from_ros_image(message)
                except ValueError as error:
                    self.failed.emit(f'调试图像转换失败：{error}')
                    return
                self.image_received.emit(image)

            def pose_callback(message):
                landmarks_visible = (
                    len(message.landmarks) >= 21
                    and all(
                        -0.02 <= float(point.x) <= 1.02
                        and -0.02 <= float(point.y) <= 1.02
                        for point in message.landmarks
                    )
                )
                self.pose_received.emit({
                    'detected': bool(message.detected),
                    'handedness': str(message.handedness).lower(),
                    'confidence': float(message.confidence),
                    'processing_time_ms': float(message.processing_time_ms),
                    'joint_names': tuple(message.joint_names),
                    'joint_angles': tuple(message.joint_angles),
                    'landmarks_visible': landmarks_visible,
                })

            node.create_subscription(
                Image,
                topics['debug_image'],
                image_callback,
                qos_profile_sensor_data,
            )
            node.create_subscription(
                HandPose,
                topics['pose'],
                pose_callback,
                10,
            )
            while self._running and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
        except Exception as error:  # Qt 线程必须把异常送回主线程。
            self.failed.emit(str(error))
        finally:
            self._node = None
            if node is not None:
                node.destroy_node()


class CalibrationWindow(QtWidgets.QMainWindow):
    """个人标定上位机主窗口。"""

    def __init__(self):
        super().__init__()
        self.qt_settings = QtCore.QSettings(
            'linkerhand_gesture_control', 'calibration_gui'
        )
        saved_language = str(
            self.qt_settings.value('language', 'zh')
        ).lower()
        self.language = saved_language if saved_language in TRANSLATIONS else 'zh'
        self.connection_state = 'disconnected'
        self.perception_process = None
        self.verification_process = None
        self.verification_state = 'idle'
        self.verification_mode = None
        self.pending_verification_mode = None
        self.verification_restore_connection = False
        self.intentional_verification_stop = False
        self.verification_feedback_key = 'verification_profile_required'
        self.verification_feedback_kwargs = {}
        self.verification_feedback_color = '#6c756f'
        self.ros_thread = None
        self.intentional_process_stop = False
        self.shutting_down = False
        self.last_image_time = None
        self.last_pose_time = None
        self.latest_pose = None
        self.latest_pose_invalid_reason = INVALID_NO_HAND
        self.valid_detection_started = None
        self.image_times = deque(maxlen=120)
        self.template_document = None
        self.template_parameters = None
        self.pose_order = (
            POSE_OPEN,
            POSE_FIST,
            POSE_THUMB_IN,
            POSE_THUMB_OUT,
        )
        self.pose_results = {}
        self.samples_dirty = False
        self.pose_attempt_states = {
            pose: 'not_sampled' for pose in self.pose_order
        }
        self.sample_collector = None
        self.sampling_pose = None
        self.sampling_started_at = None
        self.sample_feedback_key = 'wait_stable'
        self.sample_feedback_kwargs = {}
        self.sample_feedback_color = '#6c756f'
        self.sample_feedback_keep_old = False
        self.profile_change_guard = False
        self.profile_combo_guard = False
        self.available_profiles = ()
        self.profile_scan_errors = ()
        self.external_profile_paths = self._restore_external_profile_paths()
        self.current_profile = None
        self.last_saved_profile_path = None

        self._build_ui()
        self._apply_style()
        self._restore_window_state()
        self._refresh_devices()
        self._load_template_for_selection()
        self._retranslate_ui()

        self.status_timer = QtCore.QTimer(self)
        self.status_timer.setInterval(100)
        self.status_timer.timeout.connect(self._refresh_live_status)
        self.status_timer.start()

        self.sample_timer = QtCore.QTimer(self)
        self.sample_timer.setInterval(50)
        self.sample_timer.timeout.connect(self._update_sampling_progress)

    def _tr(self, key):
        return TRANSLATIONS[self.language][key]

    def _build_ui(self):
        root = QtWidgets.QWidget(self)
        root_layout = QtWidgets.QVBoxLayout(root)
        root_layout.setContentsMargins(18, 14, 18, 14)
        root_layout.setSpacing(12)
        self.setCentralWidget(root)

        header = QtWidgets.QHBoxLayout()
        self.title_label = QtWidgets.QLabel()
        self.title_label.setObjectName('titleLabel')
        header.addWidget(self.title_label)
        header.addStretch()
        self.language_combo = QtWidgets.QComboBox()
        self.language_combo.setFixedWidth(112)
        self.language_combo.addItem('中文', 'zh')
        self.language_combo.addItem('English', 'en')
        language_index = self.language_combo.findData(self.language)
        self.language_combo.setCurrentIndex(max(language_index, 0))
        self.language_combo.currentIndexChanged.connect(
            self._language_changed
        )
        header.addWidget(self.language_combo)
        root_layout.addLayout(header)

        connection_bar = QtWidgets.QFrame()
        connection_bar.setObjectName('connectionBar')
        connection_layout = QtWidgets.QGridLayout(connection_bar)
        connection_layout.setContentsMargins(12, 10, 12, 10)
        connection_layout.setHorizontalSpacing(9)
        connection_layout.setVerticalSpacing(8)

        self.model_label = QtWidgets.QLabel()
        self.model_combo = QtWidgets.QComboBox()
        self.model_combo.addItem('L30', 'l30')
        self.model_combo.addItem('O6', 'o6')
        self.model_combo.setMinimumWidth(92)
        self.model_combo.currentIndexChanged.connect(self._profile_changed)
        self.side_label = QtWidgets.QLabel()
        self.side_combo = QtWidgets.QComboBox()
        self.side_combo.addItem('', 'left')
        self.side_combo.addItem('', 'right')
        self.side_combo.setMinimumWidth(92)
        self.side_combo.currentIndexChanged.connect(self._profile_changed)
        self.camera_label = QtWidgets.QLabel()
        self.camera_combo = QtWidgets.QComboBox()
        self.camera_combo.setEditable(True)
        self.camera_combo.setMinimumWidth(190)
        self.refresh_button = QtWidgets.QPushButton()
        self.refresh_button.setIcon(
            self.style().standardIcon(QtWidgets.QStyle.SP_BrowserReload)
        )
        self.refresh_button.clicked.connect(self._refresh_devices)
        self.connect_button = QtWidgets.QPushButton()
        self.connect_button.setObjectName('primaryButton')
        self.connect_button.setMinimumWidth(112)
        self.connect_button.clicked.connect(self._toggle_connection)

        connection_layout.addWidget(self.model_label, 0, 0)
        connection_layout.addWidget(self.model_combo, 0, 1)
        connection_layout.addWidget(self.side_label, 0, 2)
        connection_layout.addWidget(self.side_combo, 0, 3)
        connection_layout.addWidget(self.camera_label, 0, 4)
        connection_layout.addWidget(self.camera_combo, 0, 5)
        connection_layout.setColumnStretch(5, 1)
        connection_layout.addWidget(self.refresh_button, 0, 6)
        connection_layout.addWidget(self.connect_button, 0, 7)

        self.advanced_button = QtWidgets.QToolButton()
        self.advanced_button.setCheckable(True)
        self.advanced_button.setChecked(False)
        self.advanced_button.setToolButtonStyle(
            QtCore.Qt.ToolButtonTextBesideIcon
        )
        self.advanced_button.setArrowType(QtCore.Qt.RightArrow)
        self.advanced_button.toggled.connect(self._toggle_advanced)
        connection_layout.addWidget(self.advanced_button, 1, 0, 1, 2)

        self.advanced_widget = QtWidgets.QWidget()
        advanced_layout = QtWidgets.QGridLayout(self.advanced_widget)
        advanced_layout.setContentsMargins(0, 0, 0, 0)
        advanced_layout.setHorizontalSpacing(8)
        advanced_layout.setVerticalSpacing(6)
        self.width_label = QtWidgets.QLabel()
        self.width_spin = QtWidgets.QSpinBox()
        self.width_spin.setRange(160, 3840)
        self.width_spin.setValue(640)
        self.height_label = QtWidgets.QLabel()
        self.height_spin = QtWidgets.QSpinBox()
        self.height_spin.setRange(120, 2160)
        self.height_spin.setValue(480)
        self.camera_fps_label = QtWidgets.QLabel()
        self.camera_fps_spin = QtWidgets.QDoubleSpinBox()
        self.camera_fps_spin.setRange(1.0, 120.0)
        self.camera_fps_spin.setValue(30.0)
        self.camera_fps_spin.setDecimals(1)
        self.processing_fps_label = QtWidgets.QLabel()
        self.processing_fps_spin = QtWidgets.QDoubleSpinBox()
        self.processing_fps_spin.setRange(1.0, 60.0)
        self.processing_fps_spin.setValue(15.0)
        self.processing_fps_spin.setDecimals(1)
        self.mirror_checkbox = QtWidgets.QCheckBox()
        self.mirror_checkbox.setChecked(True)
        self.sample_duration_label = QtWidgets.QLabel()
        self.sample_duration_spin = QtWidgets.QDoubleSpinBox()
        self.sample_duration_spin.setRange(1.0, 5.0)
        self.sample_duration_spin.setValue(2.0)
        self.sample_duration_spin.setSingleStep(0.5)
        self.sample_duration_spin.setDecimals(1)
        self.sample_duration_spin.setSuffix(' s')
        self.confidence_threshold_label = QtWidgets.QLabel()
        self.confidence_threshold_spin = QtWidgets.QDoubleSpinBox()
        self.confidence_threshold_spin.setRange(0.0, 1.0)
        self.confidence_threshold_spin.setValue(0.5)
        self.confidence_threshold_spin.setSingleStep(0.05)
        self.confidence_threshold_spin.setDecimals(2)
        self.minimum_samples_label = QtWidgets.QLabel()
        self.minimum_samples_spin = QtWidgets.QSpinBox()
        self.minimum_samples_spin.setRange(1, 120)
        self.minimum_samples_spin.setValue(15)
        self.minimum_valid_ratio_label = QtWidgets.QLabel()
        self.minimum_valid_ratio_spin = QtWidgets.QDoubleSpinBox()
        self.minimum_valid_ratio_spin.setRange(0.1, 1.0)
        self.minimum_valid_ratio_spin.setValue(0.70)
        self.minimum_valid_ratio_spin.setSingleStep(0.05)
        self.minimum_valid_ratio_spin.setDecimals(2)
        self.maximum_spread_label = QtWidgets.QLabel()
        self.maximum_spread_spin = QtWidgets.QDoubleSpinBox()
        self.maximum_spread_spin.setRange(1.0, 30.0)
        self.maximum_spread_spin.setValue(8.0)
        self.maximum_spread_spin.setSingleStep(0.5)
        self.maximum_spread_spin.setDecimals(1)
        self.maximum_spread_spin.setSuffix('°')
        self.minimum_range_label = QtWidgets.QLabel()
        self.minimum_range_spin = QtWidgets.QDoubleSpinBox()
        self.minimum_range_spin.setRange(1.0, 45.0)
        self.minimum_range_spin.setValue(5.0)
        self.minimum_range_spin.setSingleStep(0.5)
        self.minimum_range_spin.setDecimals(1)
        self.minimum_range_spin.setSuffix('°')
        self.minimum_range_spin.valueChanged.connect(
            self._refresh_profile_results
        )
        first_row = (
            (self.width_label, self.width_spin),
            (self.height_label, self.height_spin),
            (self.camera_fps_label, self.camera_fps_spin),
            (self.processing_fps_label, self.processing_fps_spin),
        )
        second_row = (
            (self.sample_duration_label, self.sample_duration_spin),
            (self.confidence_threshold_label, self.confidence_threshold_spin),
            (self.minimum_samples_label, self.minimum_samples_spin),
            (self.minimum_valid_ratio_label, self.minimum_valid_ratio_spin),
            (self.maximum_spread_label, self.maximum_spread_spin),
            (self.minimum_range_label, self.minimum_range_spin),
        )
        for row, pairs in enumerate((first_row, second_row)):
            for column, (label, field) in enumerate(pairs):
                advanced_layout.addWidget(label, row, column * 2)
                advanced_layout.addWidget(field, row, column * 2 + 1)
        advanced_layout.addWidget(
            self.mirror_checkbox, 0, len(first_row) * 2, 1, 2
        )
        advanced_layout.setColumnStretch(len(second_row) * 2, 1)
        self.advanced_widget.setVisible(False)
        connection_layout.addWidget(self.advanced_widget, 1, 2, 1, 6)
        root_layout.addWidget(connection_bar)

        main_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        main_splitter.setChildrenCollapsible(False)
        self.video_preview = VideoPreview()
        main_splitter.addWidget(self.video_preview)

        side_panel = QtWidgets.QWidget()
        side_layout = QtWidgets.QVBoxLayout(side_panel)
        side_layout.setContentsMargins(12, 0, 0, 0)
        side_layout.setSpacing(12)

        self.status_group = QtWidgets.QGroupBox()
        status_layout = QtWidgets.QGridLayout(self.status_group)
        status_layout.setContentsMargins(8, 8, 8, 8)
        status_layout.setHorizontalSpacing(8)
        status_layout.setVerticalSpacing(2)
        status_layout.setColumnStretch(1, 1)
        self.status_name_labels = {}
        self.status_value_labels = {}
        for row, key in enumerate((
            'connection',
            'target_hand',
            'recognized_hand',
            'detection',
            'confidence',
            'image_fps',
        )):
            name_label = QtWidgets.QLabel()
            value_label = QtWidgets.QLabel('—')
            value_label.setObjectName('statusValue')
            self.status_name_labels[key] = name_label
            self.status_value_labels[key] = value_label
            status_layout.addWidget(name_label, row, 0)
            status_layout.addWidget(value_label, row, 1)
        self.status_group.setMaximumHeight(170)
        side_layout.addWidget(self.status_group)

        self.pose_group = QtWidgets.QGroupBox()
        pose_layout = QtWidgets.QVBoxLayout(self.pose_group)
        self.pose_list = QtWidgets.QListWidget()
        self.pose_list.setMinimumHeight(122)
        self.pose_keys = tuple(POSE_UI_KEYS[pose] for pose in self.pose_order)
        for pose, key in zip(self.pose_order, self.pose_keys):
            item = QtWidgets.QListWidgetItem()
            item.setData(QtCore.Qt.UserRole, key)
            item.setData(QtCore.Qt.UserRole + 1, pose)
            self.pose_list.addItem(item)
        self.pose_list.setCurrentRow(0)
        self.pose_list.currentRowChanged.connect(self._pose_selected)
        pose_layout.addWidget(self.pose_list)
        self.pose_help_label = QtWidgets.QLabel()
        self.pose_help_label.setWordWrap(True)
        self.pose_help_label.setMinimumHeight(36)
        pose_layout.addWidget(self.pose_help_label)
        self.sample_feedback_label = QtWidgets.QLabel()
        self.sample_feedback_label.setWordWrap(True)
        self.sample_feedback_label.setMinimumHeight(36)
        self.sample_feedback_label.setObjectName('sampleFeedback')
        pose_layout.addWidget(self.sample_feedback_label)
        self.sample_progress = QtWidgets.QProgressBar()
        self.sample_progress.setRange(0, 1000)
        self.sample_progress.setValue(0)
        self.sample_progress.setTextVisible(True)
        self.sample_progress.setVisible(False)
        pose_layout.addWidget(self.sample_progress)
        sample_buttons = QtWidgets.QHBoxLayout()
        self.sample_button = QtWidgets.QPushButton()
        self.sample_button.setEnabled(False)
        self.sample_button.clicked.connect(self._start_sampling)
        self.cancel_sample_button = QtWidgets.QPushButton()
        self.cancel_sample_button.setEnabled(False)
        self.cancel_sample_button.clicked.connect(self._cancel_sampling)
        sample_buttons.addWidget(self.sample_button)
        sample_buttons.addWidget(self.cancel_sample_button)
        pose_layout.addLayout(sample_buttons)
        self.workflow_tabs = QtWidgets.QTabWidget()
        self.workflow_tabs.addTab(self.pose_group, '')

        self.profile_panel = QtWidgets.QWidget()
        profile_panel_layout = QtWidgets.QVBoxLayout(self.profile_panel)
        profile_panel_layout.setContentsMargins(4, 8, 4, 4)
        profile_panel_layout.setSpacing(8)

        profile_form = QtWidgets.QGridLayout()
        profile_form.setHorizontalSpacing(8)
        profile_form.setVerticalSpacing(6)
        self.profile_label = QtWidgets.QLabel()
        self.profile_combo = QtWidgets.QComboBox()
        self.profile_combo.currentIndexChanged.connect(
            self._profile_selection_changed
        )
        self.import_profile_button = QtWidgets.QPushButton()
        self.import_profile_button.setIcon(
            self.style().standardIcon(QtWidgets.QStyle.SP_DialogOpenButton)
        )
        self.import_profile_button.clicked.connect(self._import_profile)
        profile_form.addWidget(self.profile_label, 0, 0)
        profile_form.addWidget(self.profile_combo, 0, 1)
        profile_form.addWidget(self.import_profile_button, 0, 2)

        self.profile_name_label = QtWidgets.QLabel()
        self.profile_name_edit = QtWidgets.QLineEdit()
        self.profile_name_edit.setMaxLength(64)
        self.profile_name_edit.textChanged.connect(
            self._update_profile_controls
        )
        profile_form.addWidget(self.profile_name_label, 1, 0)
        profile_form.addWidget(self.profile_name_edit, 1, 1, 1, 2)

        self.profile_path_name_label = QtWidgets.QLabel()
        self.profile_path_edit = QtWidgets.QLineEdit()
        self.profile_path_edit.setReadOnly(True)
        self.profile_path_edit.setCursorPosition(0)
        profile_form.addWidget(self.profile_path_name_label, 2, 0)
        profile_form.addWidget(self.profile_path_edit, 2, 1, 1, 2)
        profile_form.setColumnStretch(1, 1)
        profile_panel_layout.addLayout(profile_form)

        self.profile_feedback_label = QtWidgets.QLabel()
        self.profile_feedback_label.setWordWrap(True)
        self.profile_feedback_label.setObjectName('profileFeedback')
        profile_panel_layout.addWidget(self.profile_feedback_label)

        self.result_table = QtWidgets.QTableWidget()
        self.result_table.setColumnCount(5)
        self.result_table.setEditTriggers(
            QtWidgets.QAbstractItemView.NoEditTriggers
        )
        self.result_table.setSelectionMode(
            QtWidgets.QAbstractItemView.NoSelection
        )
        self.result_table.setAlternatingRowColors(True)
        self.result_table.verticalHeader().setVisible(False)
        self.result_table.horizontalHeader().setStretchLastSection(True)
        self.result_table.horizontalHeader().setSectionResizeMode(
            0, QtWidgets.QHeaderView.Stretch
        )
        for column in (1, 2, 3):
            self.result_table.horizontalHeader().setSectionResizeMode(
                column, QtWidgets.QHeaderView.ResizeToContents
            )
        self.result_table.horizontalHeader().setSectionResizeMode(
            4, QtWidgets.QHeaderView.ResizeToContents
        )
        profile_panel_layout.addWidget(self.result_table, 1)
        self.workflow_tabs.addTab(self.profile_panel, '')
        side_layout.addWidget(self.workflow_tabs, 1)
        main_splitter.addWidget(side_panel)
        main_splitter.setStretchFactor(0, 2)
        main_splitter.setStretchFactor(1, 1)
        main_splitter.setSizes([760, 360])
        root_layout.addWidget(main_splitter, 1)

        action_bar = QtWidgets.QHBoxLayout()
        action_bar.addStretch()
        self.generate_button = QtWidgets.QPushButton()
        self.generate_button.setEnabled(False)
        self.generate_button.setObjectName('primaryButton')
        self.generate_button.clicked.connect(self._save_profile)
        self.save_as_button = QtWidgets.QPushButton()
        self.save_as_button.setEnabled(False)
        self.save_as_button.clicked.connect(self._save_profile_as)
        self.rviz_button = QtWidgets.QPushButton()
        self.rviz_button.setEnabled(False)
        self.rviz_button.clicked.connect(
            lambda: self._toggle_verification('rviz')
        )
        self.gazebo_button = QtWidgets.QPushButton()
        self.gazebo_button.setEnabled(False)
        self.gazebo_button.clicked.connect(
            lambda: self._toggle_verification('gazebo')
        )
        self.verification_feedback_label = QtWidgets.QLabel()
        self.verification_feedback_label.setWordWrap(True)
        self.verification_feedback_label.setObjectName('verificationFeedback')
        self.verification_feedback_label.setMinimumWidth(300)
        action_bar.addWidget(self.verification_feedback_label, 1)
        action_bar.addWidget(self.generate_button)
        action_bar.addWidget(self.save_as_button)
        action_bar.addWidget(self.rviz_button)
        action_bar.addWidget(self.gazebo_button)
        root_layout.addLayout(action_bar)

    def _apply_style(self):
        self.setStyleSheet("""
            QMainWindow, QWidget {
                background: #f4f5f2;
                color: #202522;
                font-size: 14px;
            }
            QLabel#titleLabel {
                font-size: 24px;
                font-weight: 700;
                color: #202522;
            }
            QFrame#connectionBar {
                background: #ffffff;
                border: 1px solid #c9cfca;
                border-radius: 6px;
            }
            QLabel#videoPreview {
                background: #151817;
                color: #d9dfda;
                border: 1px solid #343936;
                border-radius: 4px;
                font-size: 16px;
            }
            QGroupBox {
                background: #ffffff;
                border: 1px solid #c9cfca;
                border-radius: 6px;
                margin-top: 12px;
                padding-top: 10px;
                font-weight: 600;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
                background: #f4f5f2;
            }
            QComboBox, QSpinBox, QDoubleSpinBox, QPushButton, QToolButton {
                min-height: 30px;
            }
            QComboBox, QSpinBox, QDoubleSpinBox {
                background: #ffffff;
                border: 1px solid #aeb6b0;
                border-radius: 4px;
                padding: 0 7px;
            }
            QPushButton, QToolButton {
                background: #ffffff;
                border: 1px solid #aeb6b0;
                border-radius: 4px;
                padding: 0 12px;
            }
            QPushButton:hover, QToolButton:hover {
                background: #edf1ee;
                border-color: #78847c;
            }
            QPushButton#primaryButton {
                background: #166b4f;
                color: #ffffff;
                border-color: #166b4f;
                font-weight: 600;
            }
            QPushButton#primaryButton:hover {
                background: #125c43;
            }
            QPushButton:disabled {
                color: #8d9690;
                background: #e9ece9;
                border-color: #d2d7d3;
            }
            QListWidget {
                background: #ffffff;
                border: 1px solid #d1d6d2;
                border-radius: 4px;
                outline: none;
            }
            QListWidget::item {
                min-height: 28px;
                padding: 0 8px;
            }
            QListWidget::item:selected {
                background: #dceee6;
                color: #164d3b;
            }
            QLabel#statusValue {
                font-weight: 600;
            }
            QLabel#sampleFeedback {
                background: #f0f2ef;
                border: 1px solid #d8ddd9;
                border-radius: 4px;
                padding: 7px;
            }
            QLabel#profileFeedback {
                background: #f0f2ef;
                border: 1px solid #d8ddd9;
                border-radius: 4px;
                padding: 7px;
            }
            QLabel#verificationFeedback {
                color: #6c756f;
                padding: 4px 6px;
            }
            QLineEdit, QTableWidget, QTabWidget::pane {
                background: #ffffff;
                border: 1px solid #c9cfca;
                border-radius: 4px;
            }
            QLineEdit {
                min-height: 30px;
                padding: 0 7px;
            }
            QProgressBar {
                min-height: 22px;
                border: 1px solid #aeb6b0;
                border-radius: 3px;
                background: #ffffff;
                text-align: center;
            }
            QProgressBar::chunk {
                background: #1f7a5b;
            }
        """)

    def _restore_window_state(self):
        geometry = self.qt_settings.value('window_geometry')
        if geometry is not None:
            self.restoreGeometry(geometry)
        else:
            self.resize(1180, 760)
        self.setMinimumSize(1080, 660)

    def _language_changed(self):
        language = self.language_combo.currentData()
        if language in TRANSLATIONS:
            self.language = language
            self.qt_settings.setValue('language', language)
            self._retranslate_ui()

    def _retranslate_ui(self):
        self.setWindowTitle(self._tr('window_title'))
        self.title_label.setText(self._tr('app_title'))
        self.model_label.setText(self._tr('model'))
        self.side_label.setText(self._tr('side'))
        self.camera_label.setText(self._tr('camera'))
        self.side_combo.setItemText(0, self._tr('left'))
        self.side_combo.setItemText(1, self._tr('right'))
        self.refresh_button.setText(self._tr('refresh'))
        self.advanced_button.setText(self._tr('advanced'))
        self.width_label.setText(self._tr('width'))
        self.height_label.setText(self._tr('height'))
        self.camera_fps_label.setText(self._tr('camera_fps'))
        self.processing_fps_label.setText(self._tr('processing_fps'))
        self.mirror_checkbox.setText(self._tr('mirror'))
        self.sample_duration_label.setText(self._tr('sample_duration'))
        self.confidence_threshold_label.setText(
            self._tr('confidence_threshold')
        )
        self.minimum_samples_label.setText(self._tr('minimum_samples'))
        self.minimum_valid_ratio_label.setText(
            self._tr('minimum_valid_ratio')
        )
        self.maximum_spread_label.setText(self._tr('maximum_spread'))
        self.minimum_range_label.setText(self._tr('minimum_range'))
        self.status_group.setTitle(self._tr('status'))
        self.pose_group.setTitle(self._tr('pose_group'))
        self.workflow_tabs.setTabText(0, self._tr('pose_group'))
        self.workflow_tabs.setTabText(1, self._tr('profile_group'))
        self.profile_label.setText(self._tr('profile'))
        self.profile_name_label.setText(self._tr('profile_name'))
        self.profile_path_name_label.setText(self._tr('profile_path'))
        self.import_profile_button.setText(self._tr('import_profile'))
        self.result_table.setHorizontalHeaderLabels([
            self._tr('result_joint'),
            self._tr('result_min'),
            self._tr('result_max'),
            self._tr('result_range'),
            self._tr('result_status'),
        ])
        for key, label in self.status_name_labels.items():
            label.setText(self._tr(key))
        self._refresh_pose_items()
        self._refresh_sample_feedback()
        self._update_sample_controls()
        self.cancel_sample_button.setText(self._tr('cancel_sample'))
        self.save_as_button.setText(self._tr('save_as'))
        self._refresh_verification_feedback()
        self._refresh_profiles(
            select_path=(
                self.current_profile.path if self.current_profile else None
            )
        )
        self._refresh_profile_results()
        self._pose_selected(self.pose_list.currentRow())
        self._update_connection_controls()
        self._refresh_live_status()
        self._update_verification_controls()
        if self.connection_state == 'disconnected' and self.video_preview._image is None:
            self.video_preview.clear_image(self._tr('preview_wait'))

    def _pose_selected(self, row):
        if row < 0 or row >= len(self.pose_keys):
            return
        key = self.pose_keys[row]
        self.pose_help_label.setText(self._tr(f'{key}_help'))
        if self.sampling_pose is None:
            self._update_sample_controls()

    def _joint_label(self, joint):
        labels = JOINT_LABELS.get(joint)
        if labels is None:
            return joint
        return labels[0] if self.language == 'zh' else labels[1]

    def _refresh_pose_items(self):
        symbols = {
            'not_sampled': '○',
            'sampling': '●',
            'sample_success': '✓',
            'sample_failed': '!',
        }
        colors = {
            'not_sampled': QtGui.QColor('#202522'),
            'sampling': QtGui.QColor('#8a5a16'),
            'sample_success': QtGui.QColor('#166b4f'),
            'sample_failed': QtGui.QColor('#a33b32'),
        }
        for row, pose in enumerate(self.pose_order):
            key = POSE_UI_KEYS[pose]
            state = self.pose_attempt_states[pose]
            item = self.pose_list.item(row)
            item.setText(
                f'{symbols[state]}  {self._tr(key)}  ·  {self._tr(state)}'
            )
            item.setForeground(QtGui.QBrush(colors[state]))

    def _set_sample_feedback(
        self, key, color='#6c756f', keep_old=False, **kwargs
    ):
        self.sample_feedback_key = key
        self.sample_feedback_kwargs = kwargs
        self.sample_feedback_color = color
        self.sample_feedback_keep_old = keep_old
        self._refresh_sample_feedback()

    def _set_live_sample_hint(self, key, color):
        live_keys = {
            'wait_stable',
            'ready_to_sample',
            'ready_no_hand',
            'ready_wrong_hand',
            'ready_low_confidence',
            'ready_hand_clipped',
            'ready_incomplete_angles',
        }
        if self.sample_feedback_key in live_keys:
            self._set_sample_feedback(key, color)

    def _refresh_sample_feedback(self):
        if not hasattr(self, 'sample_feedback_label'):
            return
        text = self._tr(self.sample_feedback_key).format(
            **self.sample_feedback_kwargs
        )
        if self.sample_feedback_keep_old:
            text += self._tr('sample_old_kept')
        self.sample_feedback_label.setText(text)
        self.sample_feedback_label.setStyleSheet(
            f'color: {self.sample_feedback_color};'
        )

    def _toggle_advanced(self, checked):
        self.advanced_widget.setVisible(checked)
        self.advanced_button.setArrowType(
            QtCore.Qt.DownArrow if checked else QtCore.Qt.RightArrow
        )

    def _selected_profile(self):
        return (
            str(self.model_combo.currentData()),
            str(self.side_combo.currentData()),
        )

    def _restore_external_profile_paths(self):
        values = self.qt_settings.value('external_profiles', [])
        if isinstance(values, str):
            values = [values] if values else []
        return tuple(
            str(Path(value).expanduser())
            for value in (values or [])
            if str(value).strip()
        )

    def _save_external_profile_paths(self):
        self.qt_settings.setValue(
            'external_profiles', list(self.external_profile_paths)
        )

    def _default_profile_name(self):
        model_id, side = self._selected_profile()
        side_text = self._tr(side)
        suffix = '个人标定' if self.language == 'zh' else 'personal'
        return f'{model_id.upper()} {side_text} {suffix}'

    def _profile_display_text(self, profile):
        markers = []
        if profile.external:
            markers.append(self._tr('profile_external'))
        if profile.legacy:
            markers.append(self._tr('profile_legacy'))
        suffix = f' [{", ".join(markers)}]' if markers else ''
        return f'{profile.name}{suffix}'

    def _refresh_profiles(self, select_path=None):
        if not hasattr(self, 'profile_combo'):
            return
        model_id, side = self._selected_profile()
        profiles, errors = scan_profiles(
            model_id,
            side,
            directory=DEFAULT_PROFILE_DIRECTORY,
            external_paths=self.external_profile_paths,
        )
        self.available_profiles = profiles
        self.profile_scan_errors = errors
        for path, reason in errors:
            print(
                f'[calibration_gui] 忽略无效个人配置 {path}：{reason}',
                flush=True,
            )

        desired = Path(select_path).resolve() if select_path else None
        self.profile_combo_guard = True
        self.profile_combo.clear()
        self.profile_combo.addItem(self._tr('new_profile'), '')
        selected_index = 0
        for profile in profiles:
            self.profile_combo.addItem(
                self._profile_display_text(profile), str(profile.path)
            )
            if desired is not None and profile.path == desired:
                selected_index = self.profile_combo.count() - 1
        self.profile_combo.setCurrentIndex(selected_index)
        self.profile_combo_guard = False

        if selected_index == 0:
            self.current_profile = None
            if not self.profile_name_edit.text().strip():
                self.profile_name_edit.setText(self._default_profile_name())
            self.profile_path_edit.setText(self._tr('profile_path_default'))
        else:
            self._apply_selected_profile(profiles[selected_index - 1])
        self._update_profile_controls()

    def _confirm_discard_samples(self):
        if not self.samples_dirty and self.sampling_pose is None:
            return True
        answer = QtWidgets.QMessageBox.question(
            self,
            self._tr('unsaved_title'),
            self._tr('unsaved_profile'),
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        return answer == QtWidgets.QMessageBox.Yes

    def _clear_pose_results(self):
        if self.sampling_pose is not None:
            self._cancel_sampling()
        self.pose_results.clear()
        self.samples_dirty = False
        self.pose_attempt_states = {
            pose: 'not_sampled' for pose in self.pose_order
        }
        self._set_sample_feedback('wait_stable')
        self._refresh_pose_items()
        self._update_sample_controls()
        self._refresh_profile_results()

    def _profile_selection_changed(self, index):
        if self.profile_combo_guard or index < 0:
            return
        previous_path = (
            str(self.current_profile.path) if self.current_profile else ''
        )
        selected_path = str(self.profile_combo.itemData(index) or '')
        if selected_path == previous_path:
            return
        if not self._confirm_discard_samples():
            self.profile_combo_guard = True
            previous_index = self.profile_combo.findData(previous_path)
            self.profile_combo.setCurrentIndex(max(previous_index, 0))
            self.profile_combo_guard = False
            return
        self._clear_pose_results()
        if not selected_path:
            self.current_profile = None
            self._load_default_template()
            self.profile_name_edit.setText(self._default_profile_name())
            self.profile_path_edit.setText(self._tr('profile_path_default'))
        else:
            profile = next((
                item for item in self.available_profiles
                if str(item.path) == selected_path
            ), None)
            if profile is None:
                profile = load_profile(selected_path, external=True)
            self._apply_selected_profile(profile)
        self._update_profile_controls()

    def _apply_selected_profile(self, profile):
        document, parameters = load_parameters(profile.path)
        self.current_profile = profile
        self.template_document = document
        self.template_parameters = parameters
        self.profile_name_edit.setText(profile.name)
        self.profile_path_edit.setText(str(profile.path))
        self.profile_path_edit.setCursorPosition(0)

    def _import_profile(self):
        path, _selected_filter = QtWidgets.QFileDialog.getOpenFileName(
            self,
            self._tr('profile_import_title'),
            str(Path.home()),
            self._tr('profile_import_filter'),
        )
        if not path:
            return
        try:
            profile = load_profile(path, external=True)
        except (OSError, ValueError) as error:
            QtWidgets.QMessageBox.critical(
                self,
                self._tr('profile_import_failed_title'),
                self._tr('profile_import_failed').format(reason=error),
            )
            return
        model_id, side = self._selected_profile()
        if profile.model_id != model_id or profile.side != side:
            QtWidgets.QMessageBox.warning(
                self,
                self._tr('profile_import_failed_title'),
                self._tr('profile_mismatch').format(
                    model=profile.model_id.upper(),
                    side=self._tr(profile.side),
                    current_model=model_id.upper(),
                    current_side=self._tr(side),
                ),
            )
            return
        if not self._confirm_discard_samples():
            return
        self._clear_pose_results()
        resolved = str(profile.path)
        if resolved not in self.external_profile_paths:
            self.external_profile_paths = (
                *self.external_profile_paths,
                resolved,
            )
            self._save_external_profile_paths()
        self._refresh_profiles(select_path=profile.path)

    def _load_default_template(self):
        model_id, side = self._selected_profile()
        document, parameters = load_parameters(
            self._template_path(model_id, side)
        )
        self.template_document = document
        self.template_parameters = parameters
        return True

    def _is_repository_template(self, path):
        try:
            resolved = Path(path).expanduser().resolve()
        except OSError:
            return False
        return any(
            resolved == self._template_path(model_id, side).resolve()
            for model_id in ('l30', 'o6')
            for side in ('left', 'right')
        )

    def _template_path(self, model_id, side):
        share = Path(get_package_share_directory('linkerhand_retargeting'))
        filename = (
            f'retargeting_{side}.yaml'
            if model_id == 'l30'
            else f'retargeting_{model_id}_{side}.yaml'
        )
        return share / 'config' / filename

    def _load_template_for_selection(self):
        model_id, side = self._selected_profile()
        try:
            self._load_default_template()
        except ValueError as error:
            self.template_document = None
            self.template_parameters = None
            QtWidgets.QMessageBox.critical(
                self,
                self._tr('template_failed_title'),
                self._tr('template_failed').format(
                    model=model_id.upper(),
                    side=side,
                    reason=error,
                ),
            )
            return False
        self.loaded_model = model_id
        self.loaded_side = side
        self.current_profile = None
        if hasattr(self, 'profile_combo'):
            self._refresh_profiles()
        return True

    def _profile_changed(self):
        if self.profile_change_guard or not hasattr(self, 'pose_results'):
            return
        model_id, side = self._selected_profile()
        if (
            getattr(self, 'loaded_model', None) == model_id
            and getattr(self, 'loaded_side', None) == side
        ):
            return
        if self.samples_dirty or self.sampling_pose is not None:
            answer = QtWidgets.QMessageBox.question(
                self,
                self._tr('unsaved_title'),
                self._tr('unsaved_profile'),
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.No,
            )
            if answer != QtWidgets.QMessageBox.Yes:
                self.profile_change_guard = True
                self.model_combo.setCurrentIndex(
                    self.model_combo.findData(self.loaded_model)
                )
                self.side_combo.setCurrentIndex(
                    self.side_combo.findData(self.loaded_side)
                )
                self.profile_change_guard = False
                return
        self._clear_pose_results()
        self._load_template_for_selection()
        self.profile_name_edit.setText(self._default_profile_name())

    def _selected_pose(self):
        row = self.pose_list.currentRow()
        if row < 0 or row >= len(self.pose_order):
            return None
        return self.pose_order[row]

    def _detection_ready(self, now=None):
        now = time.monotonic() if now is None else now
        return (
            self.connection_state == 'connected'
            and self.template_parameters is not None
            and self.last_pose_time is not None
            and now - self.last_pose_time <= 0.75
            and self.valid_detection_started is not None
            and now - self.valid_detection_started >= 0.5
        )

    def _update_sample_controls(self):
        if not hasattr(self, 'sample_button'):
            return
        sampling = self.sampling_pose is not None
        selected_pose = self._selected_pose()
        ready = self._detection_ready() and selected_pose is not None
        self.sample_button.setEnabled(ready and not sampling)
        self.cancel_sample_button.setEnabled(sampling)
        if selected_pose in self.pose_results:
            self.sample_button.setText(self._tr('resample'))
        else:
            self.sample_button.setText(self._tr('start_sample'))
        self.pose_list.setEnabled(not sampling)
        for widget in (
            self.sample_duration_spin,
            self.confidence_threshold_spin,
            self.minimum_samples_spin,
            self.minimum_valid_ratio_spin,
            self.maximum_spread_spin,
            self.minimum_range_spin,
        ):
            widget.setEnabled(not sampling)

    def _start_sampling(self):
        pose = self._selected_pose()
        if (
            pose is None
            or self.template_parameters is None
            or not self._detection_ready()
        ):
            self._set_sample_feedback('wait_stable', '#8a5a16')
            self._update_sample_controls()
            return
        self.sample_collector = PoseSampleCollector(
            self.template_parameters,
            str(self.side_combo.currentData()),
            confidence_threshold=self.confidence_threshold_spin.value(),
        )
        self.sampling_pose = pose
        self.sampling_started_at = time.monotonic()
        self.pose_attempt_states[pose] = 'sampling'
        self.sample_progress.setValue(0)
        self.sample_progress.setFormat(
            f'0.0 / {self.sample_duration_spin.value():.1f} s'
        )
        self.sample_progress.setVisible(True)
        self._set_sample_feedback('sampling_hold', '#8a5a16')
        self._refresh_pose_items()
        self._update_sample_controls()
        self.sample_timer.start()

    def _cancel_sampling(self):
        pose = self.sampling_pose
        if pose is None:
            return
        had_old_result = pose in self.pose_results
        self.sample_timer.stop()
        self.sample_collector = None
        self.sampling_pose = None
        self.sampling_started_at = None
        self.pose_attempt_states[pose] = (
            'sample_success' if had_old_result else 'not_sampled'
        )
        self.sample_progress.setVisible(False)
        self._set_sample_feedback(
            'sample_cancelled_kept' if had_old_result else 'sample_cancelled',
            '#6c756f',
        )
        self._refresh_pose_items()
        self._update_sample_controls()

    def _update_sampling_progress(self):
        if self.sampling_pose is None or self.sampling_started_at is None:
            self.sample_timer.stop()
            return
        duration = self.sample_duration_spin.value()
        elapsed = max(time.monotonic() - self.sampling_started_at, 0.0)
        fraction = min(elapsed / duration, 1.0)
        self.sample_progress.setValue(round(fraction * 1000.0))
        self.sample_progress.setFormat(
            f'{min(elapsed, duration):.1f} / {duration:.1f} s'
        )
        if elapsed >= duration:
            self._finish_sampling()

    def _finish_sampling(self):
        pose = self.sampling_pose
        collector = self.sample_collector
        if pose is None or collector is None:
            return
        self.sample_timer.stop()
        result = collector.finish(
            minimum_samples=self.minimum_samples_spin.value(),
            minimum_valid_ratio=self.minimum_valid_ratio_spin.value(),
            maximum_spread_deg=self.maximum_spread_spin.value(),
        )
        had_old_result = pose in self.pose_results
        self.sample_collector = None
        self.sampling_pose = None
        self.sampling_started_at = None
        self.sample_progress.setVisible(False)

        if result.success:
            self.pose_results[pose] = result
            self.samples_dirty = True
            self.pose_attempt_states[pose] = 'sample_success'
            maximum_spread = max(result.spreads.values(), default=0.0)
            self._set_sample_feedback(
                'sample_success_detail',
                '#166b4f',
                valid=result.valid_frames,
                total=result.total_frames,
                ratio=result.valid_ratio,
                spread=maximum_spread,
            )
            self._select_next_pose(pose)
        else:
            self.pose_attempt_states[pose] = (
                'sample_success' if had_old_result else 'sample_failed'
            )
            self._show_sampling_failure(result, had_old_result)
        self._refresh_pose_items()
        self._update_sample_controls()
        self._refresh_profile_results()

    def _pose_summaries(self):
        return {
            pose: result.summary for pose, result in self.pose_results.items()
        }

    def _all_poses_complete(self):
        return all(pose in self.pose_results for pose in self.pose_order)

    def _profile_range_check(self):
        if self.template_parameters is None or not self._all_poses_complete():
            return {}, ()
        return validate_calibration_ranges(
            self.template_parameters,
            self._pose_summaries(),
            minimum_range_deg=self.minimum_range_spin.value(),
        )

    def _refresh_profile_results(self):
        if not hasattr(self, 'result_table'):
            return
        self.result_table.setRowCount(0)
        if not self._all_poses_complete() or self.template_parameters is None:
            self.profile_feedback_label.setText(self._tr('profile_incomplete'))
            self.profile_feedback_label.setStyleSheet('color: #6c756f;')
            self._update_profile_controls()
            return

        try:
            endpoints, invalid = self._profile_range_check()
        except ValueError as error:
            self.profile_feedback_label.setText(str(error))
            self.profile_feedback_label.setStyleSheet('color: #a33b32;')
            self._update_profile_controls()
            return
        invalid_by_joint = {item['joint']: item for item in invalid}
        self.result_table.setRowCount(len(endpoints))
        for row, (joint, values) in enumerate(endpoints.items()):
            minimum_pose = self._tr(POSE_UI_KEYS[values['minimum_pose']])
            maximum_pose = self._tr(POSE_UI_KEYS[values['maximum_pose']])
            invalid_joint = joint in invalid_by_joint
            cells = (
                self._joint_label(joint),
                f'{values["input_min"]:.1f}°',
                f'{values["input_max"]:.1f}°',
                f'{values["range"]:.1f}°',
                self._tr('result_invalid' if invalid_joint else 'result_ok'),
            )
            for column, text in enumerate(cells):
                item = QtWidgets.QTableWidgetItem(text)
                if column == 1:
                    item.setToolTip(
                        f'{self._tr("result_min_pose")}: {minimum_pose}'
                    )
                elif column == 2:
                    item.setToolTip(
                        f'{self._tr("result_max_pose")}: {maximum_pose}'
                    )
                if column == 4:
                    item.setForeground(QtGui.QBrush(QtGui.QColor(
                        '#a33b32' if invalid_joint else '#166b4f'
                    )))
                self.result_table.setItem(row, column, item)
        self.profile_feedback_label.setText(
            self._tr('profile_invalid' if invalid else 'profile_ready')
        )
        self.profile_feedback_label.setStyleSheet(
            f'color: {"#a33b32" if invalid else "#166b4f"};'
        )
        self._update_profile_controls()

    def _profile_can_save(self):
        if not self._all_poses_complete() or self.template_parameters is None:
            return False
        try:
            _endpoints, invalid = self._profile_range_check()
            validate_profile_name(self.profile_name_edit.text())
        except ValueError:
            return False
        return not invalid

    def _update_profile_controls(self):
        if not hasattr(self, 'generate_button'):
            return
        verification_idle = self.verification_state == 'idle'
        can_save = self._profile_can_save() and verification_idle
        has_current = self.current_profile is not None
        read_only_current = (
            has_current
            and self._is_repository_template(self.current_profile.path)
        )
        self.generate_button.setText(
            self._tr('update_profile' if has_current else 'generate')
        )
        self.generate_button.setEnabled(can_save and not read_only_current)
        if read_only_current:
            self.generate_button.setToolTip(self._tr('profile_read_only'))
        else:
            self.generate_button.setToolTip('')
        self.save_as_button.setEnabled(can_save)
        self._update_verification_controls()

    def _verification_mode_text(self, mode=None):
        value = mode or self.verification_mode or self.pending_verification_mode
        return 'RViz' if value == 'rviz' else 'Gazebo'

    def _set_verification_feedback(
        self, key, color='#6c756f', **kwargs
    ):
        self.verification_feedback_key = key
        self.verification_feedback_kwargs = kwargs
        self.verification_feedback_color = color
        self._refresh_verification_feedback()

    def _refresh_verification_feedback(self):
        if not hasattr(self, 'verification_feedback_label'):
            return
        text = self._tr(self.verification_feedback_key).format(
            **self.verification_feedback_kwargs
        )
        self.verification_feedback_label.setText(text)
        self.verification_feedback_label.setStyleSheet(
            f'color: {self.verification_feedback_color}; padding: 4px 6px;'
        )

    def _verification_profile_path(self):
        if self.current_profile is None:
            return None
        path = Path(self.current_profile.path).expanduser()
        return path.resolve() if path.is_file() else None

    def _update_verification_controls(self):
        if not hasattr(self, 'rviz_button'):
            return
        active = self.verification_state != 'idle'
        running_mode = self.verification_mode or self.pending_verification_mode
        if active:
            self.rviz_button.setText(
                self._tr('stop_rviz' if running_mode == 'rviz' else 'verify_rviz')
            )
            self.gazebo_button.setText(
                self._tr(
                    'stop_gazebo' if running_mode == 'gazebo'
                    else 'verify_gazebo'
                )
            )
            stoppable = self.verification_state == 'running'
            self.rviz_button.setEnabled(stoppable and running_mode == 'rviz')
            self.gazebo_button.setEnabled(
                stoppable and running_mode == 'gazebo'
            )
        else:
            self.rviz_button.setText(self._tr('verify_rviz'))
            self.gazebo_button.setText(self._tr('verify_gazebo'))
            profile_path = self._verification_profile_path()
            can_verify = (
                profile_path is not None
                and not self.samples_dirty
                and self.sampling_pose is None
                and self.connection_state not in {'starting', 'stopping'}
            )
            self.rviz_button.setEnabled(can_verify)
            self.gazebo_button.setEnabled(can_verify)
            if self.samples_dirty:
                self._set_verification_feedback(
                    'verification_unsaved', '#8a5a16'
                )
            elif profile_path is None:
                self._set_verification_feedback(
                    'verification_profile_required', '#6c756f'
                )
            else:
                self._set_verification_feedback(
                    'verification_ready', '#166b4f'
                )
        self._update_connection_controls()

    def _toggle_verification(self, mode):
        if self.verification_state == 'idle':
            self._request_verification(mode)
        elif (
            self.verification_mode == mode
            or self.pending_verification_mode == mode
        ):
            self._stop_verification()

    def _request_verification(self, mode):
        profile_path = self._verification_profile_path()
        if profile_path is None:
            self._set_verification_feedback(
                'verification_profile_required', '#a33b32'
            )
            return
        if self.samples_dirty:
            self._set_verification_feedback('verification_unsaved', '#a33b32')
            return
        device = self.camera_combo.currentText().strip()
        if not device or not Path(device).exists():
            QtWidgets.QMessageBox.warning(
                self,
                self._tr('device_missing_title'),
                self._tr('device_missing').format(device=device or '—'),
            )
            return

        self.pending_verification_mode = mode
        self.verification_restore_connection = self.connection_state in {
            'starting', 'connected'
        }
        self.verification_state = 'starting'
        self._set_verification_feedback(
            'verification_starting',
            '#8a5a16',
            mode=self._verification_mode_text(mode),
        )
        self._update_verification_controls()
        if self.connection_state == 'disconnected':
            self._start_verification_process()
        else:
            self._disconnect_perception()

    def _verification_settings(self, mode, profile_path):
        model_id, side = self._selected_profile()
        return VerificationSettings(
            mode=mode,
            model_id=model_id,
            side=side,
            parameters_file=str(profile_path),
            device=self.camera_combo.currentText().strip(),
            width=self.width_spin.value(),
            height=self.height_spin.value(),
            camera_fps=self.camera_fps_spin.value(),
            processing_fps=self.processing_fps_spin.value(),
            mirror_preview=self.mirror_checkbox.isChecked(),
        )

    def _start_verification_process(self):
        mode = self.pending_verification_mode
        profile_path = self._verification_profile_path()
        if mode is None or profile_path is None or self.shutting_down:
            self._reset_verification_state()
            return
        try:
            program, arguments = build_verification_command(
                self._verification_settings(mode, profile_path)
            )
        except (OSError, ValueError) as error:
            self._verification_start_failed(mode, str(error))
            return

        process = QtCore.QProcess(self)
        process.setProcessChannelMode(QtCore.QProcess.ForwardedChannels)
        process.started.connect(self._on_verification_started)
        process.finished.connect(self._on_verification_finished)
        process.errorOccurred.connect(self._on_verification_process_error)
        self.verification_process = process
        self.intentional_verification_stop = False
        print(
            '[calibration_gui] 启动仿真验证：'
            + ' '.join((program, *arguments)),
            flush=True,
        )
        process.start(program, list(arguments))

    def _on_verification_started(self):
        self.verification_mode = self.pending_verification_mode
        self.pending_verification_mode = None
        self.verification_state = 'running'
        self._set_verification_feedback(
            'verification_running',
            '#166b4f',
            mode=self._verification_mode_text(),
        )
        self._update_verification_controls()

    def _on_verification_process_error(self, error):
        if self.intentional_verification_stop or self.shutting_down:
            return
        mode = self.verification_mode or self.pending_verification_mode
        reason = (
            self.verification_process.errorString()
            if self.verification_process is not None else str(error)
        )
        self._verification_start_failed(mode, reason or str(error))

    def _verification_start_failed(self, mode, reason):
        mode_text = self._verification_mode_text(mode)
        process = self.verification_process
        self.verification_process = None
        if process is not None:
            process.deleteLater()
        restore = self.verification_restore_connection
        self._reset_verification_state()
        self._set_verification_feedback(
            'verification_failed', '#a33b32', mode=mode_text, reason=reason
        )
        QtWidgets.QMessageBox.critical(
            self,
            self._tr('verification_failed_title'),
            self._tr('verification_failed').format(
                mode=mode_text, reason=reason
            ),
        )
        if restore and not self.shutting_down:
            self._connect_perception()

    def _stop_verification(self):
        if self.verification_state == 'idle':
            return
        mode_text = self._verification_mode_text()
        self.verification_state = 'stopping'
        self.intentional_verification_stop = True
        self._set_verification_feedback(
            'verification_stopping', '#8a5a16', mode=mode_text
        )
        self._update_verification_controls()
        process = self.verification_process
        if process is None or process.state() == QtCore.QProcess.NotRunning:
            self._finish_verification_cleanup(mode_text)
            return
        self._signal_qprocess_group(process, signal.SIGINT)
        QtCore.QTimer.singleShot(5000, self._force_kill_verification)

    def _force_kill_verification(self):
        process = self.verification_process
        if process is not None and process.state() != QtCore.QProcess.NotRunning:
            self._signal_qprocess_group(process, signal.SIGKILL)

    def _on_verification_finished(self, exit_code, _exit_status):
        if (
            self.verification_state == 'idle'
            and self.verification_process is None
        ):
            return
        mode_text = self._verification_mode_text()
        if (
            not self.intentional_verification_stop
            and exit_code != 0
            and not self.shutting_down
        ):
            print(
                f'[calibration_gui] {mode_text} 验证进程退出码：{exit_code}',
                flush=True,
            )
        self._finish_verification_cleanup(mode_text)

    def _finish_verification_cleanup(self, mode_text):
        if (
            self.verification_state == 'idle'
            and self.verification_process is None
        ):
            return
        process = self.verification_process
        self.verification_process = None
        if process is not None:
            process.deleteLater()
        restore = self.verification_restore_connection
        self._reset_verification_state()
        self._set_verification_feedback(
            (
                'verification_finished_reconnecting'
                if restore and not self.shutting_down
                else 'verification_finished'
            ),
            '#166b4f',
            mode=mode_text,
        )
        if restore and not self.shutting_down:
            self._connect_perception()

    def _reset_verification_state(self):
        self.verification_state = 'idle'
        self.verification_mode = None
        self.pending_verification_mode = None
        self.verification_restore_connection = False
        self.intentional_verification_stop = False
        self._update_verification_controls()

    def _quality_settings(self):
        return {
            'sample_duration_sec': round(
                float(self.sample_duration_spin.value()), 2
            ),
            'confidence_threshold': round(
                float(self.confidence_threshold_spin.value()), 3
            ),
            'minimum_samples': int(self.minimum_samples_spin.value()),
            'minimum_valid_ratio': round(
                float(self.minimum_valid_ratio_spin.value()), 3
            ),
            'maximum_spread_deg': round(
                float(self.maximum_spread_spin.value()), 2
            ),
            'minimum_range_deg': round(
                float(self.minimum_range_spin.value()), 2
            ),
        }

    def _build_profile_document(self, created_at=''):
        name = validate_profile_name(self.profile_name_edit.text())
        document = build_personal_calibration(
            self.template_document,
            self._pose_summaries(),
            minimum_range_deg=self.minimum_range_spin.value(),
        )
        model_id, side = self._selected_profile()
        metadata = build_profile_metadata(
            name,
            model_id,
            side,
            self.camera_combo.currentText().strip(),
            self.pose_results,
            self._quality_settings(),
            created_at=created_at,
        )
        return attach_profile_metadata(document, metadata)

    def _save_profile(self):
        self._write_profile(save_as=False)

    def _save_profile_as(self):
        self._write_profile(save_as=True)

    def _write_profile(self, save_as):
        try:
            name = validate_profile_name(self.profile_name_edit.text())
        except ValueError as error:
            QtWidgets.QMessageBox.warning(
                self,
                self._tr('profile_save_failed_title'),
                self._tr('profile_name_invalid').format(reason=error),
            )
            return
        current = self.current_profile
        update_existing = current is not None and not save_as
        if update_existing and self._is_repository_template(current.path):
            QtWidgets.QMessageBox.warning(
                self,
                self._tr('profile_save_failed_title'),
                self._tr('profile_read_only'),
            )
            return
        if save_as:
            model_id, side = self._selected_profile()
            suggested_path = unique_profile_path(
                DEFAULT_PROFILE_DIRECTORY,
                model_id,
                side,
                name,
            )
            selected_path, _selected_filter = (
                QtWidgets.QFileDialog.getSaveFileName(
                    self,
                    self._tr('profile_save_as_title'),
                    str(suggested_path),
                    self._tr('profile_save_filter'),
                )
            )
            if not selected_path:
                return
            path = Path(selected_path).expanduser()
            if path.suffix.lower() not in {'.yaml', '.yml'}:
                path = path.with_suffix('.yaml')
            if path.exists():
                answer = QtWidgets.QMessageBox.question(
                    self,
                    self._tr('profile_overwrite_title'),
                    self._tr('profile_overwrite').format(path=path),
                    QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                    QtWidgets.QMessageBox.No,
                )
                if answer != QtWidgets.QMessageBox.Yes:
                    return
            created_at = ''
        elif update_existing:
            answer = QtWidgets.QMessageBox.question(
                self,
                self._tr('profile_overwrite_title'),
                self._tr('profile_overwrite').format(path=current.path),
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.No,
            )
            if answer != QtWidgets.QMessageBox.Yes:
                return
            path = current.path
            try:
                current_document, _parameters = load_parameters(path)
                created_at = str(
                    extract_profile_metadata(current_document).get(
                        'created_at', current.created_at
                    )
                )
            except ValueError:
                created_at = current.created_at
        else:
            model_id, side = self._selected_profile()
            path = unique_profile_path(
                DEFAULT_PROFILE_DIRECTORY,
                model_id,
                side,
                name,
            )
            created_at = ''
        try:
            document = self._build_profile_document(created_at=created_at)
            saved_path = write_calibration(path, document)
            saved_profile = load_profile(
                saved_path,
                external=not saved_path.is_relative_to(
                    DEFAULT_PROFILE_DIRECTORY.resolve()
                ),
            )
        except (OSError, ValueError) as error:
            QtWidgets.QMessageBox.critical(
                self,
                self._tr('profile_save_failed_title'),
                self._tr('profile_save_failed').format(reason=error),
            )
            return

        if saved_profile.external and str(saved_path) not in self.external_profile_paths:
            self.external_profile_paths = (
                *self.external_profile_paths,
                str(saved_path),
            )
            self._save_external_profile_paths()
        self.samples_dirty = False
        self.last_saved_profile_path = saved_path
        self._refresh_profiles(select_path=saved_path)
        self.profile_path_edit.setText(str(saved_path))
        self.profile_path_edit.setCursorPosition(0)
        QtWidgets.QMessageBox.information(
            self,
            self._tr('profile_saved_title'),
            self._tr(
                'profile_updated' if update_existing else 'profile_saved'
            ).format(path=saved_path),
        )

    def _select_next_pose(self, completed_pose):
        start = self.pose_order.index(completed_pose) + 1
        ordered = self.pose_order[start:] + self.pose_order[:start]
        for pose in ordered:
            if pose not in self.pose_results:
                self.pose_list.setCurrentRow(self.pose_order.index(pose))
                return

    def _invalid_hint(self, reason):
        keys = {
            INVALID_NO_HAND: 'invalid_no_hand',
            INVALID_WRONG_HAND: 'invalid_wrong_hand',
            INVALID_LOW_CONFIDENCE: 'invalid_low_confidence',
            INVALID_HAND_CLIPPED: 'invalid_hand_clipped',
            INVALID_INCOMPLETE_ANGLES: 'invalid_incomplete_angles',
        }
        key = keys.get(reason)
        return self._tr(key) if key else ''

    def _show_sampling_failure(self, result, had_old_result):
        common = {
            'valid': result.valid_frames,
            'total': result.total_frames,
            'ratio': result.valid_ratio,
            'hint': self._invalid_hint(result.dominant_invalid_reason),
        }
        if result.reason_code == FAIL_NO_MESSAGES:
            key = 'sample_no_messages'
            kwargs = {}
        elif result.reason_code == FAIL_INSUFFICIENT_SAMPLES:
            key = 'sample_few_frames'
            kwargs = {
                **common,
                'required': self.minimum_samples_spin.value(),
            }
        elif result.reason_code == FAIL_LOW_VALID_RATIO:
            key = 'sample_low_ratio'
            kwargs = {
                **common,
                'required': self.minimum_valid_ratio_spin.value(),
            }
        elif result.reason_code == FAIL_UNSTABLE:
            key = 'sample_unstable'
            joint = result.reason_detail
            kwargs = {
                'joint': self._joint_label(joint),
                'spread': result.spreads[joint],
                'limit': self.maximum_spread_spin.value(),
            }
        else:
            key = 'sample_no_messages'
            kwargs = {}
        self.sample_feedback_key = key
        self.sample_feedback_kwargs = kwargs
        self.sample_feedback_color = '#a33b32'
        self.sample_feedback_keep_old = had_old_result
        self._refresh_sample_feedback()

    def _refresh_devices(self):
        current = self.camera_combo.currentText().strip()
        devices = list_video_devices()
        blocker = QtCore.QSignalBlocker(self.camera_combo)
        self.camera_combo.clear()
        self.camera_combo.addItems(devices)
        preferred = current or str(
            self.qt_settings.value('last_camera', '')
        ).strip()
        if preferred:
            index = self.camera_combo.findText(preferred)
            if index < 0:
                self.camera_combo.addItem(preferred)
                index = self.camera_combo.count() - 1
            self.camera_combo.setCurrentIndex(index)
        elif devices:
            self.camera_combo.setCurrentIndex(0)
        else:
            self.camera_combo.addItem('/dev/video0')
            self.camera_combo.setCurrentIndex(0)
        del blocker

    def _toggle_connection(self):
        if self.connection_state == 'disconnected':
            self._connect_perception()
        elif self.connection_state in {'starting', 'connected'}:
            self._disconnect_perception()

    def _connect_perception(self):
        device = self.camera_combo.currentText().strip()
        if not device or not Path(device).exists():
            QtWidgets.QMessageBox.warning(
                self,
                self._tr('device_missing_title'),
                self._tr('device_missing').format(device=device or '—'),
            )
            return

        side = str(self.side_combo.currentData())
        settings = PerceptionSettings(
            side=side,
            device=device,
            width=self.width_spin.value(),
            height=self.height_spin.value(),
            camera_fps=self.camera_fps_spin.value(),
            processing_fps=self.processing_fps_spin.value(),
            mirror_preview=self.mirror_checkbox.isChecked(),
        )
        program, arguments = build_perception_command(settings)

        self.qt_settings.setValue('last_camera', device)
        self.connection_state = 'starting'
        self.intentional_process_stop = False
        self.last_image_time = None
        self.last_pose_time = None
        self.latest_pose = None
        self.latest_pose_invalid_reason = INVALID_NO_HAND
        self.valid_detection_started = None
        self.image_times.clear()
        self.video_preview.clear_image(self._tr('preview_starting'))
        self._update_connection_controls()

        self.ros_thread = RosSubscriberThread(side, self)
        self.ros_thread.image_received.connect(self._on_image_received)
        self.ros_thread.pose_received.connect(self._on_pose_received)
        self.ros_thread.failed.connect(self._on_ros_failed)
        self.ros_thread.start()

        process = QtCore.QProcess(self)
        process.setProcessChannelMode(QtCore.QProcess.ForwardedChannels)
        process.started.connect(self._on_process_started)
        process.finished.connect(self._on_process_finished)
        process.errorOccurred.connect(self._on_process_error)
        self.perception_process = process
        print(
            '[calibration_gui] 启动感知管线：'
            + ' '.join((program, *arguments)),
            flush=True,
        )
        process.start(program, list(arguments))

    def _on_process_started(self):
        self._refresh_live_status()

    def _on_process_error(self, error):
        if self.intentional_process_stop or self.shutting_down:
            return
        reason = self.perception_process.errorString()
        self._stop_ros_thread()
        self._finalize_disconnected(self._tr('preview_stopped'))
        QtWidgets.QMessageBox.critical(
            self,
            self._tr('start_failed_title'),
            self._tr('start_failed').format(reason=reason or str(error)),
        )

    def _on_process_finished(self, exit_code, _exit_status):
        unexpected = not self.intentional_process_stop and not self.shutting_down
        self._stop_ros_thread()
        message = (
            self._tr('process_exited').format(code=exit_code)
            if unexpected
            else self._tr('preview_stopped')
        )
        self._finalize_disconnected(message)

    def _disconnect_perception(self):
        if self.sampling_pose is not None:
            self._cancel_sampling()
        self.intentional_process_stop = True
        self.connection_state = 'stopping'
        self._update_connection_controls()
        self._stop_ros_thread()
        process = self.perception_process
        if process is None or process.state() == QtCore.QProcess.NotRunning:
            self._finalize_disconnected(self._tr('preview_stopped'))
            return
        self._signal_process_group(signal.SIGINT)
        QtCore.QTimer.singleShot(3000, self._force_kill_process)

    def _signal_process_group(self, signal_number):
        process = self.perception_process
        self._signal_qprocess_group(process, signal_number)

    def _signal_qprocess_group(self, process, signal_number):
        if process is None or process.state() == QtCore.QProcess.NotRunning:
            return
        process_id = int(process.processId())
        try:
            os.killpg(process_id, signal_number)
        except (ProcessLookupError, PermissionError):
            if signal_number == signal.SIGKILL:
                process.kill()
            else:
                process.terminate()

    def _force_kill_process(self):
        process = self.perception_process
        if process is not None and process.state() != QtCore.QProcess.NotRunning:
            self._signal_process_group(signal.SIGKILL)

    def _stop_ros_thread(self):
        thread = self.ros_thread
        self.ros_thread = None
        if thread is None:
            return
        thread.stop()
        thread.wait(1500)
        thread.deleteLater()

    def _finalize_disconnected(self, preview_text):
        process = self.perception_process
        self.perception_process = None
        if process is not None:
            process.deleteLater()
        self.connection_state = 'disconnected'
        self.last_image_time = None
        self.last_pose_time = None
        self.latest_pose = None
        self.latest_pose_invalid_reason = INVALID_NO_HAND
        self.valid_detection_started = None
        self.image_times.clear()
        self.video_preview.clear_image(preview_text)
        self._update_connection_controls()
        self._refresh_live_status()
        if (
            self.verification_state == 'starting'
            and self.pending_verification_mode is not None
            and not self.shutting_down
        ):
            QtCore.QTimer.singleShot(150, self._start_verification_process)

    def _on_ros_failed(self, reason):
        if self.shutting_down or self.connection_state == 'disconnected':
            return
        print(f'[calibration_gui] ROS 订阅错误：{reason}', flush=True)

    def _on_image_received(self, image):
        if self.connection_state not in {'starting', 'connected'}:
            return
        now = time.monotonic()
        self.last_image_time = now
        self.image_times.append(now)
        self.video_preview.set_image(image)
        if self.connection_state == 'starting':
            self.connection_state = 'connected'
            self._update_connection_controls()

    def _on_pose_received(self, pose):
        if self.connection_state not in {'starting', 'connected'}:
            return
        now = time.monotonic()
        self.last_pose_time = now
        self.latest_pose = pose
        if self.sample_collector is not None:
            self.sample_collector.add_pose(pose)

        valid = False
        if self.template_parameters is not None:
            sample, reason = extract_valid_sample(
                self.template_parameters,
                pose,
                str(self.side_combo.currentData()),
                self.confidence_threshold_spin.value(),
            )
            valid = sample is not None
            self.latest_pose_invalid_reason = reason
        if valid:
            if self.valid_detection_started is None:
                self.valid_detection_started = now
        else:
            self.valid_detection_started = None
        self._update_sample_controls()

    def _update_connection_controls(self):
        disconnected = self.connection_state == 'disconnected'
        stopping = self.connection_state == 'stopping'
        verification_idle = self.verification_state == 'idle'
        for widget in (
            self.model_combo,
            self.side_combo,
            self.camera_combo,
            self.refresh_button,
            self.width_spin,
            self.height_spin,
            self.camera_fps_spin,
            self.processing_fps_spin,
            self.mirror_checkbox,
        ):
            widget.setEnabled(disconnected and verification_idle)
        for widget in (
            self.profile_combo,
            self.profile_name_edit,
            self.import_profile_button,
            self.minimum_range_spin,
        ):
            widget.setEnabled(verification_idle)
        self.connect_button.setEnabled(not stopping and verification_idle)
        if disconnected:
            self.connect_button.setText(self._tr('connect'))
            self.connect_button.setIcon(
                self.style().standardIcon(QtWidgets.QStyle.SP_MediaPlay)
            )
        elif stopping:
            self.connect_button.setText(self._tr('disconnecting'))
            self.connect_button.setIcon(QtGui.QIcon())
        else:
            self.connect_button.setText(self._tr('disconnect'))
            self.connect_button.setIcon(
                self.style().standardIcon(QtWidgets.QStyle.SP_MediaStop)
            )

    def _set_status_value(self, key, text, color='#202522'):
        label = self.status_value_labels[key]
        label.setText(text)
        label.setStyleSheet(f'color: {color}; font-weight: 600;')

    def _refresh_live_status(self):
        if not hasattr(self, 'status_value_labels'):
            return
        now = time.monotonic()
        target_side = str(self.side_combo.currentData())
        target_text = self._tr(target_side)
        self._set_status_value('target_hand', target_text)

        image_fresh = (
            self.last_image_time is not None
            and now - self.last_image_time <= 1.0
        )
        if self.connection_state == 'disconnected':
            self._set_status_value('connection', self._tr('not_connected'), '#7a817c')
        elif self.connection_state == 'stopping':
            self._set_status_value('connection', self._tr('disconnecting'), '#8a5a16')
        elif image_fresh:
            self._set_status_value('connection', self._tr('connected'), '#166b4f')
        else:
            self._set_status_value('connection', self._tr('waiting_image'), '#8a5a16')

        while self.image_times and now - self.image_times[0] > 1.0:
            self.image_times.popleft()
        fps = float(len(self.image_times)) if image_fresh else 0.0
        self._set_status_value('image_fps', f'{fps:.1f}')

        pose_fresh = (
            self.last_pose_time is not None
            and now - self.last_pose_time <= 0.75
        )
        pose = self.latest_pose if pose_fresh else None
        if self.connection_state == 'disconnected':
            self._set_status_value('recognized_hand', '—', '#7a817c')
            self._set_status_value('confidence', '—', '#7a817c')
            self._set_status_value(
                'detection', self._tr('not_connected'), '#7a817c'
            )
            if self.sampling_pose is None:
                self._set_live_sample_hint('wait_stable', '#6c756f')
            self._update_sample_controls()
            return
        if pose is None or not pose['detected']:
            self._set_status_value('recognized_hand', '—', '#7a817c')
            self._set_status_value('confidence', '—', '#7a817c')
            self._set_status_value('detection', self._tr('no_hand'), '#a33b32')
            if self.sampling_pose is None:
                self._set_live_sample_hint('ready_no_hand', '#a33b32')
            self._update_sample_controls()
            return

        handedness = pose['handedness']
        recognized_text = (
            self._tr(handedness)
            if handedness in {'left', 'right'}
            else self._tr('unknown')
        )
        self._set_status_value('recognized_hand', recognized_text)
        self._set_status_value('confidence', f'{pose["confidence"]:.2f}')
        stable = (
            self.valid_detection_started is not None
            and now - self.valid_detection_started >= 0.5
        )
        if stable:
            self._set_status_value('detection', self._tr('stable'), '#166b4f')
            if self.sampling_pose is None:
                self._set_live_sample_hint('ready_to_sample', '#166b4f')
        else:
            reason_keys = {
                INVALID_WRONG_HAND: 'ready_wrong_hand',
                INVALID_LOW_CONFIDENCE: 'ready_low_confidence',
                INVALID_HAND_CLIPPED: 'ready_hand_clipped',
                INVALID_INCOMPLETE_ANGLES: 'ready_incomplete_angles',
            }
            reason_key = reason_keys.get(self.latest_pose_invalid_reason)
            status_text = (
                self._tr(reason_key)
                if reason_key
                else self._tr('stabilizing')
            )
            self._set_status_value('detection', status_text, '#8a5a16')
            if self.sampling_pose is None:
                self._set_live_sample_hint(
                    reason_key or 'wait_stable', '#8a5a16'
                )
        self._update_sample_controls()

    def closeEvent(self, event):
        if self.samples_dirty or self.sampling_pose is not None:
            answer = QtWidgets.QMessageBox.question(
                self,
                self._tr('unsaved_title'),
                self._tr('unsaved_close'),
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.No,
            )
            if answer != QtWidgets.QMessageBox.Yes:
                event.ignore()
                return
        self.shutdown()
        event.accept()

    def shutdown(self):
        if self.shutting_down:
            return
        self.shutting_down = True
        self.qt_settings.setValue('window_geometry', self.saveGeometry())
        self.status_timer.stop()
        self.sample_timer.stop()
        self._stop_ros_thread()
        verification = self.verification_process
        if (
            verification is not None
            and verification.state() != QtCore.QProcess.NotRunning
        ):
            self.intentional_verification_stop = True
            self._signal_qprocess_group(verification, signal.SIGINT)
            if not verification.waitForFinished(4000):
                self._signal_qprocess_group(verification, signal.SIGKILL)
                verification.waitForFinished(1500)
        self.verification_process = None
        process = self.perception_process
        if process is not None and process.state() != QtCore.QProcess.NotRunning:
            self.intentional_process_stop = True
            self._signal_process_group(signal.SIGINT)
            if not process.waitForFinished(2500):
                self._signal_process_group(signal.SIGKILL)
                process.waitForFinished(1000)
        self.perception_process = None


def main(args=None):
    raw_args = list(sys.argv if args is None else args)
    qt_args = remove_ros_args(args=raw_args)
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)
    app = QtWidgets.QApplication(qt_args)
    app.setApplicationName('Linker Hand Calibration')
    app.setOrganizationName('linkerhand_gesture_control')

    rclpy.init(args=raw_args)
    window = CalibrationWindow()
    app.aboutToQuit.connect(window.shutdown)
    signal.signal(signal.SIGINT, lambda *_args: app.quit())
    signal_timer = QtCore.QTimer()
    signal_timer.start(200)
    signal_timer.timeout.connect(lambda: None)
    window.show()
    exit_code = app.exec_()
    window.shutdown()
    if rclpy.ok():
        rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
