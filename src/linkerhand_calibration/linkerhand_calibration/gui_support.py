"""Qt 标定上位机使用的无界面辅助函数。"""

from dataclasses import dataclass
from glob import glob
from pathlib import Path
import re
import shutil


SUPPORTED_MODELS = ('l30', 'o6')
SUPPORTED_SIDES = ('left', 'right')


@dataclass(frozen=True)
class PerceptionSettings:
    """启动摄像头和 MediaPipe 感知管线所需的参数。"""

    side: str
    device: str
    width: int = 640
    height: int = 480
    camera_fps: float = 30.0
    processing_fps: float = 15.0
    mirror_preview: bool = True
    one_euro_min_cutoff: float = 0.8
    one_euro_beta: float = 0.3


@dataclass(frozen=True)
class VerificationSettings:
    """启动个人配置仿真验证所需的统一参数。"""

    mode: str
    model_id: str
    side: str
    parameters_file: str
    device: str
    width: int = 640
    height: int = 480
    camera_fps: float = 30.0
    processing_fps: float = 15.0
    mirror_preview: bool = True
    one_euro_min_cutoff: float = 0.8
    one_euro_beta: float = 0.3


def normalize_model(model_id):
    """校验并规范化型号名称。"""
    value = str(model_id).strip().lower()
    if value not in SUPPORTED_MODELS:
        raise ValueError(f'不支持的灵巧手型号：{model_id}')
    return value


def normalize_side(side):
    """校验并规范化手侧名称。"""
    value = str(side).strip().lower()
    if value not in SUPPORTED_SIDES:
        raise ValueError(f'不支持的手侧：{side}')
    return value


def calibration_topics(side):
    """返回 GUI 标定会话独占使用的话题名称。"""
    side = normalize_side(side)
    prefix = f'/{side}/calibration'
    return {
        'pose': f'{prefix}/hand_pose',
        'angles': f'{prefix}/human_joint_angles',
        'debug_image': f'{prefix}/debug_image',
    }


def _natural_sort_key(value):
    return tuple(
        int(part) if part.isdecimal() else part.lower()
        for part in re.split(r'(\d+)', value)
    )


def list_video_devices(pattern='/dev/video*'):
    """按自然顺序列出当前存在的 V4L2 设备。"""
    return tuple(sorted(set(glob(pattern)), key=_natural_sort_key))


def build_perception_command(settings):
    """构造不经过 shell 的感知管线命令。"""
    side = normalize_side(settings.side)
    topics = calibration_topics(side)
    setsid = shutil.which('setsid') or 'setsid'
    ros2 = shutil.which('ros2') or 'ros2'
    arguments = [
        ros2,
        'launch',
        'mediapipe_hand_pose',
        'pipeline.launch.py',
        f'device:={str(settings.device).strip()}',
        f'width:={int(settings.width)}',
        f'height:={int(settings.height)}',
        f'camera_fps:={float(settings.camera_fps):g}',
        f'processing_fps:={float(settings.processing_fps):g}',
        f'target_hand:={side}',
        f'pose_topic:={topics["pose"]}',
        f'angles_topic:={topics["angles"]}',
        f'debug_image_topic:={topics["debug_image"]}',
        'camera_show_preview:=false',
        'mediapipe_show_preview:=false',
        'use_one_euro_filter:=true',
        f'mirror_preview:={str(bool(settings.mirror_preview)).lower()}',
        f'one_euro_min_cutoff:={float(settings.one_euro_min_cutoff):g}',
        f'one_euro_beta:={float(settings.one_euro_beta):g}',
    ]
    return setsid, tuple(arguments)


def build_verification_command(settings):
    """构造 RViz/Gazebo 个人配置验证命令，不经过 shell。"""
    mode = str(settings.mode).strip().lower()
    if mode not in {'rviz', 'gazebo'}:
        raise ValueError(f'不支持的验证模式：{settings.mode}')
    model_id = normalize_model(settings.model_id)
    side = normalize_side(settings.side)
    parameters_file = Path(settings.parameters_file).expanduser().resolve()
    if not parameters_file.is_file():
        raise FileNotFoundError(f'个人标定配置不存在：{parameters_file}')
    device = str(settings.device).strip()
    if not device:
        raise ValueError('摄像头设备路径不能为空')

    setsid = shutil.which('setsid') or 'setsid'
    ros2 = shutil.which('ros2') or 'ros2'
    launch_file = (
        'mediapipe_rviz_single.launch.py'
        if mode == 'rviz'
        else 'mediapipe_gazebo.launch.py'
    )
    arguments = [
        ros2,
        'launch',
        'linkerhand_bringup',
        launch_file,
        f'model_id:={model_id}',
        f'side:={side}',
        f'parameters_file:={parameters_file}',
        f'device:={device}',
        f'width:={int(settings.width)}',
        f'height:={int(settings.height)}',
        f'camera_fps:={float(settings.camera_fps):g}',
        f'processing_fps:={float(settings.processing_fps):g}',
        'mediapipe_show_preview:=true',
        f'mirror_preview:={str(bool(settings.mirror_preview)).lower()}',
        'use_one_euro_filter:=true',
        f'one_euro_min_cutoff:={float(settings.one_euro_min_cutoff):g}',
        f'one_euro_beta:={float(settings.one_euro_beta):g}',
    ]
    return setsid, tuple(arguments)
