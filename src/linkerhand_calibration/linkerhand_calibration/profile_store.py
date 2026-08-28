"""个人标定配置的命名、扫描与外部档案校验。"""

from dataclasses import dataclass
import hashlib
from pathlib import Path
import re

from linkerhand_calibration.calibration import (
    extract_profile_metadata,
    load_parameters,
)


DEFAULT_PROFILE_DIRECTORY = (
    Path.home() / '.config' / 'linkerhand_gesture_control' / 'calibration'
)


@dataclass(frozen=True)
class CalibrationProfile:
    """GUI 中显示的一份可用标定配置。"""

    path: Path
    name: str
    model_id: str
    side: str
    created_at: str = ''
    updated_at: str = ''
    external: bool = False
    legacy: bool = False


def validate_profile_name(name):
    """校验用户可见的配置名称。"""
    value = str(name).strip()
    if not value:
        raise ValueError('配置名称不能为空')
    if len(value) > 64:
        raise ValueError('配置名称不能超过 64 个字符')
    if any(character in value for character in ('/', '\\', '\0')):
        raise ValueError('配置名称不能包含路径分隔符')
    return value


def profile_filename(model_id, side, name):
    """生成跨平台稳定的个人配置文件名。"""
    name = validate_profile_name(name)
    ascii_slug = re.sub(r'[^A-Za-z0-9_-]+', '_', name).strip('_').lower()
    if not ascii_slug:
        digest = hashlib.sha1(name.encode('utf-8')).hexdigest()[:8]
        ascii_slug = f'profile_{digest}'
    return f'{str(model_id).lower()}_{str(side).lower()}_{ascii_slug}.yaml'


def unique_profile_path(directory, model_id, side, name):
    """在默认目录中生成不覆盖现有文件的路径。"""
    directory = Path(directory).expanduser()
    candidate = directory / profile_filename(model_id, side, name)
    if not candidate.exists():
        return candidate
    stem = candidate.stem
    index = 2
    while True:
        alternative = candidate.with_name(f'{stem}_{index}.yaml')
        if not alternative.exists():
            return alternative
        index += 1


def load_profile(path, external=False):
    """校验 YAML 并读取可供 GUI 使用的档案摘要。"""
    path = Path(path).expanduser().resolve()
    document, parameters = load_parameters(path)
    model_id = str(parameters.get('model_id', '')).strip().lower()
    side = str(parameters.get('model_side', '')).strip().lower()
    if model_id not in {'l30', 'o6'} or side not in {'left', 'right'}:
        raise ValueError(f'配置缺少有效的 model_id/model_side：{path}')
    metadata = extract_profile_metadata(document)
    metadata_model = str(metadata.get('model_id', model_id)).lower()
    metadata_side = str(metadata.get('side', side)).lower()
    if metadata_model != model_id or metadata_side != side:
        raise ValueError(f'配置元数据与 ROS 参数中的型号或手侧不一致：{path}')
    name = str(metadata.get('name', '')).strip() or path.stem
    return CalibrationProfile(
        path=path,
        name=name,
        model_id=model_id,
        side=side,
        created_at=str(metadata.get('created_at', '')),
        updated_at=str(metadata.get('updated_at', '')),
        external=bool(external),
        legacy=not bool(metadata),
    )


def scan_profiles(
    model_id,
    side,
    directory=DEFAULT_PROFILE_DIRECTORY,
    external_paths=(),
):
    """扫描默认目录和用户登记的外部 YAML，忽略损坏或不匹配文件。"""
    model_id = str(model_id).strip().lower()
    side = str(side).strip().lower()
    candidates = []
    directory = Path(directory).expanduser()
    if directory.exists():
        candidates.extend((path, False) for path in directory.glob('*.yaml'))
    candidates.extend((Path(path).expanduser(), True) for path in external_paths)

    profiles = []
    errors = []
    seen = set()
    for path, external in candidates:
        try:
            resolved = path.resolve()
        except OSError:
            resolved = path
        if resolved in seen:
            continue
        seen.add(resolved)
        try:
            profile = load_profile(resolved, external=external)
        except (OSError, ValueError) as error:
            errors.append((resolved, str(error)))
            continue
        if profile.model_id == model_id and profile.side == side:
            profiles.append(profile)
    profiles.sort(key=lambda item: (item.name.lower(), str(item.path)))
    return tuple(profiles), tuple(errors)
