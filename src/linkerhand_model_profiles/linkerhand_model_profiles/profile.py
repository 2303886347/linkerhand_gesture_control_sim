"""从 YAML 和 URDF 构造经过严格校验的机械手型号 profile。"""

from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
import yaml


SUPPORTED_SCHEMA_VERSION = 1
SUPPORTED_SIDES = {'left', 'right'}


class ProfileError(ValueError):
    """型号配置存在格式或 URDF 一致性错误。"""


class ProfileNotFoundError(ProfileError):
    """请求的型号配置或描述包不存在。"""


@dataclass(frozen=True)
class MimicJoint:
    source: str
    multiplier: float = 1.0
    offset: float = 0.0


@dataclass(frozen=True)
class MappingDefaults:
    source_angle: str
    input_min: float
    input_max: float
    output_min: float
    output_max: float
    invert: bool
    fixed_position: float
    safe_position: float


@dataclass(frozen=True)
class ModelProfile:
    schema_version: int
    model_id: str
    side: str
    description_package: str
    urdf_path: Path
    robot_name: str
    root_link: str
    world_joint_name: str
    active_joints: tuple
    thumb_joints: tuple
    full_joints: tuple
    mimic_joints: dict
    locked_joints: dict
    joint_limits: dict
    mapping_defaults: dict
    gazebo_inertial_scale: float

    @property
    def controlled_joints(self):
        """Gazebo 和完整 JointState 使用的确定性关节顺序。"""
        return self.full_joints


def _read_yaml(path):
    path = Path(path)
    try:
        data = yaml.safe_load(path.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as error:
        raise ProfileError(f'无法读取型号配置 {path}: {error}') from error
    if not isinstance(data, dict):
        raise ProfileError(f'型号配置必须是 YAML 映射：{path}')
    return data


def _require(mapping, key, context):
    if key not in mapping:
        raise ProfileError(f'{context} 缺少字段：{key}')
    return mapping[key]


def _as_unique_tuple(value, field_name):
    if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
        raise ProfileError(f'{field_name} 必须是字符串列表')
    result = tuple(value)
    if len(result) != len(set(result)):
        raise ProfileError(f'{field_name} 包含重复关节')
    return result


def _as_finite_float(value, field_name):
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ProfileError(f'{field_name} 必须是数字') from error
    if not math.isfinite(result):
        raise ProfileError(f'{field_name} 必须是有限数字')
    return result


def _parse_mimic_joints(raw):
    if not isinstance(raw, dict):
        raise ProfileError('mimic_joints 必须是映射')
    result = {}
    for joint, settings in raw.items():
        if not isinstance(joint, str) or not isinstance(settings, dict):
            raise ProfileError('mimic_joints 的名称和配置格式无效')
        result[joint] = MimicJoint(
            source=str(_require(settings, 'source', f'mimic_joints.{joint}')),
            multiplier=_as_finite_float(
                settings.get('multiplier', 1.0),
                f'mimic_joints.{joint}.multiplier',
            ),
            offset=_as_finite_float(
                settings.get('offset', 0.0),
                f'mimic_joints.{joint}.offset',
            ),
        )
    return result


def _parse_locked_joints(raw):
    if not isinstance(raw, dict):
        raise ProfileError('locked_joints 必须是映射')
    return {
        str(joint): _as_finite_float(value, f'locked_joints.{joint}')
        for joint, value in raw.items()
    }


def _parse_mapping_defaults(raw, active_joints):
    if not isinstance(raw, dict):
        raise ProfileError('mapping_defaults 必须是映射')
    if set(raw) != set(active_joints):
        missing = sorted(set(active_joints) - set(raw))
        extra = sorted(set(raw) - set(active_joints))
        raise ProfileError(
            f'mapping_defaults 与主动关节不一致；缺少={missing}，多余={extra}'
        )

    result = {}
    for joint in active_joints:
        settings = raw[joint]
        if not isinstance(settings, dict):
            raise ProfileError(f'mapping_defaults.{joint} 必须是映射')
        result[joint] = MappingDefaults(
            source_angle=str(settings.get('source', '')),
            input_min=_as_finite_float(
                _require(settings, 'input_min_rad', f'mapping_defaults.{joint}'),
                f'mapping_defaults.{joint}.input_min_rad',
            ),
            input_max=_as_finite_float(
                _require(settings, 'input_max_rad', f'mapping_defaults.{joint}'),
                f'mapping_defaults.{joint}.input_max_rad',
            ),
            output_min=_as_finite_float(
                _require(settings, 'output_min_rad', f'mapping_defaults.{joint}'),
                f'mapping_defaults.{joint}.output_min_rad',
            ),
            output_max=_as_finite_float(
                _require(settings, 'output_max_rad', f'mapping_defaults.{joint}'),
                f'mapping_defaults.{joint}.output_max_rad',
            ),
            invert=bool(settings.get('invert', False)),
            fixed_position=_as_finite_float(
                settings.get('fixed_position_rad', 0.0),
                f'mapping_defaults.{joint}.fixed_position_rad',
            ),
            safe_position=_as_finite_float(
                settings.get('safe_position_rad', 0.0),
                f'mapping_defaults.{joint}.safe_position_rad',
            ),
        )
    return result


def _parse_urdf(urdf_path):
    try:
        robot = ET.parse(urdf_path).getroot()
    except (OSError, ET.ParseError) as error:
        raise ProfileError(f'无法解析 URDF {urdf_path}: {error}') from error
    if robot.tag != 'robot':
        raise ProfileError(f'URDF 根元素不是 robot：{urdf_path}')
    return robot


def _joint_limits(joint, joint_name):
    limit = joint.find('limit')
    if limit is None or limit.get('lower') is None or limit.get('upper') is None:
        raise ProfileError(f'关节 {joint_name} 缺少上下限')
    lower = _as_finite_float(limit.get('lower'), f'{joint_name}.limit.lower')
    upper = _as_finite_float(limit.get('upper'), f'{joint_name}.limit.upper')
    if lower > upper:
        raise ProfileError(f'关节 {joint_name} 下限大于上限')
    return lower, upper


def _validate_joint_sets(active, mimic, locked, full):
    active_set = set(active)
    mimic_set = set(mimic)
    locked_set = set(locked)
    if active_set & mimic_set or active_set & locked_set or mimic_set & locked_set:
        raise ProfileError('active、mimic 和 locked 关节集合必须互斥')
    expected = active_set | mimic_set | locked_set
    if set(full) != expected or len(full) != len(expected):
        raise ProfileError('full_joints 必须恰好包含 active、mimic 和 locked 关节')


def _validate_profile_against_urdf(
    robot,
    root_link,
    active_joints,
    full_joints,
    mimic_joints,
    locked_joints,
    mapping_defaults,
):
    links = {link.get('name') for link in robot.findall('link')}
    if root_link not in links:
        raise ProfileError(f'URDF 中不存在根 link：{root_link}')

    joints = {joint.get('name'): joint for joint in robot.findall('joint')}
    joint_limits = {}
    for joint_name in full_joints:
        joint = joints.get(joint_name)
        if joint is None:
            raise ProfileError(f'URDF 中不存在关节：{joint_name}')
        if joint.get('type') not in {'revolute', 'continuous'}:
            raise ProfileError(f'关节 {joint_name} 不是可旋转关节')
        joint_limits[joint_name] = _joint_limits(joint, joint_name)

    for joint_name in active_joints:
        if joints[joint_name].find('mimic') is not None:
            raise ProfileError(f'主动关节 {joint_name} 不应包含 mimic 标签')

    for joint_name, expected in mimic_joints.items():
        mimic = joints[joint_name].find('mimic')
        if mimic is None:
            raise ProfileError(f'从动关节 {joint_name} 缺少 mimic 标签')
        source = mimic.get('joint', '')
        multiplier = _as_finite_float(
            mimic.get('multiplier', 1.0), f'{joint_name}.mimic.multiplier'
        )
        offset = _as_finite_float(
            mimic.get('offset', 0.0), f'{joint_name}.mimic.offset'
        )
        if source != expected.source:
            raise ProfileError(
                f'关节 {joint_name} mimic 源不一致：{source} != {expected.source}'
            )
        if not math.isclose(multiplier, expected.multiplier, abs_tol=1.0e-9):
            raise ProfileError(f'关节 {joint_name} mimic multiplier 与 URDF 不一致')
        if not math.isclose(offset, expected.offset, abs_tol=1.0e-9):
            raise ProfileError(f'关节 {joint_name} mimic offset 与 URDF 不一致')

    for joint_name, value in locked_joints.items():
        lower, upper = joint_limits[joint_name]
        if not lower <= value <= upper:
            raise ProfileError(f'锁定关节 {joint_name} 的目标超出 URDF 限位')

    for joint_name, defaults in mapping_defaults.items():
        lower, upper = joint_limits[joint_name]
        for field_name, value in (
            ('output_min', defaults.output_min),
            ('output_max', defaults.output_max),
            ('fixed_position', defaults.fixed_position),
            ('safe_position', defaults.safe_position),
        ):
            if not lower <= value <= upper:
                raise ProfileError(
                    f'mapping_defaults.{joint_name}.{field_name} 超出 URDF 限位'
                )

    return joint_limits


def load_model_profile_from_files(model_path, side_path, urdf_path):
    """从明确文件加载 profile，供测试和高级集成使用。"""
    model_data = _read_yaml(model_path)
    side_data = _read_yaml(side_path)

    schema_version = int(_require(model_data, 'schema_version', 'model profile'))
    if schema_version != SUPPORTED_SCHEMA_VERSION:
        raise ProfileError(f'不支持的 profile schema：{schema_version}')
    if int(_require(side_data, 'schema_version', 'side profile')) != schema_version:
        raise ProfileError('型号公共配置和手侧配置的 schema_version 不一致')

    model_id = str(_require(model_data, 'model_id', 'model profile')).lower()
    if str(_require(side_data, 'model_id', 'side profile')).lower() != model_id:
        raise ProfileError('型号公共配置和手侧配置的 model_id 不一致')
    side = str(_require(side_data, 'side', 'side profile')).lower()
    if side not in SUPPORTED_SIDES:
        raise ProfileError(f'不支持的手侧：{side}')

    active_joints = _as_unique_tuple(
        _require(model_data, 'active_joints', 'model profile'), 'active_joints'
    )
    thumb_joints = _as_unique_tuple(
        _require(model_data, 'thumb_joints', 'model profile'), 'thumb_joints'
    )
    full_joints = _as_unique_tuple(
        _require(model_data, 'full_joints', 'model profile'), 'full_joints'
    )
    mimic_joints = _parse_mimic_joints(
        _require(model_data, 'mimic_joints', 'model profile')
    )
    locked_joints = _parse_locked_joints(
        _require(model_data, 'locked_joints', 'model profile')
    )
    _validate_joint_sets(active_joints, mimic_joints, locked_joints, full_joints)
    if not set(thumb_joints) <= set(active_joints):
        raise ProfileError('thumb_joints 必须是 active_joints 的子集')

    mapping_defaults = _parse_mapping_defaults(
        _require(model_data, 'mapping_defaults', 'model profile'), active_joints
    )
    robot = _parse_urdf(urdf_path)
    root_link = str(_require(side_data, 'root_link', 'side profile'))
    joint_limits = _validate_profile_against_urdf(
        robot,
        root_link,
        active_joints,
        full_joints,
        mimic_joints,
        locked_joints,
        mapping_defaults,
    )

    robot_name = str(_require(side_data, 'robot_name', 'side profile'))
    if robot.get('name') != robot_name:
        raise ProfileError(
            f'URDF robot 名称不一致：{robot.get("name")} != {robot_name}'
        )
    gazebo = _require(side_data, 'gazebo', 'side profile')
    if not isinstance(gazebo, dict):
        raise ProfileError('gazebo 必须是映射')
    inertial_scale = _as_finite_float(
        _require(gazebo, 'inertial_scale', 'gazebo'), 'gazebo.inertial_scale'
    )
    if inertial_scale <= 0.0:
        raise ProfileError('gazebo.inertial_scale 必须大于 0')

    return ModelProfile(
        schema_version=schema_version,
        model_id=model_id,
        side=side,
        description_package=str(
            _require(side_data, 'description_package', 'side profile')
        ),
        urdf_path=Path(urdf_path),
        robot_name=robot_name,
        root_link=root_link,
        world_joint_name=str(
            _require(side_data, 'world_joint_name', 'side profile')
        ),
        active_joints=active_joints,
        thumb_joints=thumb_joints,
        full_joints=full_joints,
        mimic_joints=mimic_joints,
        locked_joints=locked_joints,
        joint_limits=joint_limits,
        mapping_defaults=mapping_defaults,
        gazebo_inertial_scale=inertial_scale,
    )


def load_model_profile(model_id='l30', side='left'):
    """从已安装的 profile 和描述包加载指定型号。"""
    model_id = str(model_id).strip().lower()
    side = str(side).strip().lower()
    if side not in SUPPORTED_SIDES:
        raise ProfileError(f'不支持的手侧：{side}')

    try:
        profile_share = Path(
            get_package_share_directory('linkerhand_model_profiles')
        )
    except PackageNotFoundError as error:
        raise ProfileNotFoundError('找不到 linkerhand_model_profiles 包') from error

    profile_dir = profile_share / 'config' / model_id
    model_path = profile_dir / 'model.yaml'
    side_path = profile_dir / f'{side}.yaml'
    if not model_path.is_file() or not side_path.is_file():
        raise ProfileNotFoundError(f'未注册型号 profile：{model_id}/{side}')

    side_data = _read_yaml(side_path)
    description_package = str(
        _require(side_data, 'description_package', 'side profile')
    )
    try:
        description_share = Path(get_package_share_directory(description_package))
    except PackageNotFoundError as error:
        raise ProfileNotFoundError(
            f'找不到型号描述包：{description_package}'
        ) from error
    urdf_path = description_share / str(
        _require(side_data, 'urdf', 'side profile')
    )
    if not urdf_path.is_file():
        raise ProfileNotFoundError(f'找不到型号 URDF：{urdf_path}')
    return load_model_profile_from_files(model_path, side_path, urdf_path)


def expand_joint_positions(profile, positions):
    """把主动关节目标展开为包含 mimic 和 locked 的完整关节字典。"""
    expanded = dict(positions)
    for joint, settings in profile.mimic_joints.items():
        source_value = expanded.get(settings.source, 0.0)
        value = source_value * settings.multiplier + settings.offset
        lower, upper = profile.joint_limits[joint]
        expanded[joint] = min(max(value, lower), upper)
    expanded.update(profile.locked_joints)
    return expanded
