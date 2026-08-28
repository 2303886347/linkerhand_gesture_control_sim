"""从姿态采样生成可直接用于重定向节点的个人标定配置。"""

from copy import deepcopy
from datetime import datetime, timezone
import math
from pathlib import Path
import statistics

import yaml


POSE_OPEN = 'open_hand'
POSE_FIST = 'closed_fist'
POSE_THUMB_IN = 'thumb_adducted'
POSE_THUMB_OUT = 'thumb_abducted'

POSE_LABELS = {
    POSE_OPEN: '张开手掌并伸直五指',
    POSE_FIST: '自然握拳',
    POSE_THUMB_IN: '保持四指伸直，将拇指收拢到掌侧',
    POSE_THUMB_OUT: '保持四指伸直，将拇指完全展开',
}

PROFILE_SCHEMA_VERSION = 1


def load_parameters(path):
    """读取 ROS 2 参数 YAML 并返回参数映射。"""
    path = Path(path)
    try:
        document = yaml.safe_load(path.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f'无法读取标定模板 {path}：{error}') from error
    try:
        parameters = document['/**']['ros__parameters']
    except (KeyError, TypeError) as error:
        raise ValueError(f'标定模板不是有效的 ROS 2 参数 YAML：{path}') from error
    if not isinstance(parameters, dict) or not isinstance(
        parameters.get('mapping'), dict
    ):
        raise ValueError(f'标定模板缺少 mapping：{path}')
    return document, parameters


def mapping_sources(settings):
    """兼容单 source 和多 sources 两种现有映射格式。"""
    sources = [
        str(value).strip()
        for value in settings.get('sources', [])
        if str(value).strip()
    ]
    if not sources:
        source = str(settings.get('source', '')).strip()
        sources = [source] if source else []

    weights = [float(value) for value in settings.get('source_weights', [])]
    if not weights and sources:
        weights = [1.0] * len(sources)
    if len(weights) != len(sources):
        raise ValueError('mapping 的 sources 与 source_weights 数量不一致')
    if any(not math.isfinite(value) or value < 0.0 for value in weights):
        raise ValueError('mapping 的 source_weights 必须是非负有限数')
    if sources and sum(weights) <= 0.0:
        raise ValueError('mapping 的 source_weights 总和必须大于 0')
    return tuple(sources), tuple(weights)


def combine_mapping_input(settings, angles):
    """按实时重定向节点相同的规则融合 MediaPipe 输入角。"""
    sources, weights = mapping_sources(settings)
    if not sources:
        raise ValueError('固定关节没有可标定的输入角')

    missing = [
        source
        for source in sources
        if source not in angles or not math.isfinite(float(angles[source]))
    ]
    if missing:
        raise KeyError(tuple(missing))
    return sum(
        float(angles[source]) * weight
        for source, weight in zip(sources, weights)
    ) / sum(weights)


def driven_joints(parameters):
    """返回当前型号配置中实际由 MediaPipe 驱动的关节。"""
    return tuple(
        joint
        for joint, settings in parameters['mapping'].items()
        if mapping_sources(settings)[0]
    )


def required_sources(parameters):
    """返回采样消息必须包含的全部人体语义角名称。"""
    result = []
    for settings in parameters['mapping'].values():
        for source in mapping_sources(settings)[0]:
            if source not in result:
                result.append(source)
    return tuple(result)


def summarize_pose_samples(parameters, samples):
    """以中位数汇总一个姿态窗口，降低偶发识别跳变的影响。"""
    if not samples:
        raise ValueError('姿态采样为空')
    summary = {}
    for joint in driven_joints(parameters):
        settings = parameters['mapping'][joint]
        values = [
            combine_mapping_input(settings, sample) for sample in samples
        ]
        summary[joint] = statistics.median(values)
    return summary


def calibration_endpoints(parameters, pose_summaries):
    """返回每个驱动关节使用的两个姿态端点和输入角。"""
    missing_poses = {POSE_OPEN, POSE_FIST} - set(pose_summaries)
    if missing_poses:
        raise ValueError(f'缺少必要姿态：{sorted(missing_poses)}')

    endpoints = {}
    for joint in driven_joints(parameters):
        if joint == 'thumb_cmc_yaw':
            minimum_pose = POSE_THUMB_OUT
            maximum_pose = POSE_THUMB_IN
            if minimum_pose not in pose_summaries or maximum_pose not in pose_summaries:
                raise ValueError('拇指侧摆映射缺少收拢或展开姿态')
        else:
            minimum_pose = POSE_OPEN
            maximum_pose = POSE_FIST
        try:
            input_min = float(pose_summaries[minimum_pose][joint])
            input_max = float(pose_summaries[maximum_pose][joint])
        except KeyError as error:
            raise ValueError(f'姿态结果缺少关节：{joint}') from error
        endpoints[joint] = {
            'minimum_pose': minimum_pose,
            'maximum_pose': maximum_pose,
            'input_min': input_min,
            'input_max': input_max,
            'range': input_max - input_min,
        }
    return endpoints


def validate_calibration_ranges(
    parameters,
    pose_summaries,
    minimum_range_deg=5.0,
):
    """联合检查全部姿态端点，返回端点以及不合格关节。"""
    endpoints = calibration_endpoints(parameters, pose_summaries)
    invalid = []
    for joint, values in endpoints.items():
        input_min = values['input_min']
        input_max = values['input_max']
        if not math.isfinite(input_min) or not math.isfinite(input_max):
            invalid.append({
                'joint': joint,
                'reason': 'non_finite',
                **values,
            })
        elif values['range'] < float(minimum_range_deg):
            invalid.append({
                'joint': joint,
                'reason': 'range_too_small_or_reversed',
                **values,
            })
    return endpoints, tuple(invalid)


def build_personal_calibration(
    template_document,
    pose_summaries,
    minimum_range_deg=5.0,
):
    """保留模板控制参数，仅以个人姿态端点替换映射输入范围。"""
    result = deepcopy(template_document)
    parameters = result['/**']['ros__parameters']
    mapping = parameters['mapping']
    endpoints, invalid_ranges = validate_calibration_ranges(
        parameters,
        pose_summaries,
        minimum_range_deg=minimum_range_deg,
    )
    invalid = []
    for values in invalid_ranges:
        if values['reason'] == 'non_finite':
            invalid.append(f'{values["joint"]}=非有限值')
        else:
            invalid.append(
                f'{values["joint"]}='
                f'{values["input_min"]:.1f}~{values["input_max"]:.1f} 度'
            )

    for joint, values in endpoints.items():
        input_min = values['input_min']
        input_max = values['input_max']
        if any(item['joint'] == joint for item in invalid_ranges):
            continue
        mapping[joint]['input_min'] = round(float(input_min), 2)
        mapping[joint]['input_max'] = round(float(input_max), 2)

    if invalid:
        raise ValueError(
            '以下关节的个人活动范围过小或方向异常，请重新采样：'
            + '；'.join(invalid)
        )
    return result


def utc_timestamp(now=None):
    """生成可稳定写入 YAML 的 UTC ISO 时间。"""
    value = now or datetime.now(timezone.utc)
    if value.tzinfo is None:
        value = value.replace(tzinfo=timezone.utc)
    return value.astimezone(timezone.utc).replace(microsecond=0).isoformat()


def build_profile_metadata(
    profile_name,
    model_id,
    side,
    camera_device,
    pose_results,
    quality_settings,
    created_at='',
    now=None,
):
    """将采样质量与人体角中位数整理为可机读的档案元数据。"""
    updated_at = utc_timestamp(now)
    samples = {}
    for pose, result in pose_results.items():
        samples[pose] = {
            'valid_frames': int(result.valid_frames),
            'total_frames': int(result.total_frames),
            'valid_ratio': round(float(result.valid_ratio), 4),
            'maximum_spread_deg': round(
                max(result.spreads.values(), default=0.0), 3
            ),
            'source_medians_deg': {
                str(name): round(float(value), 3)
                for name, value in result.source_summary.items()
            },
        }
    return {
        'schema_version': PROFILE_SCHEMA_VERSION,
        'name': str(profile_name).strip(),
        'model_id': str(model_id).strip().lower(),
        'side': str(side).strip().lower(),
        'created_at': str(created_at).strip() or updated_at,
        'updated_at': updated_at,
        'camera_device': str(camera_device).strip(),
        'angle_unit': 'deg',
        'quality_settings': {
            str(name): value for name, value in quality_settings.items()
        },
        'samples': samples,
    }


def attach_profile_metadata(document, metadata):
    """把元数据写入独立参数命名空间，保持 ROS 2 文件兼容。"""
    result = deepcopy(document)
    result['/**']['ros__parameters']['calibration_profile'] = deepcopy(metadata)
    return result


def extract_profile_metadata(document):
    """读取个人配置元数据；旧文件没有元数据时返回空映射。"""
    try:
        metadata = document['/**']['ros__parameters'].get(
            'calibration_profile', {}
        )
    except (KeyError, TypeError):
        return {}
    return metadata if isinstance(metadata, dict) else {}


def write_calibration(path, document):
    """建立父目录并原子替换个人标定 YAML。"""
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = path.with_suffix(path.suffix + '.tmp')
    temporary_path.write_text(
        yaml.safe_dump(
            document,
            allow_unicode=True,
            sort_keys=False,
            default_flow_style=False,
        ),
        encoding='utf-8',
    )
    temporary_path.replace(path)
    return path.resolve()
