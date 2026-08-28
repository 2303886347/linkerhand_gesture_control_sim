"""个人标定姿态的帧校验、稳定度统计和采样结果。"""

from collections import Counter
from dataclasses import dataclass
import math
import statistics

from linkerhand_calibration.calibration import (
    combine_mapping_input,
    driven_joints,
    required_sources,
    summarize_pose_samples,
)


INVALID_NO_HAND = 'no_hand'
INVALID_WRONG_HAND = 'wrong_hand'
INVALID_LOW_CONFIDENCE = 'low_confidence'
INVALID_HAND_CLIPPED = 'hand_clipped'
INVALID_INCOMPLETE_ANGLES = 'incomplete_angles'

FAIL_NO_MESSAGES = 'no_messages'
FAIL_INSUFFICIENT_SAMPLES = 'insufficient_samples'
FAIL_LOW_VALID_RATIO = 'low_valid_ratio'
FAIL_UNSTABLE = 'unstable'


@dataclass(frozen=True)
class PoseSamplingResult:
    """一次姿态采样完成后的质量统计。"""

    success: bool
    summary: dict
    source_summary: dict
    spreads: dict
    valid_frames: int
    total_frames: int
    valid_ratio: float
    reason_code: str = ''
    reason_detail: str = ''
    dominant_invalid_reason: str = ''


def percentile(values, percentage):
    """使用线性插值计算百分位数。"""
    ordered = sorted(float(value) for value in values)
    if not ordered:
        raise ValueError('无法计算空序列的百分位数')
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * float(percentage) / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def extract_valid_sample(
    parameters,
    pose,
    expected_side,
    confidence_threshold=0.5,
):
    """校验一条 HandPose 快照并返回全部有限角度的度数映射。"""
    if not bool(pose.get('detected', False)):
        return None, INVALID_NO_HAND
    if str(pose.get('handedness', '')).lower() != str(expected_side).lower():
        return None, INVALID_WRONG_HAND
    confidence = float(pose.get('confidence', 0.0))
    if not math.isfinite(confidence) or confidence < float(confidence_threshold):
        return None, INVALID_LOW_CONFIDENCE
    if not bool(pose.get('landmarks_visible', False)):
        return None, INVALID_HAND_CLIPPED

    names = tuple(pose.get('joint_names', ()))
    angles = tuple(pose.get('joint_angles', ()))
    if len(names) != len(angles):
        return None, INVALID_INCOMPLETE_ANGLES
    radians_by_name = dict(zip(names, angles))
    required = required_sources(parameters)
    if any(
        name not in radians_by_name
        or not math.isfinite(float(radians_by_name[name]))
        for name in required
    ):
        return None, INVALID_INCOMPLETE_ANGLES

    degrees_by_name = {
        str(name): math.degrees(float(value))
        for name, value in radians_by_name.items()
        if math.isfinite(float(value))
    }
    return degrees_by_name, ''


def mapping_spreads(parameters, samples):
    """计算每个驱动关节在采样窗口内的 P90-P10 波动。"""
    result = {}
    for joint in driven_joints(parameters):
        settings = parameters['mapping'][joint]
        values = [combine_mapping_input(settings, sample) for sample in samples]
        result[joint] = percentile(values, 90.0) - percentile(values, 10.0)
    return result


def source_medians(samples):
    """汇总所有帧都包含的人体语义角，供后续配置元数据使用。"""
    if not samples:
        return {}
    common_names = set(samples[0])
    for sample in samples[1:]:
        common_names.intersection_update(sample)
    return {
        name: statistics.median(float(sample[name]) for sample in samples)
        for name in sorted(common_names)
    }


def evaluate_pose_samples(
    parameters,
    samples,
    total_frames,
    invalid_counts=None,
    minimum_samples=15,
    minimum_valid_ratio=0.70,
    maximum_spread_deg=8.0,
):
    """按有效帧数、有效率和姿态波动评估一次采样。"""
    invalid_counts = Counter(invalid_counts or {})
    valid_frames = len(samples)
    total_frames = max(int(total_frames), 0)
    valid_ratio = valid_frames / total_frames if total_frames else 0.0
    dominant_reason = (
        invalid_counts.most_common(1)[0][0] if invalid_counts else ''
    )
    base = {
        'summary': {},
        'source_summary': {},
        'spreads': {},
        'valid_frames': valid_frames,
        'total_frames': total_frames,
        'valid_ratio': valid_ratio,
        'dominant_invalid_reason': dominant_reason,
    }

    if total_frames == 0:
        return PoseSamplingResult(
            success=False,
            reason_code=FAIL_NO_MESSAGES,
            **base,
        )
    if valid_frames < max(int(minimum_samples), 1):
        return PoseSamplingResult(
            success=False,
            reason_code=FAIL_INSUFFICIENT_SAMPLES,
            reason_detail=str(max(int(minimum_samples), 1)),
            **base,
        )
    if valid_ratio < float(minimum_valid_ratio):
        return PoseSamplingResult(
            success=False,
            reason_code=FAIL_LOW_VALID_RATIO,
            reason_detail=f'{float(minimum_valid_ratio):.4f}',
            **base,
        )

    summary = summarize_pose_samples(parameters, samples)
    spreads = mapping_spreads(parameters, samples)
    worst_joint = max(spreads, key=spreads.get)
    if spreads[worst_joint] > float(maximum_spread_deg):
        return PoseSamplingResult(
            success=False,
            summary=summary,
            source_summary=source_medians(samples),
            spreads=spreads,
            valid_frames=valid_frames,
            total_frames=total_frames,
            valid_ratio=valid_ratio,
            reason_code=FAIL_UNSTABLE,
            reason_detail=worst_joint,
            dominant_invalid_reason=dominant_reason,
        )
    return PoseSamplingResult(
        success=True,
        summary=summary,
        source_summary=source_medians(samples),
        spreads=spreads,
        valid_frames=valid_frames,
        total_frames=total_frames,
        valid_ratio=valid_ratio,
        dominant_invalid_reason=dominant_reason,
    )


class PoseSampleCollector:
    """收集固定时间窗口内的 HandPose 消息并生成质量结果。"""

    def __init__(
        self,
        parameters,
        expected_side,
        confidence_threshold=0.5,
    ):
        self.parameters = parameters
        self.expected_side = str(expected_side).lower()
        self.confidence_threshold = float(confidence_threshold)
        self.samples = []
        self.total_frames = 0
        self.invalid_counts = Counter()

    def add_pose(self, pose):
        self.total_frames += 1
        sample, reason = extract_valid_sample(
            self.parameters,
            pose,
            self.expected_side,
            self.confidence_threshold,
        )
        if sample is None:
            self.invalid_counts[reason] += 1
            return False
        self.samples.append(sample)
        return True

    def finish(
        self,
        minimum_samples=15,
        minimum_valid_ratio=0.70,
        maximum_spread_deg=8.0,
    ):
        return evaluate_pose_samples(
            self.parameters,
            self.samples,
            self.total_frames,
            self.invalid_counts,
            minimum_samples=minimum_samples,
            minimum_valid_ratio=minimum_valid_ratio,
            maximum_spread_deg=maximum_spread_deg,
        )
