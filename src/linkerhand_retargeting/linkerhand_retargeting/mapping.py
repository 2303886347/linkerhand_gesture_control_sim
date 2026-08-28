"""人体角度到机械手关节角的线性映射。"""

from dataclasses import dataclass
import math


def clamp(value, lower, upper):
    return min(max(value, lower), upper)


def angle_to_radians(value, unit):
    normalized_unit = str(unit).strip().lower()
    if normalized_unit == 'rad':
        return float(value)
    if normalized_unit == 'deg':
        return math.radians(float(value))
    raise ValueError(f'不支持的角度单位：{unit}；只能使用 deg 或 rad')


def radians_to_angle(value, unit):
    normalized_unit = str(unit).strip().lower()
    if normalized_unit == 'rad':
        return float(value)
    if normalized_unit == 'deg':
        return math.degrees(float(value))
    raise ValueError(f'不支持的角度单位：{unit}；只能使用 deg 或 rad')


@dataclass(frozen=True)
class JointMapping:
    target_joint: str
    source_angle: str
    input_min: float
    input_max: float
    output_min: float
    output_max: float
    joint_min: float
    joint_max: float
    invert: bool = False
    fixed_position: float = 0.0
    source_angles: tuple = ()
    source_weights: tuple = ()

    def __post_init__(self):
        sources = tuple(
            str(source).strip() for source in self.source_angles if str(source).strip()
        )
        if not sources and self.source_angle:
            sources = (str(self.source_angle).strip(),)
        if len(sources) != len(set(sources)):
            raise ValueError(f'{self.target_joint} 的输入角度包含重复名称')

        weights = tuple(float(weight) for weight in self.source_weights)
        if not weights and sources:
            weights = (1.0,) * len(sources)
        if len(weights) != len(sources):
            raise ValueError(f'{self.target_joint} 的输入角度与权重数量不一致')
        if any(not math.isfinite(weight) or weight < 0.0 for weight in weights):
            raise ValueError(f'{self.target_joint} 的输入权重必须是非负有限数')
        if sources and sum(weights) <= 0.0:
            raise ValueError(f'{self.target_joint} 的输入权重总和必须大于 0')

        object.__setattr__(self, 'source_angles', sources)
        object.__setattr__(self, 'source_weights', weights)
        object.__setattr__(self, 'source_angle', sources[0] if sources else '')

    @property
    def driven(self):
        return bool(self.source_angles)

    def combine_source_angles(self, available_angles):
        """按配置权重融合一个或多个 MediaPipe 人体角度。"""
        if not self.driven:
            return self.input_min

        missing = []
        values = []
        for source in self.source_angles:
            value = available_angles.get(source)
            if value is None or not math.isfinite(value):
                missing.append(source)
            else:
                values.append(float(value))
        if missing:
            raise KeyError(tuple(missing))

        weight_sum = sum(self.source_weights)
        return sum(
            value * weight
            for value, weight in zip(values, self.source_weights)
        ) / weight_sum

    def map_angle(self, source_value):
        if not self.driven:
            return clamp(self.fixed_position, self.joint_min, self.joint_max)
        if self.input_max <= self.input_min:
            raise ValueError(f'{self.target_joint} 的输入角度范围无效')

        normalized = (source_value - self.input_min) / (
            self.input_max - self.input_min
        )
        normalized = clamp(normalized, 0.0, 1.0)
        if self.invert:
            normalized = 1.0 - normalized

        output = self.output_min + normalized * (
            self.output_max - self.output_min
        )
        return clamp(output, self.joint_min, self.joint_max)
