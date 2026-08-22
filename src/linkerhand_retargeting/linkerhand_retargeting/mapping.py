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

    @property
    def driven(self):
        return bool(self.source_angle)

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
