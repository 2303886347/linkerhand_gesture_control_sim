"""重定向节点使用的消抖、低通和速度限制函数。"""

from collections import deque
import math


class JointDeadbandHysteresis:
    """用启动/停止双阈值抑制单个关节的静止抖动。"""

    def __init__(
        self,
        initial_value,
        start_threshold,
        stop_threshold,
        settle_frames,
    ):
        self.start_threshold = float(start_threshold)
        self.stop_threshold = float(stop_threshold)
        self.settle_frames = int(settle_frames)
        if not math.isfinite(self.start_threshold) or not math.isfinite(
            self.stop_threshold
        ):
            raise ValueError('死区阈值必须是有限数值')
        if self.start_threshold <= 0.0:
            raise ValueError('启动阈值必须大于 0')
        if self.stop_threshold < 0.0:
            raise ValueError('停止阈值不能小于 0')
        if self.stop_threshold >= self.start_threshold:
            raise ValueError('停止阈值必须小于启动阈值')
        if self.settle_frames < 2:
            raise ValueError('稳定帧数必须至少为 2')

        self._settle_window = deque(maxlen=self.settle_frames)
        self.reset(initial_value)

    @property
    def moving(self):
        return self._moving

    @property
    def output(self):
        return self._output

    def reset(self, value):
        """把状态恢复为静止，并以给定角度作为新的锁定中心。"""
        value = float(value)
        if not math.isfinite(value):
            raise ValueError('锁定角度必须是有限数值')
        self._moving = False
        self._output = value
        self._settle_window.clear()

    def update(self, value):
        value = float(value)
        if not math.isfinite(value):
            raise ValueError('关节角度必须是有限数值')
        if not self._moving:
            if abs(value - self._output) < self.start_threshold:
                return self._output
            self._moving = True
            self._settle_window.clear()

        self._output = value
        self._settle_window.append(value)
        if (
            len(self._settle_window) == self.settle_frames
            and max(self._settle_window) - min(self._settle_window)
            <= self.stop_threshold
        ):
            self._output = sum(self._settle_window) / len(self._settle_window)
            self._moving = False
            self._settle_window.clear()
        return self._output


def exponential_moving_average(previous, current, alpha):
    alpha = min(max(alpha, 0.0), 1.0)
    return alpha * current + (1.0 - alpha) * previous


def move_towards(current, target, maximum_step):
    if maximum_step <= 0.0:
        return target
    delta = target - current
    if abs(delta) <= maximum_step:
        return target
    return current + maximum_step * (1.0 if delta > 0.0 else -1.0)
