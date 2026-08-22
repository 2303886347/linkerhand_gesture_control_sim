"""用于关键点和关节角消抖的 One Euro 自适应低通滤波器。"""

import math

import numpy as np


def _smoothing_factor(delta_time, cutoff):
    cutoff = np.maximum(np.asarray(cutoff, dtype=float), 1e-6)
    time_constant = 1.0 / (2.0 * math.pi * cutoff)
    return 1.0 / (1.0 + time_constant / delta_time)


class OneEuroFilter:
    """静止时强力消抖，快速运动时自动减小延迟。"""

    def __init__(self, min_cutoff=0.8, beta=0.3, derivative_cutoff=1.0):
        self.min_cutoff = max(float(min_cutoff), 1e-6)
        self.beta = max(float(beta), 0.0)
        self.derivative_cutoff = max(float(derivative_cutoff), 1e-6)
        self.reset()

    def reset(self):
        self.previous_time = None
        self.previous_raw = None
        self.previous_filtered = None
        self.previous_derivative = None

    def filter(self, value, timestamp):
        value = np.asarray(value, dtype=float)
        timestamp = float(timestamp)

        if self.previous_time is None:
            self.previous_time = timestamp
            self.previous_raw = value.copy()
            self.previous_filtered = value.copy()
            self.previous_derivative = np.zeros_like(value)
            return value.copy()

        delta_time = max(timestamp - self.previous_time, 1e-6)
        raw_derivative = (value - self.previous_raw) / delta_time
        derivative_alpha = _smoothing_factor(
            delta_time, self.derivative_cutoff
        )
        filtered_derivative = (
            derivative_alpha * raw_derivative
            + (1.0 - derivative_alpha) * self.previous_derivative
        )

        cutoff = self.min_cutoff + self.beta * np.abs(filtered_derivative)
        signal_alpha = _smoothing_factor(delta_time, cutoff)
        filtered = (
            signal_alpha * value
            + (1.0 - signal_alpha) * self.previous_filtered
        )

        self.previous_time = timestamp
        self.previous_raw = value.copy()
        self.previous_filtered = filtered.copy()
        self.previous_derivative = filtered_derivative.copy()
        return filtered
