"""重定向节点使用的低通和速度限制函数。"""


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
