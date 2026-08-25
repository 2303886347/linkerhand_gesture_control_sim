"""将 MediaPipe 三维关键点转换为人体语义关节角。"""

import math

import numpy as np


_EPSILON = 1.0e-8


def _normalize(vector):
    norm = float(np.linalg.norm(vector))
    if norm < _EPSILON:
        raise ValueError('关键点重合，无法计算方向向量')
    return vector / norm


def _project_to_plane(vector, normal):
    return vector - np.dot(vector, normal) * normal


def angle_between(first, second):
    """计算两个向量的夹角，返回范围为 0 到 pi。"""
    first_unit = _normalize(np.asarray(first, dtype=float))
    second_unit = _normalize(np.asarray(second, dtype=float))
    cosine = float(np.clip(np.dot(first_unit, second_unit), -1.0, 1.0))
    return math.acos(cosine)


def flexion_angle(parent, joint, child):
    """计算关节屈曲角；三点共线伸直时为 0。"""
    parent_vector = np.asarray(parent, dtype=float) - np.asarray(joint, dtype=float)
    child_vector = np.asarray(child, dtype=float) - np.asarray(joint, dtype=float)
    return max(0.0, math.pi - angle_between(parent_vector, child_vector))


def _palm_basis(points):
    """构造与左右手无关的手掌前向、拇指侧向和法向坐标轴。"""
    wrist = points[0]
    index_mcp = points[5]
    middle_mcp = points[9]
    pinky_mcp = points[17]

    # 侧向轴始终从小指一侧指向食指和拇指一侧。
    lateral = _normalize(index_mcp - pinky_mcp)
    forward_raw = middle_mcp - wrist
    forward = _normalize(forward_raw - np.dot(forward_raw, lateral) * lateral)
    normal = _normalize(np.cross(lateral, forward))
    return forward, lateral, normal


def _finger_angles(points, indices, forward, lateral, normal):
    mcp, pip, dip, tip = (points[index] for index in indices)
    proximal = _normalize(pip - mcp)
    proximal_in_palm = _project_to_plane(proximal, normal)

    # MCP 侧摆以手掌前向为零，朝拇指一侧为正。
    abduction = math.atan2(
        float(np.dot(proximal_in_palm, lateral)),
        float(np.dot(proximal_in_palm, forward)),
    )
    # 单目估计容易发生掌面法向翻转，因此使用无符号屈曲量保证输出稳定。
    flexion = math.atan2(
        abs(float(np.dot(proximal, normal))),
        float(np.linalg.norm(proximal_in_palm)),
    )

    return abduction, flexion, flexion_angle(mcp, pip, dip), flexion_angle(pip, dip, tip)


def calculate_joint_angles(landmarks):
    """根据 21 个 MediaPipe 关键点计算 20 个人体语义关节角。"""
    points = np.asarray(landmarks, dtype=float)
    if points.shape != (21, 3):
        raise ValueError(f'期望关键点形状为 (21, 3)，实际为 {points.shape}')

    forward, lateral, normal = _palm_basis(points)
    angles = {}

    finger_indices = {
        'index': (5, 6, 7, 8),
        'middle': (9, 10, 11, 12),
        'ring': (13, 14, 15, 16),
        'pinky': (17, 18, 19, 20),
    }
    for finger, indices in finger_indices.items():
        abduction, mcp_flexion, pip_flexion, dip_flexion = _finger_angles(
            points, indices, forward, lateral, normal
        )
        angles[f'{finger}_mcp_abduction'] = abduction
        angles[f'{finger}_mcp_flexion'] = mcp_flexion
        angles[f'{finger}_pip_flexion'] = pip_flexion
        angles[f'{finger}_dip_flexion'] = dip_flexion

    thumb_cmc, thumb_mcp, thumb_ip, thumb_tip = points[1:5]
    thumb_proximal = _normalize(thumb_mcp - thumb_cmc)
    thumb_in_palm = _project_to_plane(thumb_proximal, normal)
    angles['thumb_cmc_abduction'] = math.atan2(
        abs(float(np.dot(thumb_proximal, normal))),
        float(np.linalg.norm(thumb_in_palm)),
    )
    angles['thumb_cmc_flexion'] = math.atan2(
        float(np.dot(thumb_in_palm, forward)),
        float(np.dot(thumb_in_palm, lateral)),
    )
    angles['thumb_mcp_flexion'] = flexion_angle(thumb_cmc, thumb_mcp, thumb_ip)
    angles['thumb_ip_flexion'] = flexion_angle(thumb_mcp, thumb_ip, thumb_tip)

    return angles
