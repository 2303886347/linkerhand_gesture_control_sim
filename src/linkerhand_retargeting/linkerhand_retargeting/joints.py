"""Linker Hand L30 左手的独立关节、限位和 RViz 联动关系。"""


INDEPENDENT_JOINTS = (
    'wrist_pitch',
    'pinky_mcp_roll',
    'pinky_mcp_pitch',
    'pinky_pip',
    'ring_mcp_roll',
    'ring_mcp_pitch',
    'ring_pip',
    'middle_mcp_roll',
    'middle_mcp_pitch',
    'middle_pip',
    'index_mcp_roll',
    'index_mcp_pitch',
    'index_pip',
    'thumb_cmc_yaw',
    'thumb_cmc_pitch',
    'thumb_mcp',
    'thumb_dip',
)


JOINT_LIMITS = {
    'wrist_pitch': (-1.05, 1.05),
    'pinky_mcp_roll': (-0.26, 0.26),
    'pinky_mcp_pitch': (0.0, 1.57),
    'pinky_pip': (0.0, 1.57),
    'ring_mcp_roll': (-0.26, 0.26),
    'ring_mcp_pitch': (0.0, 1.57),
    'ring_pip': (0.0, 1.57),
    'middle_mcp_roll': (-0.26, 0.26),
    'middle_mcp_pitch': (0.0, 1.57),
    'middle_pip': (0.0, 1.57),
    'index_mcp_roll': (-0.26, 0.26),
    'index_mcp_pitch': (0.0, 1.57),
    'index_pip': (0.0, 1.57),
    'thumb_cmc_yaw': (0.0, 1.57),
    'thumb_cmc_pitch': (-0.39, 0.39),
    'thumb_mcp': (0.0, 1.57),
    'thumb_dip': (0.0, 1.57),
}


MIMIC_JOINTS = {
    'pinky_dip': 'pinky_pip',
    'ring_dip': 'ring_pip',
    'middle_dip': 'middle_pip',
    'index_dip': 'index_pip',
}


LOCKED_JOINTS = {
    'thumb_cmc_roll': 0.0,
}


RVIZ_JOINTS = (
    'wrist_pitch',
    'pinky_mcp_roll',
    'pinky_mcp_pitch',
    'pinky_pip',
    'pinky_dip',
    'ring_mcp_roll',
    'ring_mcp_pitch',
    'ring_pip',
    'ring_dip',
    'middle_mcp_roll',
    'middle_mcp_pitch',
    'middle_pip',
    'middle_dip',
    'index_mcp_roll',
    'index_mcp_pitch',
    'index_pip',
    'index_dip',
    'thumb_cmc_yaw',
    'thumb_cmc_roll',
    'thumb_cmc_pitch',
    'thumb_mcp',
    'thumb_dip',
)
