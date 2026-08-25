"""Gazebo 在线同步插件直接控制的 Linker Hand 关节集合。"""

from linkerhand_retargeting.joints import RVIZ_JOINTS


# Gazebo 中四指 DIP 也单独同步，目标值由适配器复制对应 PIP。
# 这样可避免模拟器的理想 mimic 约束在串联指节上产生反作用锁死。
CONTROLLED_JOINTS = RVIZ_JOINTS

FIXED_TARGETS = {
    'thumb_cmc_roll': 0.0,
}
