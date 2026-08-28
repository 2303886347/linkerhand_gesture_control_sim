"""L30 Gazebo 旧版关节常量的兼容导出。"""

from linkerhand_model_profiles import load_model_profile


_L30_PROFILE = load_model_profile('l30', 'left')

# Gazebo 中从动关节也单独同步，避免理想 mimic 约束产生反作用锁死。
CONTROLLED_JOINTS = _L30_PROFILE.controlled_joints
FIXED_TARGETS = dict(_L30_PROFILE.locked_joints)
