"""L30 旧版关节常量的兼容导出，新代码应直接加载型号 profile。"""

from linkerhand_model_profiles import load_model_profile


_L30_PROFILE = load_model_profile('l30', 'left')

INDEPENDENT_JOINTS = _L30_PROFILE.active_joints
JOINT_LIMITS = dict(_L30_PROFILE.joint_limits)
MIMIC_JOINTS = {
    joint: settings.source
    for joint, settings in _L30_PROFILE.mimic_joints.items()
}
LOCKED_JOINTS = dict(_L30_PROFILE.locked_joints)
RVIZ_JOINTS = _L30_PROFILE.full_joints
