# linkerhand_model_profiles

该包是 Linker Hand 多型号仿真的统一模型接口。它负责读取型号 YAML、定位描述包中的
URDF，并校验主动关节、mimic 关节、锁定关节、关节顺序、限位和 Gazebo 参数。

第一阶段只注册已经验收通过的 L30 左右手。后续加入 O6 时，重定向、RViz 和 Gazebo
继续使用同一个加载接口，不在各节点中新增型号判断分支。

```python
from linkerhand_model_profiles import load_model_profile

profile = load_model_profile('l30', 'left')
print(profile.active_joints)
print(profile.joint_limits['index_mcp_pitch'])
```

型号 profile 损坏、URDF 关节不匹配或型号不存在时会直接报错，不进行静默回退。
