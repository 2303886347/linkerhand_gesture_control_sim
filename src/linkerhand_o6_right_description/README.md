# Linker Hand O6 右手模型

本包将 Linker Hand 官方公开的 O6 右手 URDF 和 mesh 适配为 ROS 2 Humble
description 包。适配统一了 ROS 2 资源路径和型号内关节名称，并增加无质量的
`base_footprint` 挂载根；官方手部几何、坐标、转轴、惯量、联动关系及关节限位
保持不变。

```bash
ros2 launch linkerhand_o6_right_description display.launch.py
```

O6 右手包含 6 个主动自由度和 5 个 mimic 从动关节。
