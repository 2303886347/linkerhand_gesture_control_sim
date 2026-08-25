# Linker Hand L30 右手模型

`linkerhand_l30_right_description` 提供 Linker Hand L30 v6 右手的 ROS 2 URDF、mesh、
关节限位、RViz 2 配置和 Gazebo Sim 模型生成入口。包内保留右手独立几何、惯量、
关节轴和运动范围。

## RViz 2 独立显示

```bash
ros2 launch linkerhand_l30_right_description display.launch.py
```

该入口带关节调节界面，适合独立检查模型。完整摄像头手势同步请使用：

```bash
ros2 launch linkerhand_retargeting mediapipe_rviz_right.launch.py
```

## Gazebo Sim 独立生成

```bash
ros2 launch linkerhand_l30_right_description gazebo.launch.py
```

该入口只生成原始描述模型。完整摄像头到 Gazebo 同步请使用：

```bash
ros2 launch linkerhand_gazebo_control mediapipe_gazebo_right.launch.py
```

无图形界面运行原始模型：

```bash
ros2 launch linkerhand_l30_right_description gazebo.launch.py headless:=true
```

## 构建

```bash
colcon build --symlink-install --packages-select linkerhand_l30_right_description
source install/setup.bash
```
