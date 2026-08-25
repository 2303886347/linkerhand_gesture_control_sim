# Linker Hand L30 左手模型

`linkerhand_l30_left_description` 提供 Linker Hand L30 v1.6 左手的 ROS 2 URDF、mesh、
关节限位、RViz 2 配置和 Gazebo Sim 模型生成入口。该包由原始 ROS 1 SolidWorks
导出资源适配到 ROS 2 Humble。

## RViz 2 独立显示

```bash
ros2 launch linkerhand_l30_left_description display.launch.py
```

该入口带关节调节界面，适合独立检查模型。完整摄像头手势同步请使用：

```bash
ros2 launch linkerhand_retargeting mediapipe_rviz_left.launch.py
```

## Gazebo Sim 独立生成

```bash
ros2 launch linkerhand_l30_left_description gazebo.launch.py
```

该入口只生成原始描述模型。完整摄像头到 Gazebo 同步请使用：

```bash
ros2 launch linkerhand_gazebo_control mediapipe_gazebo_left.launch.py
```

无图形界面运行原始模型：

```bash
ros2 launch linkerhand_l30_left_description gazebo.launch.py headless:=true
```

## 构建

```bash
colcon build --symlink-install --packages-select linkerhand_l30_left_description
source install/setup.bash
```
