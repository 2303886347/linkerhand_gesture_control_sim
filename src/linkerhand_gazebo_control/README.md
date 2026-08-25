# Linker Hand Gazebo 手势同步

`linkerhand_gazebo_control` 把经过标定和滤波的 Linker Hand 关节目标同步到 Gazebo
Sim。它提供单左手和单右手完整入口，用户只需普通 USB 摄像头即可在 Gazebo 中实时
体验灵巧手跟随动作。

## 一键启动

```bash
# 左手
ros2 launch linkerhand_gazebo_control mediapipe_gazebo_left.launch.py

# 右手
ros2 launch linkerhand_gazebo_control mediapipe_gazebo_right.launch.py
```

指定摄像头：

```bash
ros2 launch linkerhand_gazebo_control mediapipe_gazebo_right.launch.py \
  device:=/dev/video2
```

无摄像头时可以只启动 Gazebo 控制链路，用 ROS 2 话题发送测试目标：

```bash
ros2 launch linkerhand_gazebo_control gazebo_control.launch.py side:=right
```

## 控制链路

```text
/left|right/linkerhand/target_joint_states
  -> trajectory_adapter
  -> JointTrajectory
  -> ros_gz_bridge
  -> OnlineJointController
  -> Gazebo 关节
  -> Gazebo JointStatePublisher
  -> joint_state_throttle
  -> /left|right/joint_states（60 Hz）
```

轨迹适配器会检查数组长度、关节完整性和有限值，补齐固定关节，并让四指 DIP 跟随
对应 PIP。Gazebo 插件只接受完整的 22 关节目标，避免残缺消息造成模型部分跳变。

## 同步原理

在线插件在每个仿真步计算：

```text
velocity = Kp * (target_position - actual_position)
```

速度限制为 `3 rad/s`，再根据仿真步长更新绝对关节位置。Gazebo 的实际关节状态经过
桥接和节流，以 `60 Hz` 发布回 ROS 2。One Euro、死区/迟滞、EMA、动作限速和丢失
回零由上游 `linkerhand_retargeting` 统一完成。

## 运行时模型适配

为让原始 SolidWorks URDF 适合实时手势显示，运行时会生成独立 URDF，不修改原文件：

- 添加固定的 `world -> base_footprint`。
- 移除四指 DIP mimic，由轨迹适配器显式同步。
- 保留 visual，移除会让相邻指节互锁的高精度 STL collision。
- 保留粘性阻尼，将不适合轻量指节的统一库仑摩擦设为零。
- 默认把左手质量和惯量乘以 `1/7.6`，右手保持原比例。
- 加载项目自带的在线关节同步插件和 Gazebo 状态发布插件。

## 项目边界

本包面向手势可视化和交互仿真，使用运动学位置反馈同步。它不模拟真实 Linker Hand
电机、减速器、力矩或接触抓取。原始 collision 被有意移除，因此模型可以展示动作，
但不用于物体抓取和接触力分析。

## 主要话题

| 功能 | 左手 | 右手 |
| --- | --- | --- |
| 关节目标 | `/left/linkerhand/target_joint_states` | `/right/linkerhand/target_joint_states` |
| Gazebo 轨迹 | `/left/gazebo_joint_trajectory` | `/right/gazebo_joint_trajectory` |
| 原始仿真状态 | `/left/gazebo_joint_states_raw` | `/right/gazebo_joint_states_raw` |
| 60 Hz 对外状态 | `/left/joint_states` | `/right/joint_states` |

## 构建与测试

```bash
colcon build --symlink-install \
  --packages-select linkerhand_gazebo_plugin linkerhand_gazebo_control
source install/setup.bash
colcon test --packages-select linkerhand_gazebo_control
```
