# Linker Hand 角度重定向

`linkerhand_retargeting` 是项目的核心角度转换与稳定化包。它把 MediaPipe 输出的人手
语义角度转换为 Linker Hand L30 左手或右手的关节目标，并提供 RViz 左手、右手和
双手完整启动入口。

## 数据处理

```text
MediaPipe 人手角度
  -> 左右手独立输入范围标定
  -> Linker Hand 输出角度映射
  -> 关节死区与迟滞
  -> EMA 平滑
  -> 关节速度限制
  -> 丢失保持与安全回零
  -> Linker Hand 关节目标
```

项目默认映射四指 MCP/PIP 和拇指 MCP/IP 屈曲。腕关节、四指侧摆以及当前未由
MediaPipe 稳定估计的拇指 CMC 轴保持安全零位。四指 DIP 在 RViz/Gazebo 适配层中
跟随对应 PIP。

## RViz 一键启动

```bash
# 左手
ros2 launch linkerhand_retargeting mediapipe_rviz_left.launch.py

# 右手
ros2 launch linkerhand_retargeting mediapipe_rviz_right.launch.py

# 双手，共享一个摄像头并在一个 RViz 中显示
ros2 launch linkerhand_retargeting mediapipe_rviz_both.launch.py
```

三个入口会自动启动摄像头、MediaPipe、角度重定向、状态发布和 RViz，不需要提前
运行其他节点。摄像头不是 `/dev/video0` 时追加 `device:=/dev/video2`。

## 左右手配置

```text
config/retargeting_left.yaml
config/retargeting_right.yaml
```

两个文件完全独立，标定值使用度数，便于和 MediaPipe 调试窗口直接对照：

```yaml
mapping_angle_unit: deg
```

ROS `JointState`、URDF、RViz 和 Gazebo 内部仍使用标准弧度。

实测 PIP/拇指 MCP 输入范围：

| 手侧 | 拇指 MCP | 食指 PIP | 中指 PIP | 无名指 PIP | 小拇指 PIP |
| --- | --- | --- | --- | --- | --- |
| 左手 | 5~35° | 15~75° | 30~85° | 20~80° | 20~80° |
| 右手 | 10~40° | 5~95° | 35~90° | 20~80° | 15~80° |

## 滤波与稳定化参数

```yaml
filter_alpha: 0.35
hold_timeout: 0.80
max_joint_velocity: 3.0
return_joint_velocity: 0.8
joint_deadband:
  enabled: true
  start_moving_deg: 1.5
  stop_moving_deg: 0.5
  settle_frames: 3
  thumb_start_moving_deg: 2.5
  thumb_stop_moving_deg: 0.8
```

- 死区与迟滞锁住静止小噪声，并避免运动阈值附近反复启停。
- EMA 平滑映射后的关节目标。
- 正常动作与目标丢失回零使用不同速度限制。
- `hold_timeout` 跨过半握遮挡和 3D 关键点短时退化，避免状态误切换。

完整调参方法见仓库根目录 [GUIDE.md](../../GUIDE.md)。

## 主要话题

| 功能 | 左手 | 右手 |
| --- | --- | --- |
| MediaPipe 姿态 | `/left/mediapipe/hand_pose` | `/right/mediapipe/hand_pose` |
| 机械手目标 | `/left/linkerhand/target_joint_states` | `/right/linkerhand/target_joint_states` |
| 重定向状态 | `/left/linkerhand/retargeting_status` | `/right/linkerhand/retargeting_status` |
| RViz 关节 | `/left/joint_states` | `/right/joint_states` |

## 构建与测试

```bash
colcon build --symlink-install --packages-up-to linkerhand_retargeting
source install/setup.bash
colcon test --packages-select linkerhand_retargeting
```
