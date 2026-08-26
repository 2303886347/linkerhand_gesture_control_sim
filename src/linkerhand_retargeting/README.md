# Linker Hand 角度重定向

`linkerhand_retargeting` 是项目的核心角度转换与稳定化包。它把 MediaPipe 输出的人手
语义角度转换为 Linker Hand L30 或 O6 的关节目标，并提供型号独立的稳定化处理。

## 数据处理

```text
MediaPipe 人手角度
  -> 单角度或多角度加权融合
  -> 左右手独立输入范围标定
  -> Linker Hand 输出角度映射
  -> 关节死区与迟滞
  -> EMA 平滑
  -> 关节速度限制
  -> 丢失保持与安全回零
  -> Linker Hand 关节目标
```

L30 保持原有四指 MCP/PIP 和拇指 MCP/IP 映射。O6 四指的单个主动屈曲轴默认使用
`35% MCP + 65% PIP` 融合角，O6 拇指包含侧摆和屈曲两个主动轴。mimic 从动关节在
RViz/Gazebo 适配层按各型号 profile 自动展开。

## RViz 一键启动

```bash
# 左手
ros2 launch linkerhand_retargeting mediapipe_rviz_left.launch.py

# 右手
ros2 launch linkerhand_retargeting mediapipe_rviz_right.launch.py

# 双手，共享一个摄像头并在一个 RViz 中显示
ros2 launch linkerhand_retargeting mediapipe_rviz_both.launch.py

# O6 左手
ros2 launch linkerhand_retargeting mediapipe_rviz_o6_left.launch.py

# O6 右手
ros2 launch linkerhand_retargeting mediapipe_rviz_o6_right.launch.py
```

多型号双手使用正式 bringup 入口，支持两个方向的混合组合：

```bash
ros2 launch linkerhand_bringup mediapipe_rviz.launch.py \
  left_model:=l30 right_model:=o6

ros2 launch linkerhand_bringup mediapipe_rviz.launch.py \
  left_model:=o6 right_model:=l30
```

`mediapipe_rviz_both.launch.py` 仍作为兼容入口，默认行为保持 L30 左右手不变；它也接受
`left_model`、`right_model`、`left_parameters_file` 和 `right_parameters_file` 参数。

三个入口会自动启动摄像头、MediaPipe、角度重定向、状态发布和 RViz，不需要提前
运行其他节点。摄像头不是 `/dev/video0` 时追加 `device:=/dev/video2`。

## 左右手配置

```text
config/retargeting_left.yaml
config/retargeting_right.yaml
config/retargeting_o6_left.yaml
config/retargeting_o6_right.yaml
```

两个文件完全独立，标定值使用度数，便于和 MediaPipe 调试窗口直接对照：

```yaml
mapping_angle_unit: deg
```

ROS `JointState`、URDF、RViz 和 Gazebo 内部仍使用标准弧度。

O6 四指融合配置示例：

```yaml
mapping:
  index_mcp_pitch:
    sources: [index_mcp_flexion, index_pip_flexion]
    source_weights: [0.35, 0.65]
    input_min: 9.75
    input_max: 73.25
    output_min: 0.0
    output_max: 90.0
```

旧的单一 `source` 配置继续兼容，L30 无需迁移 YAML。

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
