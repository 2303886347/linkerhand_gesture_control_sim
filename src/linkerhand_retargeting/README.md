# Linker Hand 角度转换

该包位于 MediaPipe 感知与机械手显示/控制之间，负责把人体语义角度转换为
Linker Hand L30 左手或右手的独立关节目标。包边界保持独立，后续可以继续加入
标定、滤波、动作约束，以及 RViz、Gazebo 或真实硬件输出。

当前第一阶段只控制以下 10 个屈曲关节：

- 四指的 `mcp_pitch` 和 `pip`
- 拇指的 `thumb_mcp` 和 `thumb_dip`

腕关节、四指侧摆和拇指 CMC 暂时固定为零。输出包含 17 个独立关节，单位为
弧度；四指 DIP 的 mimic 展开将在接入 RViz 时由适配节点处理。

## 构建

```bash
cd /home/ubuntu/linkerhand_ros2_ws
colcon build --symlink-install --packages-up-to linkerhand_retargeting
source install/setup.bash
```

## 启动

先启动摄像头和 MediaPipe：

```bash
ros2 launch mediapipe_hand_pose pipeline.launch.py
```

再在新终端启动角度转换：

```bash
ros2 launch linkerhand_retargeting retargeting.launch.py
```

查看输出：

```bash
ros2 topic echo /linkerhand/target_joint_states --once
ros2 topic echo /linkerhand/retargeting_status
```

左右手分别使用 `retargeting_left.yaml` 和 `retargeting_right.yaml`。两个映射
文件都设置 `mapping_angle_unit: deg`，所以 `input_min`、`input_max`、
`output_min`、`output_max`、固定位置和安全位置都直接填写度数，便于和
MediaPipe 调试窗口对照。ROS 话题、URDF、RViz 和后续控制器内部仍按标准使用
弧度。需要改回弧度配置时可设置 `mapping_angle_unit: rad`。

`max_joint_velocity` 控制正常跟手的最大速度，`return_joint_velocity` 单独控制
目标丢失后的回零速度。默认回零速度为 `0.8 rad/s`，不会拖慢正常跟手。

左右手配置还分别提供映射后的关节死区与迟滞消抖，阈值直接使用度数：

```yaml
joint_deadband:
  enabled: true
  start_moving_deg: 1.5
  stop_moving_deg: 0.5
  settle_frames: 3
  thumb_start_moving_deg: 2.5
  thumb_stop_moving_deg: 0.8
```

静止时，关节只有偏离锁定值达到 `start_moving_deg` 才重新跟随；运动后连续
`settle_frames` 帧进入 `stop_moving_deg` 范围才重新锁定。拇指因映射倍率较大而
使用独立阈值。修改 YAML 后需要重启对应启动文件。

当前四指 PIP 已按左手实测范围标定：

| 关节 | MediaPipe 输入 | RViz 输出 |
| --- | --- | --- |
| 食指 PIP | 15~75 度 | 0~90 度 |
| 中指 PIP | 30~85 度 | 0~90 度 |
| 无名指 PIP | 20~80 度 | 0~90 度 |
| 小拇指 PIP | 20~80 度 | 0~90 度 |
| 拇指 MCP | 5~35 度 | 0~85 度 |

右手实测标定：

| 关节 | MediaPipe 输入 | RViz 输出 |
| --- | --- | --- |
| 食指 PIP | 5~95 度 | 0~90 度 |
| 中指 PIP | 35~90 度 | 0~90 度 |
| 无名指 PIP | 20~80 度 | 0~90 度 |
| 小拇指 PIP | 15~80 度 | 0~90 度 |
| 拇指 MCP | 10~40 度 | 0~85 度 |

调试窗口中的四指摘要显示的是 PIP 角度，因此本轮只据此标定 PIP。MCP 仍由
对应的 `*_mcp_flexion` 独立驱动，待单独测量 MCP 输入范围后再校准。

## 启动 MediaPipe 和 RViz 同步

左手、右手和双手分别提供独立入口。三个入口都会启动摄像头、MediaPipe、角度
转换和 RViz，不需要提前单独启动其他节点。

只跟踪左手：

```bash
ros2 launch linkerhand_retargeting mediapipe_rviz_left.launch.py
```

只跟踪右手：

```bash
ros2 launch linkerhand_retargeting mediapipe_rviz_right.launch.py
```

同时跟踪左右手，并在同一个 RViz 中显示两个模型：

```bash
ros2 launch linkerhand_retargeting mediapipe_rviz_both.launch.py
```

单手模式默认以 15 FPS 运行一个 MediaPipe 实例。双手模式共享同一个 USB 摄像头，
左右识别、角度转换、关节状态和 TF 完全隔离；默认每边以 10 FPS 运行一个
MediaPipe 实例，降低同时推理时的 CPU 压力。双手主要话题如下：

| 功能 | 左手 | 右手 |
| --- | --- | --- |
| 人手姿态 | `/left/mediapipe/hand_pose` | `/right/mediapipe/hand_pose` |
| 机械手目标 | `/left/linkerhand/target_joint_states` | `/right/linkerhand/target_joint_states` |
| RViz 关节 | `/left/joint_states` | `/right/joint_states` |
| 模型描述 | `/left/robot_description` | `/right/robot_description` |

三个入口均默认显示镜像 MediaPipe 预览。需要使用其他摄像头时可追加
`device:=/dev/videoN`。单手模式可用 `mediapipe_show_preview:=false` 关闭预览；
双手模式对应参数为 `show_previews:=false`。三种模式都可用 `use_rviz:=false`
暂时关闭 RViz，也都支持通过 `one_euro_min_cutoff` 和 `one_euro_beta` 调整
One Euro 滤波。

原有 `mediapipe_rviz.launch.py` 继续保留为通用兼容入口，默认使用左手。日常使用
建议直接选择上面的左手、右手或双手入口，避免模型和标定组合错误。
