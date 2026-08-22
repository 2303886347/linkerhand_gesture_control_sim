# LinkerHand Gesture Control Sim 调试与运行手册

本文档面向本地运行、标定、滤波调节和故障排查。项目介绍和英文文档请返回
[README.md](README.md) 或 [README.en.md](README.en.md)。

## 目录

- [1. 系统边界](#1-系统边界)
- [2. 环境检查](#2-环境检查)
- [3. 构建与测试](#3-构建与测试)
- [4. 三种完整运行模式](#4-三种完整运行模式)
- [5. 分模块启动](#5-分模块启动)
- [6. ROS 2 节点与话题](#6-ros-2-节点与话题)
- [7. 左右手标定](#7-左右手标定)
- [8. 滤波原理和参数](#8-滤波原理和参数)
- [9. RViz 与 TF 检查](#9-rviz-与-tf-检查)
- [10. 常见问题](#10-常见问题)
- [11. Git 分支](#11-git-分支)

## 1. 系统边界

当前数据链路：

```text
USB 摄像头
  -> MediaPipe Hands
  -> 人手关键点与语义关节角
  -> 左/右手独立标定映射
  -> One Euro + EMA + 速度限制
  -> ROS JointState
  -> RViz 2 左手、右手或双手模型
```

已完成：

- 摄像头图像发布与 OpenCV 预览。
- 左右手识别和角度输出。
- 左右手独立实测标定。
- 单左手、单右手、双手 RViz 同步。
- One Euro、EMA、限速、短暂保持和平滑回零。
- 左右手独立 URDF、mesh、RViz 和 Gazebo Sim 模型启动。

尚未完成：

- MediaPipe 目标直接驱动 Gazebo 控制器。
- 真实 Linker Hand 硬件控制。
- 碰撞约束、动力学控制和力反馈。

## 2. 环境检查

### 2.1 ROS 2

```bash
echo "$ROS_DISTRO"
ros2 --help >/dev/null && echo "ROS 2 CLI 正常"
```

预期 ROS 发行版为 `humble`。如果新终端没有自动加载：

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/linkerhand_ros2_ws/install/setup.bash
```

本机 `~/.bashrc` 已包含工作空间自动加载逻辑。修改或重新构建工作空间后，建议打开
新终端，或者手动执行一次上面的 `source`。

### 2.2 摄像头

```bash
ls -l /dev/video*
```

如果有多个设备，可以逐个测试：

```bash
ros2 launch usb_camera_demo usb_camera.launch.py device:=/dev/video0
```

没有权限时检查当前用户是否属于 `video` 组：

```bash
groups
```

### 2.3 Python 与 MediaPipe

```bash
python3 -c "import cv2, mediapipe, numpy; print('OpenCV/MediaPipe/NumPy 正常')"
```

### 2.4 图形环境

```bash
echo "$DISPLAY"
echo "$WAYLAND_DISPLAY"
```

通过 SSH 或无桌面环境运行时，关闭预览和 RViz：

```bash
ros2 launch linkerhand_retargeting mediapipe_rviz_both.launch.py \
  show_previews:=false use_rviz:=false
```

## 3. 构建与测试

### 3.1 完整构建

```bash
cd /home/ubuntu/linkerhand_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`setuptools` 的 `easy_install` 弃用提示不会导致当前构建失败；应重点关注最后的
`Summary` 和真正的 `error`。

### 3.2 只构建手势链路

```bash
colcon build --symlink-install --packages-up-to linkerhand_retargeting
source install/setup.bash
```

### 3.3 测试

```bash
colcon test --packages-select mediapipe_hand_pose linkerhand_retargeting
colcon test-result --verbose
```

当前基线为：

```text
34 tests, 0 errors, 0 failures
```

## 4. 三种完整运行模式

三个入口都会启动摄像头、MediaPipe、角度转换、关节适配器和 RViz。

### 4.1 只运行左手

```bash
ros2 launch linkerhand_retargeting mediapipe_rviz_left.launch.py
```

加载内容：

- `target_hand=left`
- `retargeting_left.yaml`
- `linkerhand_l30_left_description`

### 4.2 只运行右手

```bash
ros2 launch linkerhand_retargeting mediapipe_rviz_right.launch.py
```

加载内容：

- `target_hand=right`
- `retargeting_right.yaml`
- `linkerhand_l30_right_description`

### 4.3 同时运行左右手

```bash
ros2 launch linkerhand_retargeting mediapipe_rviz_both.launch.py
```

双手模式特性：

- `/usb_camera` 只启动一次。
- `/left/mediapipe_hand_pose` 和 `/right/mediapipe_hand_pose` 分别识别目标手。
- 左右标定参数、JointState、robot_description 和 TF 完全隔离。
- TF 使用 `left/...` 和 `right/...` 命名空间。
- 一个 RViz 同时显示两个 RobotModel。
- 默认每侧 10 FPS，降低双实例 CPU 压力。

### 4.4 常用启动参数

```bash
ros2 launch linkerhand_retargeting mediapipe_rviz_both.launch.py \
  device:=/dev/video0 \
  width:=640 \
  height:=480 \
  camera_fps:=30.0 \
  processing_fps:=12.0 \
  show_previews:=true \
  mirror_preview:=true \
  use_rviz:=true
```

查看全部参数：

```bash
ros2 launch linkerhand_retargeting mediapipe_rviz_both.launch.py --show-args
```

## 5. 分模块启动

### 5.1 只测试左手模型

```bash
ros2 launch linkerhand_l30_left_description display.launch.py
```

### 5.2 只测试右手模型

```bash
ros2 launch linkerhand_l30_right_description display.launch.py
```

### 5.3 Gazebo Sim 模型生成

```bash
ros2 launch linkerhand_l30_left_description gazebo.launch.py
ros2 launch linkerhand_l30_right_description gazebo.launch.py
```

这里仅生成模型，不会接收 MediaPipe 目标进行控制。

### 5.4 只测试摄像头

```bash
ros2 launch usb_camera_demo usb_camera.launch.py
```

图像话题：

```text
/usb_camera/image_raw
```

### 5.5 只测试 MediaPipe

```bash
ros2 launch mediapipe_hand_pose pipeline.launch.py
```

### 5.6 只启动角度转换

先保持 MediaPipe pipeline 运行，再在新终端执行：

```bash
ros2 launch linkerhand_retargeting retargeting.launch.py
```

## 6. ROS 2 节点与话题

### 6.1 双手节点

```bash
ros2 node list | sort
```

关键节点：

```text
/usb_camera
/left/mediapipe_hand_pose
/right/mediapipe_hand_pose
/left/linkerhand_retargeting
/right/linkerhand_retargeting
/left/linkerhand_rviz_joint_state_adapter
/right/linkerhand_rviz_joint_state_adapter
/left/robot_state_publisher
/right/robot_state_publisher
/rviz2_both_hands
```

### 6.2 双手话题

| 功能 | 左手 | 右手 |
| --- | --- | --- |
| MediaPipe 姿态 | `/left/mediapipe/hand_pose` | `/right/mediapipe/hand_pose` |
| 人体关节角 | `/left/mediapipe/human_joint_angles` | `/right/mediapipe/human_joint_angles` |
| 调试图像 | `/left/mediapipe/debug_image` | `/right/mediapipe/debug_image` |
| 映射目标 | `/left/linkerhand/target_joint_states` | `/right/linkerhand/target_joint_states` |
| RViz 关节 | `/left/joint_states` | `/right/joint_states` |
| 模型描述 | `/left/robot_description` | `/right/robot_description` |

检查关节：

```bash
ros2 topic echo /left/joint_states sensor_msgs/msg/JointState --once
ros2 topic echo /right/joint_states sensor_msgs/msg/JointState --once
```

两侧均应包含 22 个 RViz 关节名称。

检查左右参数：

```bash
ros2 param get /left/mediapipe_hand_pose target_hand
ros2 param get /right/mediapipe_hand_pose target_hand
ros2 param get /left/linkerhand_retargeting accepted_hand
ros2 param get /right/linkerhand_retargeting accepted_hand
```

预期依次为 `left`、`right`、`left`、`right`。

## 7. 左右手标定

标定配置使用度数，便于和 MediaPipe 调试窗口直接比较：

```yaml
mapping_angle_unit: deg
```

ROS `JointState`、URDF、RViz 和后续控制器内部仍使用弧度。

### 7.1 左手实测范围

| 关节 | MediaPipe 输入 | RViz 输出 |
| --- | --- | --- |
| 拇指 MCP | 5~35 度 | 0~85 度 |
| 食指 PIP | 15~75 度 | 0~90 度 |
| 中指 PIP | 30~85 度 | 0~90 度 |
| 无名指 PIP | 20~80 度 | 0~90 度 |
| 小拇指 PIP | 20~80 度 | 0~90 度 |

配置文件：

```text
src/linkerhand_retargeting/config/retargeting_left.yaml
```

### 7.2 右手实测范围

| 关节 | MediaPipe 输入 | RViz 输出 |
| --- | --- | --- |
| 拇指 MCP | 10~40 度 | 0~85 度 |
| 食指 PIP | 5~95 度 | 0~90 度 |
| 中指 PIP | 35~90 度 | 0~90 度 |
| 无名指 PIP | 20~80 度 | 0~90 度 |
| 小拇指 PIP | 15~80 度 | 0~90 度 |

配置文件：

```text
src/linkerhand_retargeting/config/retargeting_right.yaml
```

四指调试摘要显示的是 PIP 角度，因此当前实测值只用于 PIP。MCP 仍使用独立的
`*_mcp_flexion` 输入，后续应分别测量 MCP 的伸直和弯曲范围。

## 8. 滤波原理和参数

当前链路包含四层处理：

```text
MediaPipe 原始检测
  -> One Euro 关键点滤波（调试骨架）
  -> 计算人体关节角
  -> One Euro 关节角滤波
  -> 标定映射
  -> 关节死区与迟滞
  -> EMA 目标滤波
  -> 关节速度限制
  -> RViz
```

MediaPipe 优先用 3D `world_landmarks` 计算角度；如果半握遮挡导致 3D 几何暂时
退化，但图像骨架仍存在，节点会自动回退到经过关键点滤波的图像坐标继续计算，避免
发布空角度并触发重定向回零。

### 8.1 One Euro

默认参数：

```yaml
use_one_euro_filter: true
one_euro_min_cutoff: 0.8
one_euro_beta: 0.3
one_euro_derivative_cutoff: 1.0
one_euro_reset_timeout: 0.5
```

| 参数 | 调小 | 调大 |
| --- | --- | --- |
| `min_cutoff` | 静止更稳、延迟更大 | 更灵敏、静止抖动更多 |
| `beta` | 快速动作也更平滑、更慢 | 快速动作跟随更快 |
| `derivative_cutoff` | 速度估计更平滑 | 速度估计更敏感 |

静止稳定优先的推荐起点：

```bash
ros2 launch linkerhand_retargeting mediapipe_rviz_both.launch.py \
  one_euro_min_cutoff:=0.5 \
  one_euro_beta:=0.4
```

### 8.2 关节死区与迟滞

死区层位于标定映射之后、EMA 之前。每个机械手关节独立判断运动状态：

- 静止时保持锁定角度，小范围噪声不会继续传给 RViz。
- 偏离锁定角度达到启动阈值后，恢复连续跟随。
- 运动过程中，连续若干帧的角度范围进入停止阈值后重新锁定。
- 启动阈值大于停止阈值，避免临界位置反复启停。

左右手分别在 `retargeting_left.yaml` 和 `retargeting_right.yaml` 中配置：

```yaml
joint_deadband:
  enabled: true
  start_moving_deg: 1.5
  stop_moving_deg: 0.5
  settle_frames: 3
  thumb_start_moving_deg: 2.5
  thumb_stop_moving_deg: 0.8
```

这些阈值使用映射后的机械手目标角度，单位固定为度，与 ROS 话题内部使用弧度无关。
拇指输入范围较窄、映射倍率较大，因此使用独立且稍宽的阈值。

- 静止仍抖：先把 `start_moving_deg` 每次提高 `0.2~0.5` 度。
- 慢动作有台阶感：降低 `start_moving_deg`，不要先降低 `filter_alpha`。
- 动作停止后锁定太慢：减小 `settle_frames`，最小值为 `2`。
- 运动中频繁锁定：减小 `stop_moving_deg` 或增大 `settle_frames`。
- 临时对照原始效果：设置 `enabled: false` 并重启节点。

### 8.3 EMA

EMA 公式：

```text
filtered = alpha * current + (1 - alpha) * previous
```

当前：

```yaml
filter_alpha: 0.35
```

- 调小到 `0.25`：RViz 更稳定，但响应变慢。
- 低于 `0.20`：可能出现明显拖尾。
- 调大到 `0.5`：响应更快，但更容易看到抖动。

`filter_alpha` 位于左右两个 `retargeting_*.yaml` 中。当前节点在启动时读取参数，修改
后需要重新启动；仅执行 `ros2 param set` 不会重建内部滤波器。

### 8.4 速度和丢失目标

```yaml
hold_timeout: 0.80
max_joint_velocity: 3.0
return_joint_velocity: 0.8
```

- `hold_timeout`：连续无效超过该时间后才返回安全姿态；默认 `0.80s`，用于跨过
  半握遮挡或 3D 关键点短时退化。
- `max_joint_velocity`：正常跟手最大速度，单位 `rad/s`。
- `return_joint_velocity`：丢失目标后回到开掌姿态的速度。

速度限制主要处理大幅跳变，不应代替 One Euro 或 EMA 的静止消抖。

### 8.5 推荐调参顺序

1. 观察 MediaPipe 骨架是否抖动。
2. 骨架抖动时先降低 `one_euro_min_cutoff`。
3. 快速动作太慢时适当提高 `one_euro_beta`。
4. 骨架稳定但 RViz 仍有小幅静止抖动时，先调整 `joint_deadband`。
5. 静止稳定但运动过程仍过于敏感时，再把 `filter_alpha` 从 `0.35` 降到
   `0.25~0.30`。

不建议直接叠加卡尔曼滤波。对于“静止抑制噪声、运动保持响应”的手势跟踪，One Euro
通常更容易调。卡尔曼更适合需要速度状态估计、短时预测或遮挡补偿的后续阶段。

## 9. RViz 与 TF 检查

双手 RViz 固定坐标系：

```text
world
```

左右根节点：

```text
world -> left/base_footprint
world -> right/base_footprint
```

检查完整 TF：

```bash
ros2 run tf2_ros tf2_echo world left/index_distal
ros2 run tf2_ros tf2_echo world right/index_distal
```

RViz 中 `LeftHand` 或 `RightHand` 出现红色错误图标时，重点检查：

1. `robot_description` 是否存在。
2. RobotModel 的 `TF Prefix` 是否分别为 `left` 和 `right`。
3. `frame_prefix` 是否发布为 `left/` 和 `right/`。
4. 是否在新终端中加载了最新 `install/setup.bash`。
5. 修改 launch 或 RViz 配置后是否重新构建。

## 10. 常见问题

### 没有 MediaPipe 预览窗口

```bash
echo "$DISPLAY"
ros2 param get /mediapipe_hand_pose show_preview
```

单手入口参数：

```text
mediapipe_show_preview:=true
```

双手入口参数：

```text
show_previews:=true
```

### 摄像头无法打开

确认设备没有被其他程序占用：

```bash
fuser /dev/video0
```

尝试另一个设备：

```bash
ros2 launch linkerhand_retargeting mediapipe_rviz_left.launch.py \
  device:=/dev/video1
```

### 左右手识别相反

`mirror_preview` 只改变显示画面。普通未镜像摄像头输入应保持：

```yaml
input_mirrored: false
mirror_preview: true
```

### 手移出画面后回零太快或太慢

修改对应标定文件中的：

```yaml
return_joint_velocity: 0.8
```

数值越小，回零越慢。

### 双手模式 CPU 占用过高

```bash
ros2 launch linkerhand_retargeting mediapipe_rviz_both.launch.py \
  processing_fps:=8.0
```

也可以关闭预览：

```text
show_previews:=false
```

## 11. Git 分支

推荐使用方式：

```bash
git switch main
```

`main` 包含全部功能。三个演示分支只突出对应运行模式：

```bash
git switch left-hand
git switch right-hand
git switch both-hands
```

不要在三个模式分支中分别维护算法实现。通用修复应先进入 `main`，再同步演示分支，
否则标定、滤波、消息接口和 RViz 配置很容易产生不一致。
