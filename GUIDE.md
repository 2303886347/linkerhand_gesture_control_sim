# LinkerHand Gesture Control Sim 调试与运行手册

本文档面向本地运行、标定、滤波调节和故障排查。项目介绍和英文文档请返回
[README.md](README.md) 或 [README.en.md](README.en.md)。

## 目录

- [1. 项目范围](#1-项目范围)
- [2. 环境检查](#2-环境检查)
- [3. 构建与测试](#3-构建与测试)
- [4. RViz 多型号运行模式](#4-rviz-多型号运行模式)
- [5. 分模块启动](#5-分模块启动)
- [6. ROS 2 节点与话题](#6-ros-2-节点与话题)
- [7. 左右手标定](#7-左右手标定)
- [8. 滤波原理和参数](#8-滤波原理和参数)
- [9. RViz 与 TF 检查](#9-rviz-与-tf-检查)
- [10. Gazebo 手势同步](#10-gazebo-手势同步)
- [11. 常见问题](#11-常见问题)
- [12. Git 分支](#12-git-分支)

## 1. 项目范围

完整数据链路：

```text
USB 摄像头
  -> MediaPipe Hands
  -> 人手关键点与语义关节角
  -> 左/右手独立标定映射
  -> One Euro + EMA + 速度限制
  -> RViz 2 L30/O6 单手或任意左右组合
  -> Gazebo Sim L30/O6 单左手或单右手模型
```

项目正式提供：

- 摄像头图像发布与 OpenCV 预览。
- 左右手识别和角度输出。
- 左右手独立实测标定。
- L30/O6 单手和任意左右型号组合的 RViz 同步。
- One Euro、EMA、限速、短暂保持和平滑回零。
- 左右手独立 URDF、mesh、RViz 和 Gazebo Sim 模型启动。
- MediaPipe 到 Gazebo 单左手、单右手运动学位置反馈同步。
- Gazebo 实际关节状态以 60 Hz 反馈到 ROS 2。

适用场景：

- 使用普通 USB 摄像头体验 Linker Hand 手势同步。
- 学习 ROS 2 图像、消息、话题、TF、URDF 和仿真插件链路。
- 调试 MediaPipe 关节角、左右手独立标定和多层滤波。
- 在 RViz 或 Gazebo 中演示灵巧手动作映射。

项目完整目标是实现“画面采集 -> MediaPipe 检测 -> 角度重定向与滤波 ->
RViz/Gazebo 显示”。Gazebo 采用运动学位置同步，接触抓取和高保真动力学属于可基于
现有接口扩展的应用方向。

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
ros2 launch linkerhand_bringup mediapipe_rviz.launch.py \
  show_previews:=false use_rviz:=false
```

## 3. 构建与测试

### 3.1 完整构建

```bash
cd /home/ubuntu/linkerhand_ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install 'mediapipe==0.10.9'
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
colcon test
colcon test-result --verbose
```

文档中的测试数量只作为发布时的验证记录；以本机 `colcon test-result --verbose`
输出的 `0 errors, 0 failures` 为通过标准。

## 4. RViz 多型号运行模式

所有入口都会启动摄像头、MediaPipe、角度转换、关节适配器和 RViz。正式多型号入口
位于 `linkerhand_bringup`；原 L30 命令继续兼容。

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

### 4.3 O6 单手

```bash
# O6 左手
ros2 launch linkerhand_retargeting mediapipe_rviz_o6_left.launch.py

# O6 右手
ros2 launch linkerhand_retargeting mediapipe_rviz_o6_right.launch.py
```

O6 四指使用每指独立的 `35% MCP + 65% PIP` 融合角；拇指屈曲和侧摆分别驱动两个
主动关节，mimic 关节由型号 profile 自动展开。

### 4.4 参数式双手核心入口

```bash
ros2 launch linkerhand_bringup mediapipe_rviz.launch.py \
  left_model:=l30 right_model:=o6
```

双手模式特性：

- `/usb_camera` 只启动一次。
- `/left/mediapipe_hand_pose` 和 `/right/mediapipe_hand_pose` 分别识别目标手。
- 左右标定参数、JointState、robot_description 和 TF 完全隔离。
- TF 使用 `left/...` 和 `right/...` 命名空间。
- 一个 RViz 同时显示两个 RobotModel。
- 默认每侧 10 FPS，降低双实例 CPU 压力。
- `left_model` 和 `right_model` 分别从 `l30`、`o6` 中选择。
- 型号默认标定缺失时回退到 profile 默认值，并在终端明确警告。

四种已支持组合：

```bash
ros2 launch linkerhand_bringup mediapipe_rviz.launch.py \
  left_model:=l30 right_model:=l30
ros2 launch linkerhand_bringup mediapipe_rviz.launch.py \
  left_model:=o6 right_model:=o6
ros2 launch linkerhand_bringup mediapipe_rviz.launch.py \
  left_model:=l30 right_model:=o6
ros2 launch linkerhand_bringup mediapipe_rviz.launch.py \
  left_model:=o6 right_model:=l30
```

对应快捷入口：

```bash
ros2 launch linkerhand_bringup mediapipe_rviz_l30_both.launch.py
ros2 launch linkerhand_bringup mediapipe_rviz_o6_both.launch.py
ros2 launch linkerhand_bringup mediapipe_rviz_l30_o6.launch.py
ros2 launch linkerhand_bringup mediapipe_rviz_o6_l30.launch.py
```

旧命令仍默认启动 L30 双手：

```bash
ros2 launch linkerhand_retargeting mediapipe_rviz_both.launch.py
```

### 4.5 个人标定和常用参数

```bash
ros2 launch linkerhand_bringup mediapipe_rviz.launch.py \
  left_model:=l30 \
  right_model:=o6 \
  left_parameters_file:=/path/to/personal_l30_left.yaml \
  right_parameters_file:=/path/to/personal_o6_right.yaml \
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
ros2 launch linkerhand_bringup mediapipe_rviz.launch.py --show-args
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

### 5.3 只生成 Gazebo Sim 模型

```bash
ros2 launch linkerhand_l30_left_description gazebo.launch.py
ros2 launch linkerhand_l30_right_description gazebo.launch.py
```

这里仅生成描述包中的静态模型，不会接收 MediaPipe 目标。完整手势同步请使用
[第 10 节](#10-gazebo-手势同步)的 `linkerhand_gazebo_control` 启动入口。

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

### 6.1 节点命名

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

单手入口只启动对应一侧的节点；双手入口同时启动以上左右两组节点。

### 6.2 左右手话题

| 功能 | 左手 | 右手 |
| --- | --- | --- |
| MediaPipe 姿态 | `/left/mediapipe/hand_pose` | `/right/mediapipe/hand_pose` |
| 人体关节角 | `/left/mediapipe/human_joint_angles` | `/right/mediapipe/human_joint_angles` |
| 调试图像 | `/left/mediapipe/debug_image` | `/right/mediapipe/debug_image` |
| 映射目标 | `/left/linkerhand/target_joint_states` | `/right/linkerhand/target_joint_states` |
| 重定向状态 | `/left/linkerhand/retargeting_status` | `/right/linkerhand/retargeting_status` |
| RViz 关节 | `/left/joint_states` | `/right/joint_states` |
| 模型描述 | `/left/robot_description` | `/right/robot_description` |

左手、右手和双手一体化入口都遵循这张表。RobotModel 不再读取全局
`/robot_description`，因此不会被同一 ROS 2 域中的 MentorPi 或其他机器人模型
覆盖；TF 帧分别使用 `left/` 和 `right/` 前缀。

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

### 7.1 Qt 个人标定上位机

```bash
ros2 launch linkerhand_bringup calibration_gui.launch.py
```

在上位机顶部选择 L30/O6、左手/右手和摄像头，再点击“连接”。标注画面、骨架、
左右手结果、置信度和画面帧率都显示在同一窗口内。

当前可以分别选择“张开手掌、自然握拳、拇指收拢、拇指展开”，由用户点击按钮后
立即采样。每个姿态默认采样 2 秒，支持取消和重新采样；成功后只切换到下一姿态，
不会自动开始。界面会检查目标手、手侧、置信度、整手可见性、有效帧率以及角度
稳定度，并显示具体失败原因。

四个姿态完成后切换到“个人配置”页。只读结果表会按运行时相同的规则计算各驱动关节
的输入最小角、最大角和活动范围；O6 四指仍采用 `35% MCP + 65% PIP`，拇指侧摆
采用“展开为最小、收拢为最大”。默认最小活动范围为 5 度，范围过小或方向反向时会
明确标出关节并阻止生成。

个人配置默认保存在：

```text
~/.config/linkerhand_gesture_control/calibration
```

“生成配置”会新建档案，不覆盖仓库默认 YAML；选择已有配置后可用本次采样更新它，
也可以“另存为新配置”到任意路径。通过“导入 YAML”可以登记默认目录之外的已有个人
配置；列表只显示当前型号和手侧匹配的档案。生成的文件仍是可直接传给 ROS 2
`parameters_file` 的标准参数 YAML，同时保存配置名称、摄像头、质量阈值和四姿态
采样统计。

### 7.2 Qt 一键仿真验证

在“配置档案”中选择一份已经保存、且与当前型号和手侧匹配的个人配置后，底部的
“RViz 验证”和“Gazebo 验证”按钮会启用。存在尚未保存的重新采样结果时，按钮会
禁用，避免验证到旧文件。

点击验证后，上位机会执行以下交接：

1. 如果标定摄像头处于连接状态，先停止 GUI 自己的 MediaPipe 和摄像头进程。
2. 等设备完全释放后，使用当前型号、手侧、摄像头参数和个人 YAML 启动单手验证。
3. 验证 launch 打开 MediaPipe 调试预览，并启动对应的 RViz 或 Gazebo 仿真。
4. 点击 GUI 中的“停止 RViz 验证”或“停止 Gazebo 验证”，清理整个 launch 进程组。
5. 如果验证前 GUI 摄像头已连接，停止后会自动恢复原来的标定连接；验证前未连接则
   保持未连接。

验证运行期间不能切换型号、手侧、配置或摄像头，防止配置和实际进程不一致。不要只
依赖关闭 RViz/Gazebo 图形窗口来结束验证，因为摄像头和 ROS 节点仍可能继续运行；
应使用 GUI 的停止按钮完成统一清理。launch 的完整日志继续输出在启动 GUI 的终端中。

个人配置也可以通过命令行直接验证：

```bash
ros2 launch linkerhand_bringup mediapipe_rviz_single.launch.py \
  model_id:=o6 side:=left \
  parameters_file:=/完整路径/个人配置.yaml

ros2 launch linkerhand_bringup mediapipe_gazebo.launch.py \
  model_id:=o6 side:=left \
  parameters_file:=/完整路径/个人配置.yaml
```

### 7.3 项目默认左手实测范围

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

### 7.4 项目默认右手实测范围

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

四指调试摘要显示的是 PIP 角度，因此实测范围用于 PIP。MCP 使用独立的
`*_mcp_flexion` 输入和默认映射范围；用户可以按自己的相机视角与手型继续个性化标定。

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
  -> RViz 或 Gazebo
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
ros2 launch linkerhand_bringup mediapipe_rviz.launch.py \
  one_euro_min_cutoff:=0.5 \
  one_euro_beta:=0.4
```

### 8.2 关节死区与迟滞

死区层位于标定映射之后、EMA 之前。每个机械手关节独立判断运动状态：

- 静止时保持锁定角度，小范围噪声不会继续传给 RViz 或 Gazebo。
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

- 调小到 `0.25`：RViz/Gazebo 更稳定，但响应变慢。
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
4. 骨架稳定但仿真手仍有小幅静止抖动时，先调整 `joint_deadband`。
5. 静止稳定但运动过程仍过于敏感时，再把 `filter_alpha` 从 `0.35` 降到
   `0.25~0.30`。

不建议直接叠加卡尔曼滤波。对于“静止抑制噪声、运动保持响应”的手势跟踪，One Euro
通常更容易调。需要速度状态估计、短时预测或遮挡补偿时，可以把卡尔曼作为扩展方案。

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

## 10. Gazebo 手势同步

### 10.1 启动 L30

```bash
ros2 launch linkerhand_gazebo_control mediapipe_gazebo_left.launch.py
ros2 launch linkerhand_gazebo_control mediapipe_gazebo_right.launch.py
```

### 10.2 启动 O6

```bash
ros2 launch linkerhand_bringup mediapipe_gazebo_o6_left.launch.py
ros2 launch linkerhand_bringup mediapipe_gazebo_o6_right.launch.py
```

参数式核心入口：

```bash
ros2 launch linkerhand_bringup mediapipe_gazebo.launch.py \
  model_id:=o6 side:=left
```

摄像头不是 `/dev/video0` 时，例如：

```bash
ros2 launch linkerhand_gazebo_control mediapipe_gazebo_right.launch.py \
  device:=/dev/video2
```

Gazebo 只支持单手运行，不要同时启动两个入口来替代双手模式，因为它们会分别启动
Gazebo 世界和摄像头。L30 两侧已完成 GUI 验收；O6 两侧完成了模型生成、目标注入和
约 60 Hz 状态反馈自动验证，并已完成左右手 GUI 显示与动作验收。

默认加载项目自带的 `linkerhand_demo.sdf`，相机从掌心方向聚焦手掌和五指，并使用
降低高光后的中性光照。Qt 上位机的“Gazebo 验证”和命令行入口使用同一套视角。
需要其他 world 时可以显式覆盖：

```bash
ros2 launch linkerhand_bringup mediapipe_gazebo.launch.py \
  model_id:=l30 side:=left \
  world:=/完整路径/custom.sdf
```

### 10.3 控制链路

```text
摄像头
  -> MediaPipe
  -> linkerhand_retargeting
  -> /left|right/linkerhand/target_joint_states
  -> trajectory_adapter
  -> JointTrajectory
  -> ros_gz_bridge（ROS 到 Gazebo，单向）
  -> OnlineJointController
  -> Gazebo 关节
  -> JointStatePublisher
  -> ros_gz_bridge（Gazebo 到 ROS）
  -> joint_state_throttle
  -> /left|right/joint_states（60 Hz）
```

`trajectory_adapter` 检查关节名称、数组长度和有限值，并按型号 profile 展开完整关节。
L30 从 10 个主动映射展开为 22 个完整关节；O6 从 6 个主动自由度展开为 11 个完整
关节。O6 四指 DIP 是 MCP 的 `0.89` 倍，拇指 IP 左手为 CMC pitch 的 `2.29` 倍、
右手为 `1.86` 倍。Gazebo 插件只接受完整目标，残缺帧不会部分更新模型。

### 10.4 控制原理与边界

每个仿真步使用：

```text
velocity = Kp * (target_position - actual_position)
```

速度被限制在 `3 rad/s`，再根据仿真步长计算下一绝对关节位置。插件持续保持最新目标，
而目标滤波、死区、迟滞、限速和丢失回零仍由上游 `linkerhand_retargeting` 完成。

这是带实际状态反馈的运动学位置同步，不是 `ros2_control` effort PID，也不代表真实
机械手的电机力矩闭环。它面向实时手势显示和交互仿真，完整满足本项目的使用目标。
抓取、接触和力矩分析属于另一类高保真动力学应用，需要额外的简化碰撞几何、惯量、
摩擦、电机和减速器参数标定。

运行时生成的 URDF 不会修改原始左右手模型，它会：

- 添加固定的 `world -> base_footprint`。
- 移除 mimic 约束，由适配器按型号 profile 显式展开从动关节目标。
- 暂时移除会导致相邻指节互锁的高精度 STL collision，但保留 visual。
- 保留 `damping=0.05`，把当前不适合轻量指节的 `friction=0.05` 设为 `0`。
- 质量与惯量使用型号/手侧 profile；L30 左手乘以 `1/7.6`，O6 保持官方比例。

左手惯量缩放可以在纯 Gazebo 控制入口覆盖：

```bash
ros2 launch linkerhand_gazebo_control gazebo_control_left.launch.py \
  inertial_scale:=0.1315789474
```

### 10.5 话题检查

```bash
ros2 topic hz /left/joint_states
ros2 topic hz /right/joint_states
ros2 topic echo /left/joint_states sensor_msgs/msg/JointState --once
ros2 topic echo /right/joint_states sensor_msgs/msg/JointState --once
```

只检查当前正在运行的一侧。正式 `/left|right/joint_states` 应接近 `60 Hz`；
`/left|right/gazebo_joint_states_raw` 是内部高频状态，一般不用于可视化或下游控制。

### 10.6 发布验收与数值记录

- 模型完整显示，手掌底座固定。
- 张开、半握、握拳连续同步，左右方向正确。
- L30 四指 DIP 跟随对应 PIP；O6 DIP 按 `0.89` 倍跟随 MCP。
- O6 拇指侧摆、屈曲和左右不同的 IP 联动方向正确。
- 静止无明显抖动，动作无明显跳变。
- 手移出画面后平滑回零，终端无红色报错。

自动半握测试记录：左手最大误差约 `0.029 rad`（`1.6°`），右手约 `0.013 rad`
（`0.76°`）。回零最大残差分别约 `0.82°` 和 `0.58°`，主要来自拇指 CMC 轴耦合；
MCP、PIP、DIP 基本回到零。该记录对应 L30，左右手 GUI 验收均已通过。

O6 自动目标注入已经确认 6 个主动目标可以驱动 11 个完整关节。O6 左右手的动作幅度、
静止稳定性、拇指侧摆与屈曲联动均已完成 GUI 现场验收，当前版本不再单独调整其
Gazebo 增益。L30/O6 的 RViz 与 Gazebo 实际画面、运动方向和动作同步均已验收通过。

## 11. 常见问题

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
ros2 launch linkerhand_bringup mediapipe_rviz.launch.py \
  processing_fps:=8.0
```

也可以关闭预览：

```text
show_previews:=false
```

### Gazebo 模型不动

先确认目标与桥接话题存在：

```bash
ros2 topic hz /right/linkerhand/target_joint_states
ros2 topic hz /right/gazebo_joint_trajectory
ros2 node list | sort
```

左手测试时把命令中的 `right` 换成 `left`。同时确认终端加载的是当前工作空间：

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/linkerhand_ros2_ws/install/setup.bash
```

### Gazebo 启动后没有摄像头画面

Gazebo 完整入口默认显示 MediaPipe 预览。检查 `DISPLAY` 和摄像头设备，并显式指定：

```bash
ros2 launch linkerhand_gazebo_control mediapipe_gazebo_left.launch.py \
  device:=/dev/video2 mediapipe_show_preview:=true
```

### Gazebo 抓取时手指穿透

这是项目设计边界。运行时 URDF 为避免原始高精度 STL 在相邻指节间互锁，主动移除了
collision，因此 Gazebo 模式用于动作显示而不是抓取。不能通过直接恢复原 STL 碰撞网格
解决；接触仿真应用应另行设计掌部和指节的 box、capsule 等简化碰撞体。

## 12. Git 分支

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
