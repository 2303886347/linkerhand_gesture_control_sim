# MediaPipe 手部姿态识别

该包订阅现有 USB 摄像头 ROS 2 图像话题，通过 MediaPipe Hands 输出手部
关键点、人体语义关节角和标注图像。感知结果不绑定具体机械手关节，后续由
独立的角度映射与滤波节点转换为 Linker Hand 控制目标。

## 依赖

当前环境使用 MediaPipe 0.10.9。由于 ROS 2 Humble 的 rosdep 没有
`python3-mediapipe` 规则，需要通过 pip 提供：

```bash
python3 -m pip install 'mediapipe==0.10.9'
```

## 构建

```bash
cd /home/ubuntu/linkerhand_ros2_ws
colcon build --symlink-install --packages-up-to mediapipe_hand_pose
source install/setup.bash
```

## 一键启动摄像头和识别

```bash
ros2 launch mediapipe_hand_pose pipeline.launch.py
```

默认只显示 MediaPipe 标注窗口，不显示未处理的摄像头窗口。分别控制两个窗口：

```bash
ros2 launch mediapipe_hand_pose pipeline.launch.py \
  camera_show_preview:=false mediapipe_show_preview:=true
```

标注窗口和 `/mediapipe/debug_image` 默认以自拍镜像方式显示。该镜像只影响调试
画面，不改变原始摄像头话题、左右手判定和关节角数据。需要原方向显示时：

```bash
ros2 launch mediapipe_hand_pose pipeline.launch.py mirror_preview:=false
```

无预览窗口运行：

```bash
ros2 launch mediapipe_hand_pose pipeline.launch.py mediapipe_show_preview:=false
```

只启动 MediaPipe 节点，使用已经存在的摄像头话题：

```bash
ros2 launch mediapipe_hand_pose mediapipe.launch.py
```

## 输出话题

- `/mediapipe/hand_pose`：完整 `hand_pose_msgs/HandPose` 感知结果。
- `/mediapipe/human_joint_angles`：检测成功时发布标准 `sensor_msgs/JointState`。
- `/mediapipe/debug_image`：带关键点、置信度、耗时和角度摘要的图像。

角度统一使用弧度。四指分别输出 MCP 侧摆、MCP 屈曲、PIP 屈曲和 DIP
屈曲；拇指输出 CMC 外展、CMC 屈曲、MCP 屈曲和 IP 屈曲，共 20 个角度。

角度计算优先使用 MediaPipe 的 3D `world_landmarks`。半握遮挡导致 3D 几何
暂时退化、但图像骨架仍有效时，节点会自动回退到滤波后的图像关键点，避免发布
空角度并让下游误触发安全回零。

## One Euro 消抖

默认对预览骨架关键点和最终人体关节角启用 One Euro 自适应低通。静止时会
抑制摄像头和 MediaPipe 的小幅抖动，动作加快时会自动降低平滑强度，减少跟手
延迟。常用调节参数：

- `one_euro_min_cutoff`：越小越稳定，但慢速动作延迟越明显，默认 `0.8`。
- `one_euro_beta`：越大越能快速跟随剧烈动作，默认 `0.3`。

对比关闭滤波的效果：

```bash
ros2 launch mediapipe_hand_pose pipeline.launch.py use_one_euro_filter:=false
```
