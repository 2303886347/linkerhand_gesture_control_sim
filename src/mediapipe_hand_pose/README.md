# MediaPipe 手部姿态识别

`mediapipe_hand_pose` 是项目的视觉感知包。它订阅 USB 摄像头的 ROS 2 图像，检测
左手或右手的 21 个关键点，计算人体语义关节角，并发布带骨架和角度摘要的调试画面。

## 提供的能力

- 按 `left`、`right` 或 `any` 筛选目标手。
- 输出图像关键点、世界关键点、左右手置信度和人体关节角。
- 调试画面支持自拍镜像，不改变左右手判定或输出数据。
- 3D 关键点短时退化时回退到滤波后的图像关键点。
- 对预览骨架和人体关节角启用 One Euro 自适应滤波。
- 支持限制 MediaPipe 处理帧率，便于在双手模式下控制 CPU 占用。

## 安装 MediaPipe

项目验证版本为 MediaPipe `0.10.9`：

```bash
python3 -m pip install 'mediapipe==0.10.9'
```

## 单独运行感知链路

一键启动摄像头和 MediaPipe：

```bash
ros2 launch mediapipe_hand_pose pipeline.launch.py
```

只启动 MediaPipe，订阅已经存在的 `/usb_camera/image_raw`：

```bash
ros2 launch mediapipe_hand_pose mediapipe.launch.py
```

常用参数：

```bash
ros2 launch mediapipe_hand_pose pipeline.launch.py \
  device:=/dev/video2 \
  target_hand:=left \
  mediapipe_show_preview:=true \
  mirror_preview:=true \
  one_euro_min_cutoff:=0.8 \
  one_euro_beta:=0.3
```

## 输出话题

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `/mediapipe/hand_pose` | `hand_pose_msgs/HandPose` | 完整检测结果 |
| `/mediapipe/human_joint_angles` | `sensor_msgs/JointState` | 人体语义关节角 |
| `/mediapipe/debug_image` | `sensor_msgs/Image` | 带骨架和角度的调试图像 |

角度话题按 ROS 标准使用弧度。四指输出 MCP 侧摆、MCP 屈曲、PIP 和 DIP 屈曲；
拇指输出 CMC 外展、CMC 屈曲、MCP 屈曲和 IP 屈曲。

调试预览会同时显示 `Thumb Abd` 和 `Thumb CMC`，用于 O6 拇指侧摆标定；其中
`Thumb CMC` 对应 `thumb_cmc_flexion`，当前作为 O6 掌面内侧摆的默认输入候选。

## One Euro 滤波

One Euro 在静止时增强平滑，在动作加快时提高响应速度：

- `one_euro_min_cutoff` 越小，静止越稳定，慢速动作延迟越明显。
- `one_euro_beta` 越大，快速动作跟随越灵敏。

关闭滤波进行对照：

```bash
ros2 launch mediapipe_hand_pose pipeline.launch.py use_one_euro_filter:=false
```

完整项目通常通过 `linkerhand_retargeting` 或 `linkerhand_gazebo_control` 的一键入口
启动本包。
