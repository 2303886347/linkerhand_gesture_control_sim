# hand_pose_msgs

`hand_pose_msgs` 是项目感知层与角度重定向层之间的标准 ROS 2 消息包。它把
MediaPipe 的检测状态、左右手结果、21 个关键点和人体语义关节角封装为稳定接口，
使感知算法和 Linker Hand 模型保持解耦。

## 消息

### `HandLandmark`

表示一个三维关键点。图像关键点使用归一化图像坐标，世界关键点使用 MediaPipe
世界坐标。

### `HandPose`

包含：

- 图像时间戳和坐标系。
- 当前帧是否检测到目标手。
- 左右手标签、置信度和单帧处理耗时。
- MediaPipe 21 个图像关键点和 21 个世界关键点。
- 人体语义关节角名称与弧度值。

该消息由 `mediapipe_hand_pose` 发布，由 `linkerhand_retargeting` 订阅。消息内部角度
遵循 ROS 约定使用弧度；面向用户的标定配置可以使用度数。

## 构建

```bash
colcon build --symlink-install --packages-select hand_pose_msgs
source install/setup.bash
```
