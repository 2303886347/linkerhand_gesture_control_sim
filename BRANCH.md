# Left-hand Demo / 左手演示

本分支包含完整工作空间，并默认突出左手 MediaPipe 到 LinkerHand L30 RViz 仿真入口。
通用功能开发仍以 `main` 分支为基线。

This branch contains the full workspace and highlights the left-hand
MediaPipe-to-LinkerHand L30 RViz demo. Shared development remains on `main`.

## Run / 运行

```bash
cd ~/linkerhand_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch linkerhand_retargeting mediapipe_rviz_left.launch.py
```

详细的环境、标定、滤波和故障排查说明见 [GUIDE.md](GUIDE.md)。
See [GUIDE.md](GUIDE.md) for setup, calibration, filtering, and troubleshooting.
