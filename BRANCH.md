# Dual-hand Demo / 双手演示

本分支包含完整工作空间，并默认突出左右手到两套 LinkerHand L30 RViz 模型的同步入口。
通用功能开发仍以 `main` 分支为基线。

This branch contains the full workspace and highlights the synchronized dual-hand
MediaPipe-to-LinkerHand L30 RViz demo. Shared development remains on `main`.

## Run / 运行

```bash
cd ~/linkerhand_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch linkerhand_retargeting mediapipe_rviz_both.launch.py
```

详细的环境、标定、滤波和故障排查说明见 [GUIDE.md](GUIDE.md)。
See [GUIDE.md](GUIDE.md) for setup, calibration, filtering, and troubleshooting.
