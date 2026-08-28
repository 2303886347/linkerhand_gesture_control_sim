# Dual-hand Experience / 双手体验

本分支包含完整工作空间，突出使用一个 USB 摄像头同时识别左右手，并在同一个 RViz 2
中体验两套 Linker Hand L30 模型的实时同步。完整稳定版本以 `main` 分支为基线。

This branch contains the full workspace and highlights a shared-camera dual-hand
Linker Hand L30 experience in one RViz 2 instance. The complete stable baseline is `main`.

## Run / 运行

```bash
cd ~/linkerhand_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch linkerhand_retargeting mediapipe_rviz_both.launch.py
```

详细的环境、标定、滤波和故障排查说明见 [GUIDE.md](GUIDE.md)。
See [GUIDE.md](GUIDE.md) for setup, calibration, filtering, and troubleshooting.
