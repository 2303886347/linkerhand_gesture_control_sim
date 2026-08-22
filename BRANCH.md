# Left-hand Demo / 左手演示

本分支用于突出左手 MediaPipe 到 LinkerHand L30 RViz 仿真的完整演示入口。
完整源码与通用开发基线位于 `main` 分支。

This branch highlights the complete left-hand MediaPipe-to-LinkerHand L30 RViz
demo. The full source and shared development baseline remain on `main`.

## Run / 运行

```bash
cd /home/ubuntu/linkerhand_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch linkerhand_retargeting mediapipe_rviz_left.launch.py
```

详细的环境、标定、滤波和故障排查说明见 [GUIDE.md](GUIDE.md)。
See [GUIDE.md](GUIDE.md) for setup, calibration, filtering, and troubleshooting.
