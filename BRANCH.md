# Right-hand Experience / 右手体验

本分支包含完整工作空间，突出使用 USB 摄像头体验 Linker Hand L30 右手在 RViz 和
Gazebo 中的实时手势同步。完整稳定版本以 `main` 分支为基线。

This branch contains the full workspace and highlights the camera-driven right-hand
Linker Hand L30 experience in RViz and Gazebo. The complete stable baseline is `main`.

## Run / 运行

```bash
cd ~/linkerhand_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch linkerhand_retargeting mediapipe_rviz_right.launch.py

# Gazebo 右手 / Gazebo right hand
ros2 launch linkerhand_gazebo_control mediapipe_gazebo_right.launch.py
```

详细的环境、标定、滤波和故障排查说明见 [GUIDE.md](GUIDE.md)。
See [GUIDE.md](GUIDE.md) for setup, calibration, filtering, and troubleshooting.
