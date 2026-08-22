# USB 摄像头最小示例

该 ROS 2 Humble 节点通过 V4L2 打开 USB 摄像头，将画面发布到
`/usb_camera/image_raw`，并可选择显示 OpenCV 预览窗口。

## 构建

```bash
cd /home/ubuntu/linkerhand_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select usb_camera_demo
source install/setup.bash
```

## 启动

默认打开 `/dev/video0`，请求 640×480 分辨率并显示预览窗口。在预览窗口中
按 `q` 可以停止节点。

```bash
ros2 launch usb_camera_demo usb_camera.launch.py
```

指定其他设备、分辨率和帧率：

```bash
ros2 launch usb_camera_demo usb_camera.launch.py \
  device:=/dev/video2 width:=1280 height:=720 fps:=30.0
```

不显示预览窗口：

```bash
ros2 launch usb_camera_demo usb_camera.launch.py show_preview:=false
```

检查图像发布频率：

```bash
ros2 topic hz /usb_camera/image_raw
```
