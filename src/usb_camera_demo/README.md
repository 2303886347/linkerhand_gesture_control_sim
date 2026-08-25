# USB 摄像头采集

`usb_camera_demo` 是完整手势仿真链路的图像输入包。节点通过 V4L2 打开普通 USB
摄像头，将画面发布为 ROS 2 `sensor_msgs/Image`，并可显示 OpenCV 预览窗口。

## 启动

```bash
ros2 launch usb_camera_demo usb_camera.launch.py
```

默认设备和输出：

```text
设备：/dev/video0
分辨率：640 x 480
帧率：30 FPS
图像话题：/usb_camera/image_raw
```

指定其他摄像头或采集参数：

```bash
ros2 launch usb_camera_demo usb_camera.launch.py \
  device:=/dev/video2 width:=1280 height:=720 fps:=30.0
```

关闭本包自己的原始画面预览：

```bash
ros2 launch usb_camera_demo usb_camera.launch.py show_preview:=false
```

完整 RViz/Gazebo 启动入口会自动启动本包，普通使用不需要单独运行它。

## 检查

```bash
ls -l /dev/video*
ros2 topic hz /usb_camera/image_raw
```

## 构建

```bash
colcon build --symlink-install --packages-select usb_camera_demo
source install/setup.bash
```
