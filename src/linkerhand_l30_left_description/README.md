# Linker Hand L30 左手模型包

该包由 Linker Hand L30 v1.6 左手的 ROS 1 URDF 导出包迁移而来，适用于
ROS 2 Humble，提供模型资源、RViz 2 显示和 Gazebo Sim 生成入口。

## 构建

```bash
cd /home/ubuntu/linkerhand_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## RViz 2 显示

```bash
ros2 launch linkerhand_l30_left_description display.launch.py
```

仅运行状态发布节点，不打开关节界面和 RViz 2：

```bash
ros2 launch linkerhand_l30_left_description display.launch.py use_gui:=false use_rviz:=false
```

## Gazebo Sim 仿真

```bash
ros2 launch linkerhand_l30_left_description gazebo.launch.py
```

仅启动 Gazebo 仿真服务器：

```bash
ros2 launch linkerhand_l30_left_description gazebo.launch.py headless:=true
```

Gazebo 启动文件目前只生成机械结构模型。若要控制真实硬件或驱动仿真关节，
还需要单独配置 `ros2_control` 硬件接口、控制器和仿真插件。
