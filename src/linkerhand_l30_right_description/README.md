# Linker Hand L30 右手模型包

该包由 `/home/ubuntu/Downloads/linkerhand_L30_v6_right_urdf-3112` 中的 ROS 1
SolidWorks 导出资源迁移而来，适用于 ROS 2 Humble。包内保留右手独立几何、
惯性、关节轴和限位，并提供 RViz 2 与 Gazebo Sim 启动入口。

## 构建

```bash
cd /home/ubuntu/linkerhand_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select linkerhand_l30_right_description
source install/setup.bash
```

## RViz 2 显示

```bash
ros2 launch linkerhand_l30_right_description display.launch.py
```

仅验证 URDF 和状态发布节点：

```bash
ros2 launch linkerhand_l30_right_description display.launch.py \
  use_gui:=false use_rviz:=false
```

## Gazebo Sim 仿真

```bash
ros2 launch linkerhand_l30_right_description gazebo.launch.py
```

Gazebo 启动文件目前只生成机械结构模型，尚未配置 `ros2_control` 控制接口。
