# linkerhand_bringup

该包提供 Linker Hand 多型号仿真的统一用户入口。核心 launch 接受左右型号参数，快捷
launch 则覆盖常见组合；具体识别、重定向和 RViz 节点仍由原功能包负责。

```bash
# 参数式核心入口，支持任意已注册的左右组合
ros2 launch linkerhand_bringup mediapipe_rviz.launch.py \
  left_model:=l30 right_model:=o6

# 常用快捷入口
ros2 launch linkerhand_bringup mediapipe_rviz_l30_both.launch.py
ros2 launch linkerhand_bringup mediapipe_rviz_o6_both.launch.py
ros2 launch linkerhand_bringup mediapipe_rviz_l30_o6.launch.py
ros2 launch linkerhand_bringup mediapipe_rviz_o6_l30.launch.py
```

个人标定文件可通过 `left_parameters_file` 和 `right_parameters_file` 显式覆盖。

Qt 个人标定与单手参数式 RViz 入口：

```bash
ros2 launch linkerhand_bringup calibration_gui.launch.py

ros2 launch linkerhand_bringup mediapipe_rviz_single.launch.py \
  model_id:=o6 side:=left parameters_file:=/path/to/personal.yaml
```

Gazebo 使用单手参数式入口或四个快捷入口：

```bash
ros2 launch linkerhand_bringup mediapipe_gazebo.launch.py \
  model_id:=o6 side:=left

ros2 launch linkerhand_bringup mediapipe_gazebo_l30_left.launch.py
ros2 launch linkerhand_bringup mediapipe_gazebo_l30_right.launch.py
ros2 launch linkerhand_bringup mediapipe_gazebo_o6_left.launch.py
ros2 launch linkerhand_bringup mediapipe_gazebo_o6_right.launch.py
```
