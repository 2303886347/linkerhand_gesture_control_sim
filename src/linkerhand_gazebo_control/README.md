# Linker Hand Gazebo 控制

该包把重定向节点输出的独立关节目标转换为
标准 `JointTrajectory` 单点轨迹，再通过 `ros_gz_bridge` 送入项目内置的
Gazebo 在线多关节控制插件，驱动 Gazebo Sim 中的 Linker Hand L30。

第一阶段提供单左手和单右手入口：

```bash
ros2 launch linkerhand_gazebo_control mediapipe_gazebo_left.launch.py
ros2 launch linkerhand_gazebo_control mediapipe_gazebo_right.launch.py
```

无摄像头测试时，可先启动纯 Gazebo 控制链路：

```bash
ros2 launch linkerhand_gazebo_control gazebo_control.launch.py side:=right
```

控制目标和实际反馈分别为：

```text
/right/linkerhand/target_joint_states
/right/joint_states
/right/gazebo_joint_trajectory
```

双手 Gazebo 控制将在单手同步稳定后，结合碰撞模型和资源占用策略单独接入。

当前阶段用于验证手势到关节运动的位置反馈同步。原始 SolidWorks 模型使用高精度
STL 兼作碰撞体，弯曲时会因相邻指节网格接触而锁死，因此 Gazebo 动态生成的
URDF 暂时只保留视觉、惯量和关节限位，不加载碰撞网格。后续进入抓取与接触
仿真时，应为掌部和各指节补充 box/capsule 等简化碰撞体，而不是直接恢复原始
高精度 STL 碰撞网格。

原始模型还为所有可动关节统一填写了 `friction=0.05`。对于质量只有几克的
指节，这会产生明显静态卡滞。动态 URDF 保留 `damping=0.05` 用于抑制振荡，
但将关节库仑摩擦归零；后续拿到真实电机和减速器参数后再做系统辨识与回填。

左手原始导出模型总质量约 `9.81 kg`，而右手约 `1.43 kg`；多数同名活动
指节的质量和惯量约为右手的 `7.6` 倍。这会让同一组动态参数在左手上产生
明显不同的响应。Gazebo 生成器因此默认对左手质量和惯量乘以 `1/7.6`，右手保持
`1.0`。该校正不会修改原始 URDF，并可在启动时覆盖：

```bash
ros2 launch linkerhand_gazebo_control gazebo_control_left.launch.py \
  inertial_scale:=0.1315789474
```

第一阶段由项目内置的 Gazebo System 插件执行运动学位置同步：每个仿真步根据
`velocity = Kp * (target - actual)` 计算限速位置步进，并持续保持最新的 22 关节
目标。输入角度已经由 `linkerhand_retargeting` 完成自适应滤波、死区、迟滞和丢失回零，
因此摄像头移出画面后仍由上游平滑发布零位目标。Gazebo 官方
`JointStatePublisher` 只读取实际状态，原始高频数据经 `ros_gz_bridge` 进入 ROS，
再由节流节点以 `60 Hz` 发布 `/left|right/joint_states`。这样绕开 Humble 版
`ign_ros2_control` 在无命令接口时仍把速度持续写零、多关节 position command
只执行最后一个关节，以及不可信惯性参数在 effort PID 下产生极限环的问题。
该模式用于验证视觉动作同步，不模拟真实电机力矩。后续做接触、抓取和力矩分析
时，需要补充简化碰撞体，并在重新标定惯性、摩擦和电机参数后切换到 effort 闭环。
