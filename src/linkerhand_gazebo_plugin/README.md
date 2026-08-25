# Linker Hand Gazebo 在线关节插件

`linkerhand_gazebo_plugin` 是项目的 Gazebo System 插件。它订阅标准
`ignition.msgs.JointTrajectory`，持续保持最新的完整关节目标，并在每个仿真步执行
带比例增益和最大速度限制的位置同步。

## 行为

- 在配置阶段解析并验证全部受控关节。
- 只接受名称和位置数量一致、数值有限的完整目标。
- 忽略不属于模型的额外关节名称。
- 使用 `velocity = Kp * position_error` 计算位置步进。
- 使用 `max_velocity` 限制单关节最大响应速度。
- 通过 `ResetPosition` 更新位置并清除残余动力学速度。

插件由 `linkerhand_gazebo_control` 在运行时 URDF 中自动加载，普通用户无需直接配置。

## 环境

包安装的 `.dsv` hook 会把插件目录加入：

```text
GZ_SIM_SYSTEM_PLUGIN_PATH
IGN_GAZEBO_SYSTEM_PLUGIN_PATH
```

构建后执行 `source install/setup.bash` 即可被 Gazebo Sim 6 发现。

## 项目边界

该插件实现运动学位置同步，不是基于真实电机参数的 effort 控制器。它用于实时手势
可视化，力矩、接触和抓取物理不属于其职责。
