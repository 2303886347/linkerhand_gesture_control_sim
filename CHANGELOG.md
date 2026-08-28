# Changelog

本项目采用语义化版本号。各 ROS 2 包的资源版本以对应 `package.xml` 为准。

## [1.0.0] - 2026-08-28

首个稳定版本，完成普通 USB 摄像头驱动 Linker Hand L30/O6 仿真显示的完整链路。

### Added

- USB 摄像头 ROS 2 图像发布与 MediaPipe 左右手关键点、语义关节角检测。
- One Euro、关节死区/迟滞、EMA、速度限制和检测丢失平滑回零。
- L30/O6 左右手型号 profile、角度重定向及 O6 六主动自由度适配。
- L30/O6 单手、同型号双手和任意混合型号双手 RViz 2 显示。
- L30/O6 单左手、单右手 Gazebo Sim 运动学位置同步与 60 Hz 状态反馈。
- Qt 个人标定上位机，支持四姿态手动采样、质量检查、重采和个人 YAML 管理。
- Qt 一键 RViz/Gazebo 验证、摄像头占用交接和验证进程清理。
- 项目自带 Gazebo world，提供掌心近景、中性光照和自定义 world 覆盖参数。
- 中英双语主页、中文调试运行手册、贡献指南和 GitHub 问题模板。

### Validated

- L30/O6 左右手及全部 RViz 型号组合的显示、方向、动作和静止稳定性。
- L30/O6 单手 Gazebo 的张开、半握、握拳、拇指联动和丢失回零。
- Qt 标定、配置生成/更新/导入，以及 RViz/Gazebo 一键验证完整流程。
- 自动测试基线：`176 tests, 0 errors, 0 failures, 0 skipped`。

### Scope

Gazebo 模式用于实时动作显示和交互仿真，采用运动学位置同步。接触抓取、真实电机、
力矩闭环和高保真动力学不属于 `v1.0.0` 的交付范围。

[1.0.0]: https://github.com/2303886347/linkerhand_gesture_control_sim/releases/tag/v1.0.0
