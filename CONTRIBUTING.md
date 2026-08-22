# 贡献指南 / Contributing Guide

## 中文

1. 从最新 `main` 创建功能分支。
2. 保持左右手功能对称，公共算法不要复制到多个分支。
3. Python 与 launch 文件使用英文标识符，注释、日志和用户文档优先使用中文。
4. 修改角度映射时，说明输入来自 MCP、PIP 或拇指关节中的哪一项。
5. 修改 TF 或 topic 时，验证左、右、双手三种入口。
6. 提交前运行：

```bash
colcon build --symlink-install --packages-up-to linkerhand_retargeting
colcon test --packages-select mediapipe_hand_pose linkerhand_retargeting
colcon test-result --verbose
```

Pull Request 应包含目的、行为变化、验证命令和必要的 RViz/MediaPipe 截图。

## English

1. Branch from the latest `main`.
2. Keep left and right behavior symmetric; do not duplicate shared algorithms.
3. Use English identifiers. Existing user-facing comments, logs, and operational docs are Chinese-first.
4. Identify whether calibration values describe MCP, PIP, thumb MCP, or thumb IP motion.
5. Validate the left, right, and dual-hand launchers after topic or TF changes.
6. Include the purpose, behavior change, test commands, and relevant screenshots in the pull request.
