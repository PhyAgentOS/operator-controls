# Forge Controls

[English](README.md) | [简体中文](README_zh.md)

提供 Forge/Dora 数据流可复用的关节轨迹控制器与夹爪动作控制器。

- `forge-joint-trajectory-controller`：处理轨迹目标、反馈、结果及取消。
- `forge-gripper-action-controller`：处理夹爪目标、反馈、结果及取消。
- `examples/` 中提供配置、模拟执行器及独立运行示例。

使用 Python 3.12，在根目录执行 `uv sync --frozen --all-packages --dev` 安装工作区。
两个入口均接受 `--config`；使用 `--help` 查看参数。

测试执行 `uv run pytest --import-mode=importlib packages`。
模拟执行器验证软件控制合同，真机连接、运动限制及最终发布包仍需独立验收。

## Forge 2.x 迁移

本分支准备节点版本 `2.0.0`，适配 Dora 1.x，迁移验证锁定 Dora `1.0.1`。使用正式 Forge 2.x 分发包；Common 至少为 `2.1.0`。此次更新包括依赖锁文件与各独立可执行入口。

每轮报告必须列出仓库、改动和测试证据，包括各节点的 `dora run` 记录，以及包含 `adapter-forge-gateway` 的串联测试。缺少硬件或模型的验收项保持待验证。源码 PR 不等于正式发布；整体验收通过后再同步 GitLab。
