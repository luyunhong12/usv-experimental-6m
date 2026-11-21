# 发布前完整版本复核（2026-11-21）

本次按用户确认提交工作区全部代码、模型参数和验证资料，包含 Supervisor 修复与后续分布式阻力、实体配重工作。
此前修复记录属于较早快照，不替代本次完整版本的检查。

执行 `./usv review --profile deep --run-commands`，报告退出码 0，源码快照一致，全部已配置检查通过。

- 完整构建：12 个包。
- 单元测试：12 个 C++、37 个 Python、5 个设计接口测试通过。
- 原生惯性 force/torque 与推进 forward/reverse/left/right/reversal 共 7 项回归通过。
- 配置选中的核心头文件行覆盖率：97.47%。
- ASan/UBSan：12 项原生测试通过；有限二元算子变异：转向积分 12 项全部检出。

证据：[完整报告](report.txt)、[源码摘要和机器结果](report.json)、[构建与单元测试](usv-build-and-test.log)、[运行回归](usv-runtime-regression.log)。
完整工具产物保留在本机 `artifacts/validation/supervisor/20261121T085410Z-684cca1a`；运行中断的前一次检查没有当作通过证据。

## 复现环境

使用本机 ROS 2 Jazzy/Gazebo Harmonic 依赖，入口见 [ROS 操作指南](../../getting_started/README.md) 和 [Supervisor 说明](../supervisor.md)。
`review` 依赖同级 `code-supervisor/.venv/bin/supervisor` 或 `CODE_SUPERVISOR_BIN`。
本次 Supervisor 基于 27223d3，并使用前序修复的本地 C++ 解析兼容改动；[工具源码摘要](supervisor-tool.json) 记录准确文件版本。
本次仅发布 usv-experimental-6m，不把另一仓库的未提交改动一并推送；仅安装 Supervisor 远端旧提交不等于复现本次工具版本。

## 仍然存在的问题

以 [当前已知问题](../known_issues.md) 和 [分布式阻力/配重复验](../distributed_drag_2026-11-21/README.md) 为准。
持续倒车仍会缓慢转向；新增配重使名义全速约 14.59 km/h，未达到严格 15 km/h；配重安装与结构、实船/CFD 标定、当前配置全海况和长期运行仍需验证。
本次 deep 通过不等于上述工程问题已经解决，也不代表全部上游物理后端分支、全工程覆盖率或实船认证。
