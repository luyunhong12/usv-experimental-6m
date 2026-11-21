# 数据分层

- `reference/`：构建和参数计算使用的工作输入；从原数据复制，与历史证据隔离。
- `calibration/`：已有校准与拟合资料。
- `manifests/`：迁移与历史数据清单。

完整原始测试数据在本机 `artifacts/validation/`；旧运行日志在 `artifacts/legacy_runtime_logs/` 和 `legacy_root_logs/`。这些目录默认不提交 Git，但本次全部保留，公开发布时应作为有校验值的数据附件另行提供。
