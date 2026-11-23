# 参数来源与边界

权威配置位于 `ros_ws/src/usv_description/config/` 和 `ros_ws/src/usv_energy/config/`；整船默认运行配置位于 `ros_ws/src/usv_bringup/config/platform.json`。

质量与功耗使用 `component_mass_ledger.json`；推进使用 `propulsion.json` 与试验载荷配置；阻力使用 `resistance.json`；能源使用 `energy.json`。不要为了同步文档另维护一套数值。

`datasets/reference/` 是构建/计算输入及派生数据。历史测试原件在 `artifacts/validation/`，其参数快照只代表当时版本。当前基准满油约 1230.78 kg，40 kW 概念方案约 1576.34 kg（升级电池已重布置、仅保留横向配重，见 [配平记录](../validation/layout_2026-11-21/README.md)）；两方案另增约 5.17 kg 被动稳定鳍，见 [倒车恢复设计](../validation/reverse_recovery_2026-11-23/README.md)，另加所选载荷。历史 1443 kg、1225.61 kg、1571.17 kg 等数字不是当前默认值。

没有实船数据的参数必须保留工程假设、来源、单位和适用工况。目录整理不会提高模型物理可信度。

## 机器检查参数表

Supervisor 已关联两个主 SDF，以及能源、推进、油箱、附体、设备台账、附加质量和阻力 JSON 的逐字段说明。具体文件见根目录 `supervisor.toml` 的 `settings.parameter-docs.documentation`。
这些表不复制数值、不提高参数可信度；其余配置仍未纳入逐字段检查。

阻力系数已统一放到 `resistance.json`：旧纵向总预算通过 `tools/lib/resistance_budget.py` 分配给船体剩余项和显式附体型阻。`propulsion.json` 不再保存船体二次阻力。该分账是工程约定，横向及角向分解仍未标定。详见 [受力与预算报告](../validation/force_maneuvers_2026-11-21/README.md)。
