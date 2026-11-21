# 参数来源与边界

权威配置位于 `ros_ws/src/usv_description/config/` 和 `ros_ws/src/usv_energy/config/`；整船默认运行配置位于 `ros_ws/src/usv_bringup/config/platform.json`。

质量与功耗使用 `component_mass_ledger.json`；推进使用 `propulsion.json` 与试验载荷配置；阻力使用 `resistance.json`；能源使用 `energy.json`。不要为了同步文档另维护一套数值。

`datasets/reference/` 是构建/计算输入及派生数据。历史测试原件在 `artifacts/validation/`，其参数快照只代表当时版本。当前基准满油约 1193.56 kg，40 kW 概念方案约 1539.12 kg，另加所选载荷；历史 1443 kg 等数字不是当前默认值。

没有实船数据的参数必须保留工程假设、来源、单位和适用工况。目录整理不会提高模型物理可信度。
