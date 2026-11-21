# component_mass_ledger.json 参数说明

对应 `ros_ws/src/usv_description/config/component_mass_ledger.json`。本表解释现有字段，不另存参数数值，也不更改工程假设。
数值、来源与适用范围仍以该配置及其消费者为准；通配符只合并数组内同一字段。

| 参数路径 | 中文说明 |
| --- | --- |
| `/schema` | 设备台账结构版本。 |
| `/status` | 数据状态和可信度声明，工程估算不能视为实测标定。 |
| `/accounting_coverage_complete` | 是否声称设备记账条目覆盖完整；不等于实测覆盖完整。 |
| `/physical_measured_coverage_complete` | 实测质量及惯性属性是否完整覆盖。 |
| `/coordinate_frame` | 位置和张量采用的坐标轴及参考点约定。 |
| `/active` | 是否启用这份设备台账。 |
| `/normal_power_mode` | 默认辅助负载工作模式。 |
| `/replaces_legacy_residual_kg` | 由逐项台账替代的旧残余质量预算，单位 kg。 |
| `/fuel_accounting` | 燃油单独联动并避免重复计入干质量的规则。 |
| `/propeller_accounting` | 不同方案中桨几何与质量惯性缩放的记账规则。 |
| `/power_exclusions` | 不重复计入静态辅助功率的动态负载类别。 |
| `/parts` | 逐项设备的质量、重心、惯性及功耗台账。 |
| `/parts/*/name` | 部件或表面的名称。 |
| `/parts/*/link` | 附体或部件所属刚体名称。 |
| `/parts/*/mass_kg` | 该部件或参考燃油的质量，单位 kg。 |
| `/parts/*/cg_m` | 质量中心坐标，单位 m，按文件 coordinate_frame 或所属模型约定。 |
| `/parts/*/inertia_at_cg_kg_m2` | 以部件重心为原点、模型轴方向表达的惯性张量，单位 kg·m²。 |
| `/parts/*/basis` | 参数或几何推导依据，包含估算方法与限制。 |
| `/parts/*/quality` | 该条目依据的质量类别，如厂家质量、几何推导或工程估算。 |
| `/parts/*/id` | 台账条目的唯一标识。 |
| `/parts/*/profiles` | 该条目适用的方案名称集合。 |
| `/parts/*/mass_range_kg` | 轻重敏感性工况采用的质量范围，单位 kg。 |
| `/parts/*/cg_uncertainty_m` | 重心位置不确定性的工程估计，单位 m。 |
| `/parts/*/power` | 部件的功率分类与占空比预算。 |
| `/parts/*/power/class` | 部件在功率记账中的类别。 |
| `/parts/*/power/idle_w` | 空闲功率，单位 W。 |
| `/parts/*/power/rated_w` | 额定或工作功率，单位 W。 |
| `/parts/*/power/duty` | 各运行模式的工作占空比。 |
| `/parts/*/power/duty/normal` | 正常运行模式占空比。 |
| `/parts/*/power/duty/survey` | 测量任务模式占空比。 |
| `/parts/*/power/duty/peak` | 峰值模式占空比。 |
| `/parts/*/coverage` | 条目覆盖与旧预算的归属说明。 |
| `/parts/*/source` | 该项采用的资料来源或来源链接。 |
| `/parts/*/size_m` | 部件包络长宽高，单位 m，用于估算几何与惯性。 |
| `/parts/*/power/classification` | 条目的来源或工程分类标识。 |
| `/coverage_note` | 整体台账覆盖范围与未测量项说明。 |
| `/parts/*/geometry` | 新增实体的显示及碰撞包络；质量已在所属刚体台账合并，不在几何生成时再加质量。 |
| `/parts/*/geometry/type` | 包络形状，目前支持 box。 |
| `/parts/*/geometry/size_m` | 长方体三轴尺寸，单位 m，用于质量—体积及惯性一致性验证。 |
| `/parts/*/geometry/material_density_kg_m3` | 工程材料密度，单位 kg/m³；与体积相乘应等于该部件名义质量。 |
