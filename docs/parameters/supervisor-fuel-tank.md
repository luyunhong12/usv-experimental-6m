# fuel_tank.json 参数说明

对应 `ros_ws/src/usv_description/config/fuel_tank.json`。本表解释现有字段，不另存参数数值，也不更改工程假设。
数值、来源与适用范围仍以该配置及其消费者为准；通配符只合并数组内同一字段。

| 参数路径 | 中文说明 |
| --- | --- |
| `/status` | 数据状态和可信度声明，工程估算不能视为实测标定。 |
| `/inner_size_m` | 油箱内部长宽高估计，单位 m。 |
| `/center_m` | 油箱中心位置，单位 m，模型坐标系。 |
| `/density_kg_m3` | 所属配置使用的介质密度，单位 kg/m³。 |
| `/reference_fuel` | 用于扣除旧静态燃油记账的参考燃油质量属性。 |
| `/reference_fuel/mass_kg` | 该部件或参考燃油的质量，单位 kg。 |
| `/reference_fuel/cg_m` | 质量中心坐标，单位 m，按文件 coordinate_frame 或所属模型约定。 |
| `/reference_fuel/inertia_kg_m2` | 关于该质量中心的对称惯性张量，单位 kg·m²。 |
| `/basis` | 参数或几何推导依据，包含估算方法与限制。 |
| `/surface_model` | 油箱准静态自由液面模型及忽略的动态效应。 |
| `/mass_flux_model` | 燃油消耗的质量流假设与动量处理边界。 |
