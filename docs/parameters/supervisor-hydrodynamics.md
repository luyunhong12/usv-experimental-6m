# hydrodynamics.json 参数说明

对应 `ros_ws/src/usv_description/config/hydrodynamics.json`。本表解释现有字段，不另存参数数值，也不更改工程假设。
数值、来源与适用范围仍以该配置及其消费者为准；通配符只合并数组内同一字段。

| 参数路径 | 中文说明 |
| --- | --- |
| `/status` | 数据状态和可信度声明，工程估算不能视为实测标定。 |
| `/added_linear_kg` | 三轴平动附加质量的工程近似，单位 kg。 |
| `/added_angular_kg_m2` | 三轴转动附加惯性的工程近似，单位 kg·m²。 |
| `/reference` | 附加质量近似的参考说明。 |
| `/basis` | 参数或几何推导依据，包含估算方法与限制。 |
| `/source` | 该项采用的资料来源或来源链接。 |
| `/max_step_s` | 对应模型建议或验证的最大时间步长，单位 s。 |
| `/limitations` | 当前模型不覆盖或未验证的条件。 |
