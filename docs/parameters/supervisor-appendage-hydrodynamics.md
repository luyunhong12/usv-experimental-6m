# appendage_hydrodynamics.json 参数说明

对应 `ros_ws/src/usv_description/config/appendage_hydrodynamics.json`。本表解释现有字段，不另存参数数值，也不更改工程假设。
数值、来源与适用范围仍以该配置及其消费者为准；通配符只合并数组内同一字段。

| 参数路径 | 中文说明 |
| --- | --- |
| `/status` | 数据状态和可信度声明，工程估算不能视为实测标定。 |
| `/link` | 附体或部件所属刚体名称。 |
| `/source_sha256` | 原始几何来源的 SHA-256，用于追溯导出版本。 |
| `/geometry_method` | 原几何轮廓提取方法及坐标系说明。 |
| `/density_kg_m3` | 所属配置使用的介质密度，单位 kg/m³。 |
| `/surfaces` | 现有水下附体的受力轮廓集合。 |
| `/surfaces/*/name` | 部件或表面的名称。 |
| `/surfaces/*/source_object` | 原 Blender 中的对象名称。 |
| `/surfaces/*/outline_xz_m` | 在 azimuth 刚体坐标系下的二维轮廓点，分量为 x、z，单位 m。 |
| `/surfaces/*/area_m2` | 完整侧向轮廓面积，单位 m²；运行时按波面另行裁切。 |
| `/surfaces/*/span_m` | 完整轮廓跨度，单位 m；运行时使用浸水跨度。 |
| `/surfaces/*/profile_drag_coefficient` | 附体型阻系数的工程假设，未做实船标定。 |
| `/surfaces/*/oswald_efficiency` | 有限展弦比模型中的效率系数工程假设。 |
| `/sources` | 参考资料列表或索引；引用资料不代表全部参数来自厂家标定。 |
| `/assumptions` | 附体受力近似、未覆盖效应及重复记账风险说明。 |
