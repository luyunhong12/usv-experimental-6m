# resistance.json 参数说明

对应 `ros_ws/src/usv_description/config/resistance.json`。本表解释现有字段，不另存参数数值，也不更改工程假设。
数值、来源与适用范围仍以该配置及其消费者为准；通配符只合并数组内同一字段。

| 参数路径 | 中文说明 |
| --- | --- |
| `/status` | 数据状态和可信度声明，工程估算不能视为实测标定。 |
| `/enabled` | 是否启用按几何缩放的阻力配置。 |
| `/reference_profile` | 阻力基准对应的方案名称。 |
| `/reference_pose_z_roll_pitch` | 基准姿态，依次为高度 m、滚转 rad、俯仰 rad。 |
| `/reference_wet_area_m2` | 基准浸水面积，单位 m²。 |
| `/reference_projected_area_m2` | 基准各轴投影面积，单位 m²。 |
| `/geometry_sha256` | 阻力几何依据的 SHA-256。 |
| `/law` | 当前采用的阻力缩放关系说明。 |
| `/basis` | 参数或几何推导依据，包含估算方法与限制。 |
| `/sources` | 参考资料列表或索引；引用资料不代表全部参数来自厂家标定。 |
| `/limitations` | 当前模型不覆盖或未验证的条件。 |
| `/legacy_quadratic_drag_N_s2_m2` | 保留的原平动二次阻尼系数，单位 N·s²/m²。 |
| `/hull_quadratic_rotational_drag_Nm_s2_rad2` | 船体转动二次阻尼系数，单位 N·m·s²/rad²。 |
| `/allocation_policy` | 集中阻力与新增附体型阻的分配及重复记账边界。 |
| `/reference_scene` | 生成固定阻力参考几何时使用的场景记录路径，用于追溯基准载荷、油量与姿态。 |
| `/distributed_lateral_drag` | 启用按实际浸水表面积（线性项）及侧投影面积（二次项）积分的侧向阻力，同时替代旧集中 Y 阻力和独立偏航阻尼，避免重复计力；默认 true。 |
| `/lateral_distribution_policy` | 现有侧向预算到面片的分配方法、固定参考面积及旧偏航参数仅供关闭模式对照的边界。 |
