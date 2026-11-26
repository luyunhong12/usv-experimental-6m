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

## 可选SDF横移/偏航耦合

HullHydrostatics插件支持可选元素`added_sway_yaw`，单位kg·m，默认0保持旧对角模型。它是流体惯性矩阵中A26=A62，不是力矩、阻尼系数或新增实体质量；与`added_linear`、`added_angular`使用同一个`drag_center`参考点。可正可负，必须有限，并满足A26²≤A22·A66；所有对角项有限且非负。固定系数通过加速度项及一致的动量/科氏项共同生效，不能只设置加速度耦合而漏掉动量耦合。

本次未修改默认JSON和SDF的旧参数；几何重算候选及失败证据见[湿剖面重算](../validation/overhaul_2026-11-26/section-hydrodynamics/README.md)。该参数目前只在隔离候选中启用，不表示整船已通过操纵验收。

## 可选固定载荷分离修正

`separation_sections_file`可选，缺省不改变旧模型。其数据格式、同源惯性校验、单位及适用范围见[船体分离修正参数](../validation/overhaul_2026-11-26/hull-separation/README.md#参数与边界)。该项修正已有势流载荷的下游差额，不是任意增加偏航阻尼，也不新增实体鳍。

## 可选分轴空气阻力形心

`air_center_x`、`air_center_y`、`air_center_z`为三个方向空气力各自的作用点，船体FLU坐标，单位m，须有限；每项缺省分别回退到既有`air_center`，保持旧模型行为。沿对应力方向的坐标不改变力矩和该方向局部速度。`air_cd_area`仍是各方向Cd乘投影面积，单位m²，不能把几何面积直接误称为CdA。新实现逐点计入角速度引起的局部相对风速，并在受力审计中保存真实合力矩。

`tools/modeling/derive_windage.py`从当前视觉网格与静态水面裁切产生投影并集，重叠只计一次；形心等于风压中心仅在均匀法向压力近似下成立。默认Cd=1.28来自NASA低速法向平板示例，是公开工程假设而不是本船风洞数据。须检查网格收敛、露出载荷与遮挡。默认船模不自动替换参数。
