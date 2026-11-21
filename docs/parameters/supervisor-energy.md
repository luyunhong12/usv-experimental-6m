# energy.json 参数说明

对应 `ros_ws/src/usv_energy/config/energy.json`。本表解释现有字段，不另存参数数值，也不更改工程假设。
数值、来源与适用范围仍以该配置及其消费者为准；通配符只合并数组内同一字段。

| 参数路径 | 中文说明 |
| --- | --- |
| `/status` | 数据状态和可信度声明，工程估算不能视为实测标定。 |
| `/topology` | 电池、充电器、发电机和启动电源的连接拓扑假设。 |
| `/parameters` | 能源模型参数集合；约束见 Parameters::Validate。 |
| `/parameters/capacity_ah` | 等效并联电池容量，单位 Ah。 |
| `/parameters/nominal_voltage_v` | 电池标称电压，单位 V。 |
| `/parameters/initial_soc` | 初始荷电状态，取值为容量比例。 |
| `/parameters/resistance_ohm` | 等效电池内阻，单位 Ω。 |
| `/parameters/max_discharge_a` | 最大放电电流，单位 A。 |
| `/parameters/cutoff_voltage_v` | 放电截止电压，单位 V。 |
| `/parameters/charge_voltage_v` | 充电电压，单位 V。 |
| `/parameters/reserve_soc` | 停止放电的保留荷电状态阈值。 |
| `/parameters/restart_soc` | 重新允许放电的荷电状态阈值。 |
| `/parameters/derate_soc` | 低荷电状态降额阈值。 |
| `/parameters/low_soc` | 低电量告警阈值。 |
| `/parameters/charge_efficiency` | 充电电量转换效率。 |
| `/parameters/ambient_c` | 环境温度，单位 °C。 |
| `/parameters/initial_temperature_c` | 初始电池温度，单位 °C。 |
| `/parameters/thermal_capacity_j_k` | 等效热容量，单位 J/K。 |
| `/parameters/thermal_conductance_w_k` | 向环境散热的等效热导，单位 W/K。 |
| `/parameters/derate_temperature_c` | 热降额温度，单位 °C。 |
| `/parameters/cutoff_temperature_c` | 过温截止温度，单位 °C。 |
| `/parameters/aux_output_w` | 辅助负载输出功率，单位 W；生成模型按设备台账所选模式计算。 |
| `/parameters/aux_efficiency` | 辅助电源转换效率。 |
| `/parameters/motor_limit_w` | 电机电输入功率限制，单位 W。 |
| `/parameters/motor_efficiency` | 电机效率工程假设。 |
| `/parameters/motor_idle_w` | 电机空载损耗功率，单位 W。 |
| `/parameters/motor_copper_w_per_nm2` | 电机转矩平方铜耗系数，单位 W/(N·m)²。 |
| `/parameters/tank_capacity_l` | 油箱容量，单位 L。 |
| `/parameters/initial_fuel_l` | 初始燃油体积，单位 L。 |
| `/parameters/unusable_fuel_l` | 无法使用的保留燃油体积，单位 L。 |
| `/parameters/low_fuel_l` | 低油量告警体积，单位 L。 |
| `/parameters/diesel_kg_l` | 燃油密度，单位 kg/L。 |
| `/parameters/generator_ac_limit_w` | 发电机交流输出功率限制，单位 W。 |
| `/parameters/fuel_idle_l_h` | 发电机空载耗油率，单位 L/h。 |
| `/parameters/fuel_l_kwh` | 按发电量估算的耗油系数，单位 L/kWh。 |
| `/parameters/crank_s` | 启动机拖动时间，单位 s。 |
| `/parameters/warmup_s` | 发电机预热时间，单位 s。 |
| `/parameters/generator_ramp_s` | 发电机功率爬升时间，单位 s。 |
| `/parameters/charger_current_a` | 充电器输出电流限制，单位 A。 |
| `/parameters/charger_efficiency` | 充电器效率。 |
| `/parameters/starter_capacity_ah` | 启动电池容量，单位 Ah。 |
| `/parameters/starter_initial_soc` | 启动电池初始荷电比例。 |
| `/parameters/starter_crank_a` | 启动过程电流，单位 A。 |
| `/parameters/starter_min_soc` | 允许启动的最小荷电状态。 |
| `/parameters/starter_charge_a` | 启动电池充电电流，单位 A。 |
| `/parameters/sensor_period_s` | 能源传感器采样周期，单位 s。 |
| `/parameters/sensor_filter_s` | 能源测量滤波时间常数，单位 s。 |
| `/parameters/fuel_filter_s` | 燃油测量滤波时间常数，单位 s。 |
| `/parameters/soc_bias` | 荷电状态测量偏置，按比例表示。 |
| `/parameters/fuel_bias_l` | 油量测量偏置，单位 L。 |
| `/sources` | 参考资料列表或索引；引用资料不代表全部参数来自厂家标定。 |
| `/sources/battery_manual` | 电池参考手册链接。 |
| `/sources/charger` | 充电器参考资料链接。 |
| `/sources/generator` | 发电机参考资料链接。 |
| `/limitations` | 当前模型不覆盖或未验证的条件。 |
| `/auxiliary_budget_source` | 辅助功率的权威台账及运行模式说明。 |
