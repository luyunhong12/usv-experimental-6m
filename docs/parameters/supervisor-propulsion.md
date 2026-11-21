# propulsion.json 参数说明

对应 `ros_ws/src/usv_description/config/propulsion.json`。本表解释现有字段，不另存参数数值，也不更改工程假设。
数值、来源与适用范围仍以该配置及其消费者为准；通配符只合并数组内同一字段。

| 参数路径 | 中文说明 |
| --- | --- |
| `/status` | 数据状态和可信度声明，工程估算不能视为实测标定。 |
| `/rated_electrical_power_W` | 基准电机额定电输入功率，单位 W。 |
| `/motor_efficiency_assumed` | 电机效率的工程假设，不是效率地图。 |
| `/forward_static_thrust_N` | 正向静推力参考值，单位 N。 |
| `/reverse_static_ratio` | 反向静推力相对正向的比例假设。 |
| `/reference_rpm` | 推力模型参考桨轴转速，单位 rpm。 |
| `/reference_diameter_m` | 参考螺旋桨直径，单位 m。 |
| `/advance_zero_thrust_m_s` | 推进模型中推力降为零的前进速度，单位 m/s。 |
| `/motor_stall_torque_Nm` | 电机堵转转矩限制，单位 N·m。 |
| `/speed_control_gain_Nm_per_rad_s` | 轴速误差转为转矩的增益，单位 N·m/(rad/s)。 |
| `/speed_reference_tau_s` | 轴速请求的滤波时间常数，单位 s。 |
| `/throttle_deadband` | 归一化轴速请求的油门死区。 |
| `/command_timeout_s` | 控制命令超时时间，单位 s。 |
| `/water_level_m` | 静水面高度参考，单位 m。 |
| `/basis` | 参数或几何推导依据，包含估算方法与限制。 |
| `/basis/factory_proxy` | 厂家产品参考链接；当前桨型与参考产品可能不同。 |
| `/basis/factory_proxy_values` | 采用的厂家参考量和工程换算假设。 |
| `/basis/unverified` | 尚需辨识或实测确认的参数范围。 |
| `/basis/throttle_semantics` | 油门与轴速请求的对应约定，不把油门等同功率比例。 |
| `/basis/scope` | 当前模型适用工况与能力边界。 |
