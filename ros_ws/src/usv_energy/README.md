# usv_energy

中央单机船的工程估算能源模型，作为独立 ament_cmake 包提供可测试的 C++ 核心。`usv_simulation` 的推进插件在每个物理步内同步调用，先分配供电预算、限制轴转矩，再按实际电负载扣除电量。不是异步读取低频转速后随意扣百分比。

- 双 E163 等效并联：326 Ah、51.2 V 标称，初始 SOC 90%；两包均衡假设，无单包断线/环流模型。
- 电流积分 SOC，OCV–SOC 估算曲线、等效电阻压降、I²R 损耗及集总温升。
- 低 SOC / 温度降额、保留电量停推、恢复迟滞；为仿真控制策略，不冒充原厂 BMS。
- 独立启动电池 Ah 预算，柴油机启动/预热/运行/故障，冷却许可和缺油停机。
- 25 A 充电限制、充电效率与近满电减流；发电负载决定耗油。
- 1 Hz 仪表采样、滤波、偏差/噪声、姿态影响油位代理、断流有效位和数据年龄。

配置：`config/energy.json`；`include/usv_energy/Parameters.def` 为算法独立测试的参考默认值。实际 Gazebo 值由 JSON 经 `scripts/prepare_hull.py` 写入 SDF。更改 JSON 后重新运行工作区 `scripts/build.sh`。没有倍速耗电系数；只按仿真时间积分，暂停不耗油电，重置恢复初始工况并关闭发电机。

控制与监视：工作区 `python3 scripts/energy_console.py --generator on --duration 60`。不带控制参数只监视。`--generator off` 停机；`--cooling fault` / `--cooling ok` 与 `--sensors fault` / `--sensors ok` 仅为仿真故障注入。故障机组需要先 off 再 on 复位，恢复冷却许可不会自动重启。控制台退出不改变发电机命令；船舶主机系统保持最后的开/关请求。

Gazebo 话题前缀 `/usv/energy/`：

| 话题 | 类型 | 用途 |
|---|---|---|
| `generator_enable` | gz.msgs.Boolean | 人工开/关发电机 |
| `cooling_ok` | gz.msgs.Boolean | 简化冷却许可，默认 true；不等于真实流量传感器 |
| `sensors_ok` | gz.msgs.Boolean | 仪表失联注入，默认 true |
| `truth` | gz.msgs.Double_V | 物理/控制内部真值与能量账，不能冒充实船测量 |
| `sensors` | gz.msgs.Double_V | 仪表输出、仿真时间、有效标志和数据年龄 |

字段顺序在 `scripts/energy_fields.py`。电池电流真值/仪表为正放电、负充电；机械轴功率见推进话题，电推进输入见 `motor_w`，两者不同。`battery_wh` 是带符号累计母线输出电能，充电时减少，并非剩余能量。剩余 SOC 来自电流积分。`removed_fuel_kg` 是燃油质量变化诊断；当前剩余燃油已通过 `usv_physics` 动态联动 DART 船体质量、重心和完整惯性，见 `/usv/fuel_mass/state`。

`python3 scripts/energy_console.py --ros --duration 60` 可桥接 ROS 2：`/usv/battery`（sensor_msgs/BatteryState，正充电/负放电）、`/usv/fuel_l`（std_msgs/Float64）和 `/usv/energy/diagnostics`。电池值来自仪表话题；无效/超时时输出 NaN 与诊断错误。仿真内部低电/低油及机组状态另列诊断，不能当作已实现原厂 CAN 报文。桥接进程退出即停止发布；下游必须检测超时。时间戳为仿真时间，消费者应与仿真时钟一致。

燃油质量联动细节见 `docs/过程记录/2026-11-19_燃油质量重心惯性联动.md`；准静态内腔仍未实测。

完整参数来源、结果、复现命令及物理边界见工作区 `docs/过程记录/2026-11-19_油电能源与感知验证.md`。

## 舵机与断电联动更新

推进插件内同时求解舵机功率和电机预算。`aux_w` 包含实际舵机负载，不能在能量账中重复相加。`/usv/actuators/state` 字段依次为 time、actual_angle、actual_rate、reference_angle、torque_Nm、servo_W、servo_Wh、payload_powered。无供电时无驱动力矩；公开 ROS 载荷停发由 usv_control 实施，Gazebo 内部真值和 raw 调试通道仍存在。不是所有真实设备电源支路的电路级模拟。
