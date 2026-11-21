# VRX 船体试验

已建立 ament_cmake 包，提供 `UsvHullHydrostatics` 插件和两个工作流：

- 原版默认船本地基线：`scripts/run_baseline.sh`。
- 自定义单机船静水试验：`scripts/run_hull_trial.sh`。

浮力从封闭三角网格的实际浸水体积和浮心计算。已包含静水浮力、未标定线性/二次阻尼及桨轴转矩驱动；不包含实船水动力校准。

诊断 `/usv/hydrostatics`（gz.msgs.Double_V）每行字段：time, volume, cb_x, cb_y, cb_z, buoyancy_N, x, y, z, roll, pitch, yaw。SI 单位，角度为弧度，位置在世界坐标。

转向输入 `/usv/steering/angle`（gz.msgs.Double，弧度）；关节反馈 `/usv/joint_state`（gz.msgs.Model）。关节物理限位 ±35°，参考速度 0.15 rad/s、力矩 100 Nm。推进由实际动力学桨转速计算，正负转矩控制前进/倒车。

验证：`colcon test --packages-select usv_simulation`，以及 `python3 scripts/validate_hull_runtime.py --case equilibrium` / `disturbed` / `steering_contact`。后者需要本机网络/IPC、Gazebo 渲染权限，并会自动停止本次启动的进程。

当前默认质量为约 1443 kg 的单机工程估算。转角输入先经速度 0.15 rad/s、加速度 0.3 rad/s² 的参考轨迹，再进入原力矩控制器；均待实际执行器标定。`--case steering_limits` 验证两端和回中。内部齿轮/滚动体与柔性线束已从刚性碰撞排除，外部离散扫碰报告单独记录。


推进输入 `/usv/propulsion/throttle`（gz.msgs.Double，[-1,1]），至少每秒刷新一次仿真时间；推荐使用 `scripts/drive_boat.py` 持续发送。该输入为含 0.05 死区的归一化轴速请求。

推进诊断 `/usv/propulsion/state`（gz.msgs.Double_V）：time, command, actual_rpm, reference_rpm, thrust_N, motor_torque_Nm, water_torque_Nm, shaft_power_W, wet_fraction, axial_velocity, body_u, body_v, world_x, world_y, yaw, roll, pitch。角度为弧度，速度 m/s，时间为仿真秒。shaft_power 为带符号机械轴功率，不能直接当电池功率。

运行 `python3 scripts/validate_propulsion.py --case forward`，其他工况为 reverse / left / right / reversal，逐个串行运行。输出在 log/propulsion_validation/；不会把未达标结果标为通过。

推进插件现在同步调用 `usv_energy`：每个物理步按电源预算限制轴转矩，再扣实际电能。`shaft_power_W` 仍为机械轴功率，电输入见 `/usv/energy/truth` 的 `motor_w`。能源与仪表接口见 `../usv_energy/README.md`；运行中燃油质量、重心与完整惯性已通过 `usv_physics` 联动；现已耦合舵机耗电、断电停驱及公开 ROS 载荷停发，内部真值保持。

质量诊断 `/usv/fuel_mass/state` 为物理后端读回及整船组合真值；`src/usv_description/config/fuel_tank.json` 定义估算内腔。新场景必须加载 `libUsvPhysics.so`，不能用普通 Physics 插件代替后只观察 ECM 的惯性数字。

当前浮力随 VRX PMS 波面局部切平面更新，阻力相对于水流计算，并含估算风载；Environment.hh 共用上游 Wavefield。steepness 必须为零，无附加质量/拍击/空化模型。新增集成测试、标准 ROS 接口和 ArduPilot 步骤见工作区最新六项整改记录。
