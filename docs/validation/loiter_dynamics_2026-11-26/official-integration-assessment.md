# 官方 Gazebo 接口与桨舵一体模型调查

日期：2026-11-26。状态：源码调查，未完成官方插件构建、迁移或运行验收。

## 当前实现

项目使用开源VRX、Gazebo、ArduPilot；并非没有使用开源方案。区别在于当前SITL连接由`ros_ws/src/usv_ardupilot/scripts/json_bridge.py`实现，尚未使用官方`ArduPilotPlugin`。自有桥承担坐标转换、PWM映射、时序、控制权及部分能耗保护，不能只删除它而忽略这些职责。

本船SDF已有`steering`转动关节和`propeller_spin`关节，属于单套可转向推进器。外形、机械转舵和真实推进力由项目自身模型/插件负责；ArduPilot输出命令，不直接替代水动力模型。

## 官方方案依据

- [ArduPilot官方Gazebo插件](https://github.com/ArduPilot/ardupilot_gazebo)：已下载至被忽略的`dependencies/src/ardupilot_gazebo`，调查固定提交`f116b03552d05c8d0b0bed5ed67033a1183121d1`。源码的`COMMAND`类型发布`gz.msgs.Double`到`cmd_topic`，并跳过直接关节驱动分支；加载时仍要求有效的`jointName`。存在`lock_step`配置，但仅下载源码不等于已启用同步模式。
- [官方SITL/Gazebo接入说明](https://ardupilot.org/dev/docs/sitl-with-gazebo.html)：提供标准接入路径。项目当前JSON接口不能与已淘汰的旧Gazebo后端混为一谈。
- [ArduRover矢量推进说明](https://ardupilot.org/rover/docs/rover-vectored-thrust.html)：支持舵机改变推力方向，与独立尾舵不同。
- [VRX推进器偏转教程](https://github.com/osrf/vrx/wiki/thruster_articulation_tutorial)：WAM-V可以分别控制推进器推力和偏角。项目固定VRX源码的`wamv_gazebo_thruster_config.xacro`也包含发动机转向关节控制与`thrusters/.../pos`话题。双体双推进器仅作为接口参考，不代表与6米单体单推进器具有相同操纵性。

## 建议迁移顺序

1. 保留现有船体、质量惯量、浮力、水动力、推进/舵机动力学及原始验收数据。
2. 固定官方插件源码版本，在独立容器和试验配置内构建；以同一船体、同一正式飞控版本对比原桥和官方接口，避免同时更换船体掩盖原因。
3. 优先评估COMMAND输出接现有推进系统。核对单位、正负号、中位、正反推、坐标系、时钟、传感器、控制权、失联停推及燃油/电量保护；不得由两个接口同时驱动执行器。
4. 验证前进、倒车、左右转向、解锁/上锁与断联行为，再按原25项及3 m保持门槛回归。官方接口是否改善已有问题须由结果证明。
5. 通过后才切换默认入口；旧接口在对照完成前保留。开源WAM-V可另作参考船，不直接替换用户模型，不接Nav2。

尚未找到并验证与本船尺寸、质量、单桨舵机构完全匹配的现成开源船。不能将“存在可偏转推进器示例”表述成“有可直接替换且物理参数可信的整船”。
