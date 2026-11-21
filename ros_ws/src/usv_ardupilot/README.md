# usv_ardupilot

Gazebo ENU/FLU → ArduRover JSON SITL NED/FRD 的本机适配器。物理数据来自原生 IMU、真值里程计和能源；PWM SERVO1 转向、SERVO3 推进。仅绑定 localhost，不是实船硬件接口。软同步而非严格锁步。

运行悉尼场景后，分别启动 `bash scripts/run_ardupilot.sh --port 9102`、`python3 scripts/run_sitl.py --instance 10`，地面站 TCP 127.0.0.1:5860。如需 ROS 感知使用 `bash scripts/run_interfaces.sh control:=False`。默认 ArduRover 为项目相邻 ardupilot/build/sitl/bin/ardurover，可设 ARDUPILOT_BIN 或 --binary。每个实例 EEPROM/日志位于 log/ardupilot/<profile>/instance_N；--wipe 会重置该实例参数。

正常解锁及前进/倒车/转弯已自动验证；初始化需等待 IMU/EKF 稳定，不要禁用解锁检查。Gazebo reset 后应同步重启飞控与桥。桥心跳中断 1 秒停驱。当前平台的自动任务验收见下文；真实硬件及无限时长运行不在验证范围。

## 平台模式

当前推荐 `bash scripts/run_platform.sh --control ardupilot --wave-gain 0`，它同时启动匹配的 15 km/h 工程船和飞控参数。baseline/platform15 的 EEPROM 和日志分目录存放在 log/ardupilot/<profile>/instance_N。自动验收脚本 validate_navigation.py 在导航就绪后正常解锁，上传任务并检查实际 AUTO、LOITER、RTL 和 GCS 失联行为；没有强制解锁。参数依据及验收范围见当前十五公里时速仿真平台文档。

JSON 桥只在 IMU/里程计/能源及 PWM 都新鲜时发布驱动指令；缺数据、超时或无供电时停驱。仍是软同步，不能替代真实硬件闭环时序。

平台完整自动验收已通过 AUTO 三航点、带水流 LOITER、RTL、GCS 失联 HOLD。最大任务折线距离约 14.065 m，LOITER 稳态阶段最大误差约 4.714 m；不是高精度循迹保证。桥按舵机帧请求应答，步长最多 20 ms，滞后超过 0.1 s 停驱；本次最大步长 12 ms。世界姿态由里程计加机体系角速度外推对齐 IMU 时间，不能直接将初始参考 IMU 四元数当作世界姿态。

平台 `ARMING_RUDDER=0`：满左/满右方向杆只用于转向，不作解锁或解除武装手势。正常解锁检查保持启用，用地面站或 MAVLink 明确解锁/解除武装。已有实例若曾手动保存其他值，需要确认该参数为 0；无需为此清空整个 EEPROM。
