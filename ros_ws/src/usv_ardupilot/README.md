# usv_ardupilot（保留的历史适配器，当前暂停联调）

这是已有的 Gazebo ENU/FLU → ArduRover JSON SITL NED/FRD 本地适配代码，不表示当前船体已经完成飞控验收。本轮只完善 ROS/Gazebo 船体仿真，**不启动或重新接入 ArduPilot/Nav2**。日常启动请用项目根 `./usv run`，详见 [ROS 操船文档](../../../docs/getting_started/README.md)。

适配器使用原生 IMU、真值里程计和能源状态；SERVO1 PWM 对应转向、SERVO3 PWM 对应推进。它仅绑定 localhost，采用软同步而非严格锁步，不是实船硬件接口。传感器/PWM 过期或无供电时停驱，并使用控制权租约避免同时驱动。

旧文档中的 AUTO 三航点、LOITER、RTL、GCS 失联 HOLD 数值属于当时版本的历史试验。后续船体几何、水动力、阻力与执行器已有变化，**这些旧结果不能作为当前版本已通过的证明**。历史原始证据保留，不改写成新版本通过。

未来重新联调需先完成 [接入门槛](../../../docs/integration/README.md)，再核对 ArduPilot 二进制及源码版本、坐标/单位、PWM 中点与方向、仿真时间、正常解锁和失联行为。旧入口已迁至 `ros_ws/src/usv_bringup/scripts/`；本轮不提供绕过门槛的一键飞控启动推荐。ArduPilot 不在本包中，下载其源码也不等于 SITL 已构建或已连接。
