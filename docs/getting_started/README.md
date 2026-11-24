# 用 ROS 2 在 VRX 中开船

> 2026-11-24 环境提醒：本机 Gazebo 依赖已升级，旧构建缓存当前不能重新构建。请先核对 [依赖漂移记录](../validation/known_issues.md#2026-11-24-本机依赖漂移)；SITL 联调改用固定版本隔离环境，不能将历史本机验收视为当前环境保证。


当前推荐入口是 **ROS 控制 + Gazebo/VRX 悉尼场景**，不需要安装或启动 ArduPilot、SITL 或 Nav2。本轮接入前验证见 [四步验收记录](../validation/platform_readiness_2026-11-21/README.md)，物理边界见 [已知问题](../validation/known_issues.md)。

## 环境与构建

验证环境为 Ubuntu、ROS 2 Jazzy、Gazebo Harmonic；自定义物理后端依赖 gz-sim **8.15.0**、gz-physics **7.8.0** 与系统 DART。其他版本不能直接视为兼容。首次克隆还需按 [依赖说明](../../dependencies/README.md) 获取固定版本 VRX、Sydney 素材和 Python 依赖，并取回 Git LFS 模型。已验证的独立容器安装见 [容器复现指南](container.md)；宿主机图形安装仍不是已验证的一键安装器。

在项目根目录运行：

```bash
./usv build
./usv test
```

构建成功会保存 `ros_ws/log/build_binding.json`。修改 C++ 或安装库后必须重新构建；不要在构建或航行验收期间修改源文件。单元测试通过不等于全部航行品质通过。

## 终端一：打开悉尼水面

```bash
# 项目根目录；图形窗口，使用默认工程风浪配置
./usv run
```

等到终端出现 `Boat ready: physics, energy, actuators, IMU and GPS are fresh.` 再操船。第一次加载场景可能较慢。需要先观察平静水面的运动时，用以下命令替代上面的启动命令：

```bash
./usv run --wave-gain 0
# 无图形窗口的物理运行（与上一条二选一）
./usv run --wave-gain 0 headless:=True
```

默认工程动力配置为 40 kW，15 km/h 是设计目标，不能把所有载荷、油量、风浪下的速度都当成 15 km/h。一个分区只启动一套船；关闭时在启动终端按 Ctrl+C，让启动器清理子进程。

## 终端二：前进、倒车、转向

同样进入项目根目录。下面每条命令单独执行，运行时间结束后会发送零油门、零舵角。

```bash
# 小油门前进 10 秒
./usv drive --throttle 0.3 --duration 10
# 低速倒车；60% 倒车已有仿真验收，仍不自动保持航向
./usv drive --throttle -0.25 --duration 10
# 前进左转：当前安装坐标下负舵角
./usv drive --throttle 0.3 --steering-deg -10 --duration 10
# 前进右转
./usv drive --throttle 0.3 --steering-deg 10 --duration 10
# 显式停推、回零舵
./usv drive --throttle 0 --steering-deg 0 --duration 2
```

`--throttle` 范围为 -1～1，是归一化推进指令，并非直接指定速度或功率百分比；`--steering-deg` 范围为 -35～35 度，是桨舵一体关节目标角。倒车时同一舵角的船头转向与前进相反。命令退出、Ctrl+C 或失联会停推，**不是瞬间刹停**，船仍会随惯性、风浪和水流移动。只保留一个手动指令发布端。

## 看能源和 ROS 状态

```bash
./usv energy --duration 30
```

`usv` 入口会自行加载 ROS 环境；直接使用 ROS 命令时，在每个终端执行：

```bash
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
ros2 topic list
ros2 topic echo /usv/control/health
```

默认 Gazebo 分区是 `usv_platform15`。如果自定义，启动、操船、能源查看三个终端都先设置相同的值，例如 `export GZ_PARTITION=my_usv`；`ROS_DOMAIN_ID` 也必须一致。Gazebo 分区不能代替 ROS domain 对多个同名 `/usv` ROS 接口的隔离。

## 常见问题

- **No ready ROS controller**：检查启动终端是否已经 ready、两端分区和 ROS domain 是否一致、Gazebo 是否暂停；健康日志中的 command_connected 检查油门/舵角物理订阅端，direct_command_confirmed 检查实际油门回执；不要通过忽略就绪检查硬发指令。
- **缺船体或悉尼资产**：核对 Git LFS 和依赖下载，先看启动日志中的资源路径错误；参照依赖说明恢复资源后重新构建。
- **无法初始化 Transport/DDS**：本地 Gazebo/ROS 需要网络、组播和共享内存权限；受限沙箱可能无法运行，使用有相应权限的本地终端。
- **启动一半退出**：看原终端错误和 `ros_ws/log/platform-ready-<分区>.json`。不要在同一分区再启动一套去掩盖旧进程。
- **船没有一直直走**：直接油门/舵角模式不锁定航向，转子反扭矩、质量偏置、风浪流及反向水动力都会影响航迹。当前操纵限制见验收记录。

## 可复现操纵验收

```bash
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
python3 tests/regression/run_performance_suite.py \
  --spec tests/scenarios/maneuver_audit.json \
  --output artifacts/my_maneuver_run --workers 3
python3 tests/regression/assess_maneuvers.py \
  --spec tests/scenarios/maneuver_audit.json \
  --criteria tests/scenarios/maneuver_acceptance.json \
  --output artifacts/my_maneuver_run
```

使用不存在的新结果目录。只有源码、安装库、配置、工况与证据摘要完全相同时才允许加 `--resume` 复用；失败或中断结果保留，重试另开目录。第二条命令返回非零意味着操纵品质存在不合格项，即使运行脚本成功也不能忽略。历史数据没有本轮绑定清单，不可直接当成当前版本证据。

## 固定配重与当前方案

基准方案保留约 32.05 kg 横向板；`speed15_concept` 采用升级电池前后重布置，取消上一版约 167.86 kg 纵向板。上述布置加上当前约 5.17 kg 的稳定鳍后，满油总质量约 1576.34 kg，静水轻载全速约 15.03 km/h。显示、碰撞、质量与惯性由台账同步生成；实际安装、线缆与承载能力尚需核实。倒车仍有配平回转，详见 [布置复验](../validation/layout_2026-11-21/README.md)。


## 当前接触精度与运行成本

当前生成场景使用 1 ms 物理步长，保留原船体和薄鳍几何；相比旧 4 ms 配置需要更多 CPU。
仿真时间与墙钟时间可能不同，不能把运行 20 分钟墙钟当作已完成 20 分钟仿真。
接触数值参数、精度边界及长时证据见 [接触与长时专项](../validation/contact_endurance_2026-11-23/README.md)。

原生性能测试器会独占租用本机 ROS 测试域（120–159），避免不同 spec 相同工况索引串扰。
外部手动 ROS 节点不要占用该范围；这不是对任意其他主机或独立容器的域协调。
碰撞验证建议通过 `validate_contact_directions.py --case stabilizer_side --output artifacts/my_contact_side`
逐位置执行；按每次实际源码、插件回读、仿真时间和完整采样判定结果。


## 相机、点云与本机传输

完整悉尼场景默认包含三路 640×360 RGB 相机，以及 360×16 三维点云和二维扫描，均为 5 Hz **仿真时间**频率。无 GUI 的 `headless:=True` 仍启用 EGL/OGRE2 渲染这些载荷；它不等于删除传感器。

公开话题：

- `/usv/sensors/cameras/camera/image_raw`、`/usv/sensors/cameras/camera_right/image_raw`、`/usv/sensors/cameras/camera_side/image_raw`；对应 `camera_info` 在同一相机目录。
- `/usv/sensors/lidars/lidar/points` 与 `/usv/sensors/lidars/lidar/scan`。

使用 sensor-data / best-effort QoS 订阅。侧相机是独立单目，不能将其投影矩阵当成左右双目的一部分。桥接/控制入口默认使用项目的 Fast DDS 大消息缓冲配置，普通同机订阅无需额外设置；自定义 `FASTRTPS_DEFAULT_PROFILES_FILE` 会覆盖此默认值。改完这些源码后先运行 `./usv build`。

帧率、ROS 链路分段、内存/显存和实时率测量见 [完整场景渲染验收](../validation/sydney_rendering_2026-11-23/README.md)。同一张 GPU 上不要同时启动多个性能试验来代表单船性能；多机网络和其他 RMW 需要另验。
