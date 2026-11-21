# usv_control

提供命名空间化的 ROS 载荷接口、断电门控、静态 TF 和 IMU/GPS 反馈速度/航向控制。`scripts/run_interfaces.sh` 启动；默认接管执行器，其他控制源运行时改为 `control:=False`。支持 `name:=escort clock:=False` 第二船接口。控制指令失联 1 秒停止推进，保留真实惯性滑行。

`usv_control/ownership.py` 为同用户、Gazebo 分区、船名提供排他进程锁，供 ROS、手动脚本及 ArduPilot 共用。它不阻止外部直接发布原始 Gazebo 指令。

话题、测试、功率门控边界见工作区 docs/过程记录/2026-11-19_六项整改与WAMV接口及15公里时速方案.md。传感器安装位置应与 prepare_hull.py 同步；改变位置必须同步静态 TF 并重验。


## 2026-11-21 就绪与固定油门试验接口

`/usv/control/health` 为 String JSON，待机也发布；包含 `ready`、逐项 `checks`、`missing`、`gz_partition` 和 `command_mode`。看门狗/调度使用独立 steady clock，控制更新仍使用原生物理时间。ROS `/clock` 暂时缺失不会使健康检查停止。启动入口会等待真实就绪；任一必要接口进程退出会关闭接口栈。

`/usv/actuators/heading_hold_throttle` 为 Float64，取值 −1..1。它固定桨轴转速请求，仅调舵保持航向，不闭环控制航速；须持续发送指令。不要同时发送直接舵角或 cmd_vel。`/usv/control/state` 的旧航速字段仍只用于速度控制模式，固定油门模式见 health 中的 fixed_throttle_request/heading_reference_rad 及原生推进/舵机反馈。

高速零舵开环仍表现出方向不稳定，不能将固定油门航向保持的结果当作被动直航稳定性验证。
