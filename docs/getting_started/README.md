# 安装与运行

本次验证环境：Ubuntu / ROS 2 Jazzy、Gazebo Harmonic。当前物理后端明确依赖 gz-sim 8.15.0、gz-physics 7.8.0 与系统 DART。不能直接假设任意版本兼容。

1. 安装 ROS 2 Jazzy、对应 Gazebo/ROS bridge、colcon、C++ 构建依赖。
2. 按 `dependencies/README.md` 获取固定版本 VRX、场景素材及 Python 依赖。现有本地副本已保留。
3. 项目根执行 `./usv build`，成功后 `./usv run`。不需要 ArduPilot。
4. 等待 `Boat ready`；另一终端用 `./usv drive --throttle 0.3 --duration 10` 试航，退出或指令超时会停推，船仍可能惯性滑行。
5. `./usv energy --duration 30` 查看能源，`./usv test` 跑单元测试。

这些入口从任意当前目录调用都能定位仓库，例如 `/path/to/usv-experimental-6m/usv run`。手动 ROS 调试需先 source `/opt/ros/jazzy/setup.bash` 和 `ros_ws/install/setup.bash`。默认 Gazebo 分区为 `usv_platform15`；自定义时启动和控制两端使用相同 `GZ_PARTITION`。

集成测试需要本地 Gazebo Transport / ROS DDS 网络与共享内存权限：

```bash
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
mkdir -p artifacts/my_smoke
python3 tests/integration/validate_platform_launcher.py --output artifacts/my_smoke --kill-controller
```

高级启动入口位于 `ros_ws/src/usv_bringup/scripts/`。`./usv run` 默认图形窗口；`./usv run headless:=True` 用于无窗口物理测试。不要在同一分区同时启动两套船。
