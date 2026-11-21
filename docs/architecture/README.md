# 架构与职责

- `usv_description`：船体与设备几何、碰撞、质量和推进参数。
- `usv_simulation`：水动力、推进、环境作用和 Gazebo 场景。
- `usv_physics`：固定版本 DART 质量/重心/惯性运行时适配。
- `usv_energy`：油电消耗、充电、保护及仪表。
- `usv_control`：当前 ROS 船舶控制、载荷接口、健康诊断。
- `usv_bringup`：整船组合配置与统一启动；开发脚本需完整源码树。
- `usv_ardupilot`：保留历史适配，不代表本轮重新验收。

设计源文件与仿真运行网格分开；公共船体参数保留在 description，两个设计方案不复制一套物理实现。工具通过 `tools/lib/project_paths.py` 定位仓库，不依赖启动时工作目录。`tool_paths.json` 记录工具分组位置；新增脚本时同步维护。

第三方 VRX 在 `dependencies/src/vrx`；colcon 显式扫描自有包和该依赖，不在源码目录维护第二份副本。编译输出、原始证据不属于源码。
