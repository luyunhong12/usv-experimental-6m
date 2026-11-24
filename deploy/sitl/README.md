# ArduPilot SITL 容器配置位置

状态：目录准备完成，尚未下载 ArduPilot、编写镜像配方或完成联调。

- 上游源码：项目根目录下 `dependencies/src/ardupilot/`，独立 Git 仓库；当前仅预留本地目录。SITL 从该仓库编译，不另建 SITL 源码副本。
- 候选版本：`Rover-4.7.3`。下载时核实完整提交 SHA 及子模块，再增加独立的 `dependencies/ardupilot.repos`；保持已有 `sources.repos` 只获取仿真所需依赖，避免普通操船用户被迫下载飞控源码。
- 本目录后续保存实际需要的 Dockerfile、容器启动配置；仅在实现时添加，不先创建无法运行的占位脚本。
- 自有适配器和参数：见 [usv_ardupilot](../../ros_ws/src/usv_ardupilot/README.md)。已有参数属于历史配置，须按当前船体重新验收。
- QGC：用户安装宿主机应用，不放入此镜像；候选版本 `v5.1.5`，待联合验证。
- 日志与参数运行副本：`artifacts/sitl/<试验批次>/`，作为持久化挂载；不得覆盖版本化参数源。

版本候选来自 2026-11-24 官方发布核查，不代表项目兼容验收已经通过：[Rover 发布](https://github.com/ArduPilot/ardupilot/releases/tag/Rover-4.7.3)、[QGC 发布](https://github.com/mavlink/qgroundcontrol/releases/tag/v5.1.5)。

测试复用 `tests/integration/`、`tests/scenarios/` 和 `tests/regression/`；验收证据写入 `docs/validation/`，不在部署目录再建立一套测试框架。
