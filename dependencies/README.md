# 依赖与版本

VRX 本地源码保留在 `src/vrx`，版本固定于 `sources.repos`。新环境可使用 vcstool：

```bash
mkdir -p dependencies/src
vcs import dependencies/src < dependencies/sources.repos
```

本仓库包含物理适配需要的最小上游源码片段及原 LICENSE / SOURCE.json，位置为 `vendor/gz-sim-physics` 和 `vendor/gz-physics7-dartsim`。不修改系统 Gazebo。

Python 工具依赖见 `tools/setup/requirements.txt`；Gazebo Python bindings 和 ROS 来自系统安装。`vendor/python` 为已有本地兼容依赖，默认不提交，版本应由安装环境管理。

悉尼资产保留在 `vendor/fuel`。缺失时先配置 Gazebo Fuel 缓存，或检查 `tools/setup/download_sydney_assets.py` 的下载来源和校验；资产不因复制到本地而自动获得开源许可。

已完成独立 Ubuntu 24.04 amd64 / ROS Jazzy 容器的 GitHub 检出、Git LFS、官方资源下载、全新构建和公开入口验收，见 [安装指南](../docs/getting_started/container.md) 与 [证据](../docs/validation/platform_readiness_2026-11-21/step4/README.md)。实时 ROS 源已提供 gz-physics 7.9，工程仍要求 7.8；配方通过官方签名快照固定依赖，不放宽 ABI 约束。其他架构、GUI/GPU 和未来软件源版本仍需验证。

可选 Blender 连接源码本地保留但不嵌入主仓库。需要时执行 `vcs import dependencies/vendor < dependencies/blender.repos`。

## 飞控依赖位置

ArduPilot（含 SITL）源码统一放在 `src/ardupilot/`，保留独立 Git 历史，不嵌入本仓库。当前仅预留目录，尚未下载；Git 不保存空目录。后续新增独立的 `ardupilot.repos` 并固定完整提交及子模块，原 `sources.repos` 继续服务现有仿真安装。源码下载、构建和联调状态见 [SITL 部署说明](../deploy/sitl/README.md)。QGC 由用户在宿主机安装，不作为 ROS 源码依赖。
