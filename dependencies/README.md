# 依赖与版本

VRX 本地源码保留在 `src/vrx`，版本固定于 `sources.repos`。新环境可使用 vcstool：

```bash
mkdir -p dependencies/src
vcs import dependencies/src < dependencies/sources.repos
```

本仓库包含物理适配需要的最小上游源码片段及原 LICENSE / SOURCE.json，位置为 `vendor/gz-sim-physics` 和 `vendor/gz-physics7-dartsim`。不修改系统 Gazebo。

Python 工具依赖见 `tools/setup/requirements.txt`；Gazebo Python bindings 和 ROS 来自系统安装。`vendor/python` 为已有本地兼容依赖，默认不提交，版本应由安装环境管理。

悉尼资产保留在 `vendor/fuel`。缺失时先配置 Gazebo Fuel 缓存，或检查 `tools/setup/download_sydney_assets.py` 的下载来源和校验；资产不因复制到本地而自动获得开源许可。

完整新机器安装尚未用容器重新验证。本轮验证的是现有安装环境下的新目录和干净 colcon 重建。不要把本地通过等同于任意机器开箱即用。

可选 Blender 连接源码本地保留但不嵌入主仓库。需要时执行 `vcs import dependencies/vendor < dependencies/blender.repos`。
