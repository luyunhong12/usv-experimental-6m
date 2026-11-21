# 代码与资产来源

本项目尚未发布，不将所有文件统一重新许可。

- 自有 ROS 包元数据已有 Apache-2.0 声明；根 LICENSE 提供该协议文本，仅适用于有权按此协议发布的自有代码。
- VRX：`dependencies/src/vrx`，来源/提交见 `dependencies/sources.repos`，遵守其原 LICENSE 与资源许可。
- Gazebo 物理源码片段：`dependencies/vendor/gz-sim-physics`、`gz-physics7-dartsim`，保留各自 LICENSE、SOURCE.json 及补丁说明。
- 悉尼场景资产：`dependencies/vendor/fuel`，保留来源信息，逐项检查模型 metadata 及许可后再分发；缓存默认不进入主仓库。
- Blender 连接工具：源码在 `dependencies/vendor/blender-mcp`，保留原许可，属于可选开发依赖。
- 船舶设计、设备商标/外观、图片和外部参考资料：来源资料随 design 保存，公开再分发范围尚待确认，不能由代码包的许可自动覆盖。

环境中现有 ArduPilot 二进制不属于本仓库；历史适配需显式指定可用二进制。Nav2 本轮未引入。
