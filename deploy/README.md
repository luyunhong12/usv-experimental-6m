# 部署配置

本目录管理运行环境与进程编排；船体参数、飞控参数和桥接业务代码仍由各 ROS 包管理，不复制到镜像配方中。

## 当前运行入口

使用 [桌面容器操作指南](../docs/getting_started/desktop_sitl.md) 启动两服务，验收状态见 [本轮报告](../docs/validation/desktop_sitl_2026-11-24/README.md)。

- 原依赖配方 `tools/setup/Dockerfile` 提供 ROS/Gazebo 编译环境。
- `simulation/Dockerfile` 在该依赖环境上打包本项目源码和资源，干净构建运行镜像。
- `sitl/Dockerfile` 构建固定 ArduRover 飞控运行镜像。
- `compose.yaml` 运行仿真和飞控；`./usv container up/down/status/logs` 管理本机实验。

QGC安装在宿主机。两层仿真配方分别负责依赖和项目运行包，不复制船体物理实现。宿主机旧构建依赖已漂移，优先使用记录中的容器环境。

## 网络和数据约定

现有 JSON 桥仅绑定 localhost，本部署明确使用host网络，两个容器的localhost因而可互通。桥接器保留在仿真侧；SITL 容器不安装第二套船体物理。执行器始终只有一个控制者。

运行日志、参数运行副本和容器数据放在项目 artifacts 下按试验批次分目录。版本化参数源保留在 ROS 包内；第三方源码留在 dependencies/src。镜像按实际源码提交固定版本，安装与 GPU 测试通过后再记录镜像摘要，不使用可漂移的 latest 作为验收标识。
