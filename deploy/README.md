# 部署配置

本目录管理运行环境与进程编排；船体参数、飞控参数和桥接业务代码仍由各 ROS 包管理，不复制到镜像配方中。

## 当前状态与实施顺序

1. 优先复用已验收的 ROS/Gazebo/VRX，通过独立 SITL 容器接入；QGC 在宿主机安装。2026-11-24 检测到宿主机依赖升级，当前联调改用原固定版本容器，不能沿用旧本机通过结论。
2. 先验证时间、坐标系、PWM、唯一控制权、失联停推和低速闭环。
3. 恢复完整相机/点云并验收，再迁移到 GPU 仿真容器。
4. 两端验收后才增加 Compose 编排。当前没有可运行的 Compose 文件。

[SITL 配置位置](sitl/README.md) 已包含固定源码的 Dockerfile 与现有启动入口的容器模式。原有仿真配方继续位于 [tools/setup/Dockerfile](../tools/setup/Dockerfile)，其使用说明见 [容器指南](../docs/getting_started/container.md)。不维护第二份同功能仿真 Dockerfile。

## 网络和数据约定

现有 JSON 桥仅绑定 localhost，分容器部署前必须明确网络模式，不能默认不同容器的 localhost 互通。桥接器保留在仿真侧；SITL 容器不安装第二套船体物理。执行器始终只有一个控制者。

运行日志、参数运行副本和容器数据放在项目 artifacts 下按试验批次分目录。版本化参数源保留在 ROS 包内；第三方源码留在 dependencies/src。镜像按实际源码提交固定版本，安装与 GPU 测试通过后再记录镜像摘要，不使用可漂移的 latest 作为验收标识。
