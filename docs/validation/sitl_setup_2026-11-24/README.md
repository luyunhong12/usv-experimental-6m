# QGC 安置、SITL 容器与最小控制链接入

2026-11-24。本轮完成 QGC 安置、固定 ArduRover 源码与子模块、SITL 镜像构建、容器启动/退出及最小手动控制链验收。**这不是低速闭环、完整感知导航或 Nav2 验收。** 船体基线 tag `sim-baseline-2026-11-23` 不变。

## 安装与源码

- QGC 用户提供的 x86_64 AppImage 与官方 v5.1.5 资产 SHA-256 相同：`9a47e4cf269d9e4f897f582f6b19aa7ebf1e9e67c9d68a532a45b541354bc344`。移动到 `/home/yh/Applications/QGroundControl/QGroundControl-v5.1.5-x86_64.AppImage`，添加应用菜单入口；`--version` 返回 v5.1.5。[安装记录](qgc-install.json)。未做 QGC GUI 连接/操作验收。
- ArduPilot 位于 `dependencies/src/ardupilot`，固定 `Rover-4.7.3` / `d9c5097c3d8333435712ffed86bca8ce00c4bb46`，递归子模块全部初始化。[子模块清单](submodules.txt)。源码与 AppImage 均不嵌入主仓库。
- [飞控 Dockerfile](../../../deploy/sitl/Dockerfile) 使用固定 Ubuntu 镜像摘要与非 root 编译，生成不含编译工具链的运行镜像；实际镜像 [ID](sitl-image.txt)、[运行库/二进制摘要](sitl-runtime-packages.txt)。APT 包清单是本次实测，不承诺未来构建逐字节相同。
- 本机启动入口增加容器模式，默认源码位置改为项目 dependencies，状态统一放 artifacts。参数只读、当前用户 UID/GID、capabilities 清空；正常中断后容器自动删除，数据保留。[实际挂载与退出结果](launcher-smoke.json)、[非法参数拒绝](launcher-invalid-inputs.json)。

## 为什么本轮提前用了隔离仿真环境

本机 Gazebo 依赖已变化：物理库7.9.0不符合工程要求的7.8.0，渲染库8.3.0也不再是旧缓存的8.2.3。宿主机 full 构建失败，详见 [原失败报告](supervisor/host-failure.txt)；没有修改系统包、放宽 EXACT 或创建假版本链接。

使用此前独立复现容器 `usv-baseline-20261123` 保存本地恢复镜像 `usv-sim:baseline-20261123`，另开 `usv-sim-sitl-check`，不改写原基线容器。船体端源码基于1bf459e，覆盖本轮两个自有入口/测试文件。与当前受检129个源码文件逐一 SHA-256 一致：[最终对照](final-source-comparison.json)。21个配置文件仅PBR绝对路径不同，归一项目路径后相同：[参数对照](container-parameter-comparison.json)。这不是重新从零搭建的 GPU 发行镜像。

两端使用 Linux 主机网络，桥仅监听127.0.0.1。正式跨容器控制权锁共享尚未实现；本轮所有直接控制船体的进程在同一仿真容器，同一 UID。[第二控制端被拒绝](control-ownership.txt)。SITL 仅通过 JSON 输出 PWM，不直接施加另一套物理力。

## 测试结果与中间发现

| 试验 | 结果与适用范围 |
|---|---|
| 完整悉尼、无渲染传感器手动试验 | 前进、倒车、左右转及回中停推通过；当时尚未增加估计就绪门槛。[结果](manual_container_01/result.json) |
| 完整悉尼追加失联工况 | 手动/停推检查通过，但人工分析发现后段 EKF variance，不能据此认定估计健康。[结果](manual_dropout_02/result.json) |
| 隔离水域对照 | 相同控制过程移除其他地图模型，后段 EKF variance 未复现。[结果](isolated_dropout_03/result.json) |
| 最终就绪门槛版本、隔离水域 | 14/14通过；正常解锁前估计连续就绪2秒，解锁后无 EKF variance 告警。[最终结果](ready_dropout_04/result.json) |

完整场景第二轮在约104秒时位置停滞、速度突变、IMU冲击，随后 EKF 告警；移除地图其他模型后未复现，支持场景接触影响的解释。没有逐接触点取证，不声称已经定位到具体码头碰撞体。[估计器日志事件](estimator-events.json)、[IMU/速度导数诊断](imu-kinematic-diagnostic.json)。该导数诊断是采样近似，不是 IMU 标定结果。

原手动测试在 MANUAL 刚允许解锁时即推进，GPS融合尚未完成，故增加导航估计就绪条件，未关闭任何解锁检查。原右转判据依赖绝对航向，本轮改为阶段航向增量，补左转/回中，以及未完成阶段默认失败，避免误报通过。

最终试验在仿真45.222秒解锁；手动前进窗口平均3.546 m/s，倒车−2.835 m/s。这是60%左右手动推进指令响应，**不是低速速度闭环或最大航速试验**。JSON记录最大滞后0秒只指本次记录样本，不能推广为严格锁步。[汇总指标](metrics.json)。

失联试验先确认正在推进，再同时停止 GCS 心跳与 RC override；约3.834仿真秒后推进请求持续归零，状态报告 Radio Failsafe。证明该组合失联的停推，**不等于隔离验证了纯GCS失联、JSON失联或所有故障**；停推也不等于船立即静止。SITL 自身与船体惯性保持原计算。

## 检查与证据

Code Supervisor 最终 full 退出0；宿主机静态扫描，构建命令明确改为固定依赖容器内 `./usv build && ./usv test`。12项C++、55项Python、5项设计接口测试通过。保留 [完整报告](supervisor/report.txt)、[实际覆盖配置](supervisor/container-full.toml)、[24项人工审查](code-review.md)。Dockerfile和部署语义另行人工检查，没有声称工具自动验证全部容器安全性或上游飞控代码。

四轮试验各有独立快照、参数、MAVLink、JSON与推进遥测；公开压缩选择文件，完整归档位于本地 `artifacts/sitl/setup_2026-11-24`，DataFlash位于 `artifacts/sitl/platform15/instance_42/logs`。首次 `docker cp` 导出遇到模型材料相对符号链接拒绝，原数据未丢失，后续以 tar 归档并选择普通文件公开。公开快照带容器绝对路径，仅作证据，不是可直接在任意目录运行的发行模型。

源码变更以 [文件摘要](source_manifest.json) 绑定，公开证据以 [文件清单](manifest.json) 校验。容器镜像留本机，不推送镜像仓库；Git只发布自有配方、接口改动、文档和选择证据。

## 下一步与未完成项

1. 在固定环境继续验证低速航向/速度闭环，并独立测试 GCS、JSON、传感器丢失和仿真暂停/重置。
2. QGC GUI连接、参数读写、模式切换、任务上传下载尚未验收。
3. 正式仿真运行镜像、跨容器锁共享、Compose与GPU完整场景联调仍待完成；已确认Docker能识别RTX3060，不等于GPU渲染验收通过。
4. 宿主机依赖漂移仍未修复，普通本机启动不能沿用历史保证；本轮没有更改系统依赖。
5. 完整悉尼航线需规避码头，不能把地图接触造成的估计告警当成无条件通过。
6. Nav2未接入；船体未实船标定、自然配平偏航和重载航速等原有边界仍保留。

使用入口见 [SITL 部署说明](../../../deploy/sitl/README.md)。官方依据：[JSON协议](https://github.com/ArduPilot/ardupilot/blob/Rover-4.7.3/libraries/SITL/examples/JSON/readme.md)、[Rover4.7.3](https://github.com/ArduPilot/ardupilot/releases/tag/Rover-4.7.3)、[QGC5.1.5](https://github.com/mavlink/qgroundcontrol/releases/tag/v5.1.5)。
