# ArduPilot SITL 容器

固定源码 `Rover-4.7.3` / `d9c5097c3d8333435712ffed86bca8ce00c4bb46`。本目录管理飞控容器，不安装船体物理、ROS、Gazebo 或 QGC。具体已完成测试见后续验收记录，构建配方存在不等于整船联调通过。

## 获取与构建

以下命令从项目根执行：

```bash
vcs import dependencies/src < dependencies/ardupilot.repos
git -C dependencies/src/ardupilot submodule update --init --recursive
docker build -f deploy/sitl/Dockerfile -t usv-sitl:rover-4.7.3 .
```

只复制已固定的上游源码；Dockerfile 专用忽略文件排除项目数据。构建校验提交、子模块及已跟踪源码未修改。编译为非 root 用户，运行镜像不含编译工具链。基础镜像固定摘要，Python 直接依赖固定版本；APT 软件包仍取当前源，验收需保存实际包清单与镜像 ID。修改版飞控应另建有明确版本的配方，不绕过校验。

若 Docker 注入了不可达代理，应修正容器代理；本机直接联网构建时可显式传入空的 HTTP_PROXY、HTTPS_PROXY、ALL_PROXY 及对应小写 build-arg。不要关闭 TLS 校验。

## 启动

先按 [集成规划](../../docs/integration/README.md) 准备仿真与桥接器；不能与现有 ROS 操船控制器同时驱动执行器。

```bash
python3 ros_ws/src/usv_bringup/scripts/run_sitl.py \
  --container-image usv-sitl:rover-4.7.3 --instance 10 --profile platform15
```

这是现有启动入口新增的容器模式，不另建一套启动脚本。`ARDUPILOT_CONTAINER_IMAGE` 可提供默认镜像；不设置时仍支持 `--binary` / `ARDUPILOT_BIN` 的本机模式。本机默认二进制改为 `dependencies/src/ardupilot/build/sitl/bin/ardurover`。

| 参数/资源 | 约定 |
|---|---|
| instance | 0–99 的整数；不同实例必须不同，默认10 |
| JSON 端口 | UDP `9002 + 10 × instance`；默认9102，桥接器需一致 |
| MAVLink | TCP `5760 + 10 × instance`；默认5860 |
| profile | baseline 或 platform15；加载现有参数，不代表已经重新调参 |
| wipe | 显式重置该实例的 SITL 参数存储；日常重启不添加 |
| 容器名称 | `usv-sitl-<profile>-<instance>`；同名运行会拒绝，不自动删除其他进程 |
| 参数源 | `ros_ws/src/usv_ardupilot/config` 只读挂载到 `/config` |
| 持久状态 | `artifacts/sitl/<profile>/instance_<编号>` 挂载到 `/data` |
| 权限 | 当前用户 UID/GID；删除 capabilities，禁止新增权限，无 GPU/USB 挂载 |

Linux 主机网络用于兼容现有 localhost JSON 桥；不代表网络隔离。仅用于可信的本机开发环境，不配置路由器端口转发。容器通过 Docker init 转发信号、回收子进程，正常退出后删除容器，保留数据。端口冲突时换实例并同步桥接端口，不能随意终止其他项目进程。

## QGC

用户提供的 AppImage 已核对官方 `v5.1.5` 摘要，宿主机路径：

```text
/home/yh/Applications/QGroundControl/QGroundControl-v5.1.5-x86_64.AppImage
```

应用菜单入口为 QGroundControl 5.1.5；已验证 `--version`，界面操作与联合连接需要单独验收。QGC 中可创建 TCP 连接到 `127.0.0.1:5860`（实例10）。同一 MAVLink 端口不要同时接测试客户端与 QGC；测试期间先断开 QGC。AppImage 不进入项目 Git 仓库。

官方依据：[JSON 协议](https://github.com/ArduPilot/ardupilot/blob/Rover-4.7.3/libraries/SITL/examples/JSON/readme.md)、[Rover 发布](https://github.com/ArduPilot/ardupilot/releases/tag/Rover-4.7.3)、[QGC 发布](https://github.com/mavlink/qgroundcontrol/releases/tag/v5.1.5)。

## 当前验收与限制

[2026-11-24 准备与接入记录](../../docs/validation/sitl_setup_2026-11-24/README.md) 保存实测结果。宿主机 Gazebo 库已发生版本漂移，不能直接用当前本机重新构建结果启动；本轮在从上一轮已构建容器保存的本地镜像 `usv-sim:baseline-20261123` 中验证船体端。该恢复镜像不是新发布的可复现 GPU 运行镜像，不上传镜像、不改变宿主机系统包。

现有控制权租约依赖同一用户和共享文件系统；本轮所有船体控制端放在同一仿真容器中，不能据此声称不同容器/宿主机控制端已自动互斥。正式多容器运行入口需统一锁目录及 UID，或集中执行器仲裁，再验收。

最小接口测试复用 `tests/integration/validate_ardupilot.py`：`--external-sitl` 表示外部管理实例42（JSON9422、MAVLink6180）；`--no-rendering` 只在试验快照中移除相机与GPU雷达；`--isolated-water` 移除地图其他模型但不改变船体物理；`--gcs-dropout` 在推进中停止地面站心跳和遥控覆盖。每轮用不同 `--label`，输出位于 `artifacts/sitl/<label>`。默认等待姿态、水平速度、绝对位置估计连续就绪2秒后请求正常解锁。

## 实验性 LOITER 速度阻尼（尚未替换正式镜像）

`Dockerfile.loiter` 从干净的固定上游源码构建显式实验补丁；默认是 `dependencies/patches/ardupilot-rover-loiter-active-stop.patch`，可用构建参数LOITER_PATCH选择保留的失败速度阻尼实验，不修改本地上游checkout。需先具备原版运行镜像和对应本地 `usv-sitl-toolchain:rover-4.7.3` 工具链镜像；构建使用 `--network none`，工具链与运行基础镜像ID应另行存档。它是实验分支，不能标称未经修改的官方固件。

```bash
docker build --network none -f deploy/sitl/Dockerfile.loiter -t usv-sitl:loiter-active-stop-experimental .
```

新增 `LOIT_VEL_D` 无量纲参数，默认0，范围0–3。仅机动船、有效有限的AHRS水平速度下生效；实际目标位置不变，回归速度矢量减去 `LOIT_VEL_D × 水平地速`，再经过已有速度、航向、执行器限制。无有效速度或参数0时使用原逻辑。没有修改推力、舵速、惯性或水动力；该候选尚需多工况验证，不用于实体船。

测试入口支持 `USV_SITL_IMAGE` 和 `USV_NAV_CONFIG_DIR`，分别指定实验镜像与含三个完整parm文件的候选目录；仍复制输入快照并记录不可变镜像ID，默认继续使用原正式版本。调查、失败候选和采用状态见[跨流向保持调查](../../docs/validation/current_hold_2026-11-25/README.md)。

主动零地速实验新增LOIT_KEEP_SPEED（0/1，默认0），只改变机动船LOITER的零速度分支，保留原速度积分和反馈。原版半径对照及试验结果未完成前不得推广；HOLD和失联语义不改。速度阻尼与主动零地速是两个独立补丁，不叠加。
