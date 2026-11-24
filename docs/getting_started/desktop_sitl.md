# Docker 悉尼场景、ArduRover SITL 与 QGroundControl

本页是 Linux 本机桌面操作入口：Gazebo/ROS/JSON桥在仿真容器，ArduRover在飞控容器，QGC在宿主机。Nav2不在本次部署范围。验收范围和剩余问题见[本轮报告](../validation/desktop_sitl_2026-11-24/README.md)，船体物理基线见[基线报告](../validation/stable_baseline_2026-11-23/README.md)。

## 环境和目录

当前验证环境为 Ubuntu 24.04 amd64、X11、NVIDIA RTX 3060 Laptop、ROS 2 Jazzy、Gazebo Sim 8.15.0/Physics 7.8.0、ArduRover 4.7.3、QGC 5.1.5。当前镜像内文件属主为UID/GID 1000；其他UID、Wayland、AMD/Intel GPU、Windows/macOS不在本轮验证范围。

- `deploy/simulation/Dockerfile`：在原依赖配方上复制必要源码、模型并干净构建；不打包宿主机ROS安装、旧build/install、ArduPilot源码或QGC。
- `deploy/sitl/Dockerfile`：固定ArduPilot源码构建飞控运行镜像。
- `deploy/compose.yaml`：两服务编排；相机、点云仍由仿真端生成。
- `tools/setup/run_containers.sh`：由 `./usv container` 调用，管理启动、停止、状态和日志。
- `artifacts/container-runtime`：会话X11授权、ROS日志和SITL状态；禁止加入Git或公开其中凭据。
- `artifacts/control-locks`：本机与容器同UID、同Gazebo分区、同船名的协作控制锁。
- `dependencies/vendor/fuel`：第三方原资源；递归离线副本生成在 `ros_ws/src/usv_description/models/instances/sydney_assets`，不修改上游原文件。

## 首次构建

已有本轮镜像时可直接跳到启动。重新安装需要先按[原容器指南](container.md)下载ROS/VRX和Fuel资源，再构建运行镜像：

```bash
cd /home/yh/Project/usv-experimental-6m
docker build -t usv-repro:sensor-baseline tools/setup
docker build -f deploy/simulation/Dockerfile -t usv-simulation:sitl-gui .
# 飞控源代码与固定版本要求见 deploy/sitl/README.md
docker build -f deploy/sitl/Dockerfile -t usv-sitl:rover-4.7.3 .
```

基础镜像仍依赖软件源，最终验收记录具体镜像摘要；本地标签不是永久不变的发行版本。构建严格检查物理库版本，不能通过删除EXACT约束绕过宿主机升级问题。

悉尼场景除了顶层模型，还需要 `tent/4`、`foldable_table/4`、`foldable_chair/3`。先使用官方Fuel下载器下载这些固定版本，再将缓存中的模型目录放入项目 `dependencies/vendor/fuel/<模型名>`。可在已加载Gazebo环境的下载终端执行：

```bash
for spec in tent/4 foldable_table/4 foldable_chair/3; do
  gz fuel download -u "https://fuel.gazebosim.org/1.0/openrobotics/models/$spec"
  model_name="${spec%/*}"
  model_version="${spec##*/}"
  if [[ ! -e "dependencies/vendor/fuel/$model_name" ]]; then
    cp -a "$HOME/.gz/fuel/fuel.gazebosim.org/openrobotics/models/$model_name/$model_version" \
      "dependencies/vendor/fuel/$model_name"
  fi
done
```

上述复制用于首次缺失目录，已有资源应先核对版本和摘要，不能把旧目录再嵌套复制一层。其他模型的固定版本沿用原容器指南。缺少递归资源时准备场景会明确报错，不再依赖运行期间临时联网补模型。各模型版本和文件摘要以本轮资源清单为准。

## 开始实验

在登录桌面的终端中执行：

```bash
cd /home/yh/Project/usv-experimental-6m
./usv container up
./usv container status
./usv container logs
```

`up`返回只表示容器已启动，不表示导航估计已就绪。等待Gazebo完整显示，QGC连上飞控，消息出现GPS融合正常后再解锁。不得关闭解锁检查或强制解锁来掩盖故障。

启动安装好的QGC：

```bash
/home/yh/Applications/QGroundControl/QGroundControl-v5.1.5-x86_64.AppImage
```

若当前执行环境确实不能使用FUSE，可使用AppImage自带解包运行模式：

```bash
APPIMAGE_EXTRACT_AND_RUN=1 /home/yh/Applications/QGroundControl/QGroundControl-v5.1.5-x86_64.AppImage
```

不要同时开两个接收同一UDP端口的QGC实例。SITL通过UDP `127.0.0.1:14550`主动向地面站发送遥测；QGC默认UDP自动连接即可。TCP5860留作诊断，不要让两个客户端争用同一TCP连接。两容器使用host网络，并非隔离网络；仅用于受信任的本机实验环境。

## 用QGC开船

本轮验证的布局为：Settings → Fly View → Virtual Joystick，启用Virtual Joystick和Auto-Center Throttle，保持Left-Handed Mode关闭。

1. 模式选择 **Manual**。点击顶部状态，选择Arm并按界面要求长按确认。
2. **左摇杆上下**控制前进/倒车，中心为零油门。先小幅推杆。
3. **右摇杆左右**控制桨舵偏转。右摇杆上下不控制这艘Rover船的油门。
4. 松手回中会归零推进请求，但船会按惯性继续滑行；这不是瞬间刹停。
5. 实验结束点击状态面板Disarm并确认。

QGC沿用通用飞行界面，部分文字显示Flying/Land或飞机图标，不代表仿真船采用飞行器物理。MAVLink心跳实际类型为SURFACE_BOAT（11），固件为ArduRover。

虚拟摇杆适合基本检查；鼠标单点不方便同时控制双摇杆。需要持续协调油门和转向时使用兼容实体手柄并单独校准，不把本轮单鼠标试验当作手柄验收。

不要同时运行 `./usv drive`、其他直接推进发布者或第二个JSON桥。共享锁只能约束本项目遵守该协议、同UID的入口，不能阻止任意外部程序直接发布指令。直接调用底层脚本时，也必须显式设置相同的 `USV_CONTROL_LOCK_DIR` 和 `GZ_PARTITION`，不能假定未配置的临时目录锁能跨容器互斥。

## 停止与重启

```bash
./usv container down
./usv container up
```

`down`只删除本项目两个容器，保留运行数据和SITL EEPROM；不会停止其他项目容器。由于EEPROM会保留参数，修改参数默认文件不等于覆盖所有历史参数。需要新工况时，先停止服务并归档SITL数据，再建立明确命名的新实验目录，不能删除旧实验来“恢复通过”。

## 常见问题

- **Docker能运行nvidia-smi但仿真很慢**：检查Gazebo `~/.gz/rendering/ogre2.log`，应显示NVIDIA RTX3060而不是llvmpipe。Compose显式挂载NVIDIA EGL清单并选用该后端；仅`--gpus all`不足以证明传感器已使用GPU。
- **QGC灰色底图**：属于在线地图瓦片网络问题。本轮Google局部底图未成功加载，仍能显示船位、轨迹并操控；它与本地Gazebo悉尼三维场景不是同一资源。不要据此认定GPS或VRX水面丢失。
- **Gazebo无法找到帐篷/桌椅/浮标网格**：核对递归Fuel资源，而不只是顶层Sydney模型。
- **本机 `./usv run` 构建失败**：宿主机依赖漂移尚未回退；本页运行环境在容器内，不能混用旧本机build/install。

原始接口依据见[ArduPilot JSON协议](https://github.com/ArduPilot/ardupilot/blob/Rover-4.7.3/libraries/SITL/examples/JSON/readme.md)、[Rover手动控制映射](https://github.com/ArduPilot/ardupilot/blob/Rover-4.7.3/Rover/GCS_MAVLink_Rover.cpp)、[QGC虚拟摇杆说明](https://docs.qgroundcontrol.com/Stable_V5.1/en/qgc-user-guide/settings_view/virtual_joystick.html)。最终以本轮实际界面和遥测证据为准。
