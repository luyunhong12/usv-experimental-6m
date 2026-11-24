# 独立容器安装与复现

新增GPU桌面和飞控双容器入口见[桌面SITL指南](desktop_sitl.md)，本页保留原独立无窗口复现过程。

这是 Ubuntu 24.04 amd64、ROS 2 Jazzy / Gazebo Harmonic 的无窗口复现路径。容器从 GitHub 获取源码、Git LFS 模型、固定 VRX 和官方 Fuel 资源，不挂载主机源码、ROS 安装或构建产物。GPU 图形窗口、其他架构和任意未来软件源版本不在本次验证范围。

## 构建依赖镜像

在已经克隆本仓库的主机上运行，仅把小型 setup 目录作为构建上下文：

```bash
docker build -t usv-repro:jazzy tools/setup
# 如果 Docker 配置了不可用的代理，可显式清空构建代理后重试：
docker build --build-arg HTTP_PROXY= --build-arg HTTPS_PROXY= \
  --build-arg http_proxy= --build-arg https_proxy= \
  --build-arg ALL_PROXY= --build-arg all_proxy= \
  -t usv-repro:jazzy tools/setup

docker run --init -it --shm-size=256m --name usv-clean usv-repro:jazzy bash
```

本轮容器 `/dev/shm` 上限为256 MiB，给多参与者图像传输段留出预算；该设置不改变传感器频率或QoS。

下面命令均在容器内执行。宿主机代理若需使用，应自行配置容器可达的代理地址；不在镜像中保存认证凭据。

## 固定源码和资源

本轮测试使用直接联网。若容器继承了不可达的宿主代理，可先在当前容器终端执行下面命令；需要有效代理的网络应保留正确配置，不要照搬清空：

```bash
unset HTTP_PROXY HTTPS_PROXY ALL_PROXY http_proxy https_proxy all_proxy
```

```bash
GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/luyunhong12/usv-experimental-6m.git /work/usv
cd /work/usv
git checkout 1bf459e0918b94a9318e7dfb040c977856da35cf
git lfs pull
git lfs fsck
mkdir -p dependencies/src
git clone https://github.com/osrf/vrx.git dependencies/src/vrx
git -C dependencies/src/vrx checkout 7609d1bd90ce7edb29d040a082f949e8b089c864

source /opt/ros/jazzy/setup.bash
python3 tools/setup/download_sydney_assets.py
for spec in mb_marker_buoy_red/5 mb_marker_buoy_black/7 \
  mb_marker_buoy_green/4 mb_marker_buoy_white/4 \
  mb_round_buoy_orange/5 mb_round_buoy_black/4 \
  post/4 antenna/5 ground_station/7; do
  gz fuel download -u "https://fuel.gazebosim.org/1.0/openrobotics/models/$spec"
done

./usv build
./usv test
source ros_ws/install/setup.bash
```

Sydney 下载器核对官方 v3 ZIP 的 SHA-256 和 ZIP CRC。其他 Fuel 模型按上述版本下载，保留各模型许可；镜像配方不内嵌这些素材。当前稳定基线运行源码的安装过程和验收状态见 [稳定基线复现报告](../validation/stable_baseline_2026-11-23/README.md)；旧版本证据保留在 [历史复现报告](../validation/platform_readiness_2026-11-21/step4/README.md)。

## 启动与验收

手动运行：

```bash
./usv run --wave-gain 0 headless:=True
```

另开容器终端进入 `/work/usv`，等平台 ready 后执行 `./usv drive --throttle 0.3 --duration 20`。更完整的操作见 [ROS 操作指南](README.md)。不要同时启动手动平台和自动验收程序。

自动验收会自己启动和关闭平台。在停止手动平台后运行：

```bash
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
# 本轮无硬件GPU时限制 llvmpipe 线程，避免软件渲染占满整机。
export LP_NUM_THREADS=4
python3 tests/integration/validate_public_cli.py \
  --drive-duration 20 --ready-timeout 180 --output artifacts/clean_cli
python3 tests/integration/validate_platform_launcher.py \
  --kill-controller --startup-timeout 180 --output artifacts/clean_exit
```

无硬件 GPU 的软件渲染可能明显慢于实时；这些参数增加的是墙钟观察时间，没有降低实际舵角、油门和仿真时间门槛。`headless` 不等于关闭相机/激光渲染。冷启动、软件渲染和多进程清理的实测范围见报告；不能把这套无窗口验收当作 GUI 显示或实时性能认证。

## 为什么固定 gz-physics

本地惯性后端依赖 gz-sim **8.15.0**、gz-physics **7.8.0**。2026-11-23 的干净安装从实时 ROS 软件源得到 gz-physics 7.9.0，CMake 因 EXACT 约束拒绝构建。配方保留约束，从 [ROS Jazzy 2026-09-11 官方快照](http://snapshots.ros.org/jazzy/2026-09-11/) 安装 `ros-jazzy-gz-physics-vendor=0.0.9-1noble.20260905.065116` 并 hold。

快照使用独立签名密钥。配方从 Ubuntu HTTPS 密钥服务器取得密钥，核对完整指纹 `4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA`，再由 APT 验证仓库签名；没有使用 `trusted=yes`、`--allow-unauthenticated` 或关闭证书验证。密钥区别与快照安装背景可查 [Open Robotics 说明](https://discourse.openrobotics.org/t/security-issue-on-ros-build-farm/9342) 和 [OSRF 镜像问题记录](https://github.com/osrf/docker_images/issues/897)。

其余 Ubuntu/ROS 包仍来自实时源，完整实测版本清单保存在复现报告。本配方不是永久冻结整个操作系统；软件源变化时应重新验收，不能简单删除 EXACT 约束来绕过 ABI 不兼容。
