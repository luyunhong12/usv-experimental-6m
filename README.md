# 6 米实验无人船 · VRX 工程仿真平台

一个以**可解释的船体物理、可追溯的工程参数和可重复测试**为目标的无人船仿真项目。基于 ROS 2 Jazzy、Gazebo Harmonic 和 VRX 悉尼赛艇水域，当前重点是中央单机、桨舵一体推进的 6 米船模，同时保留双固定推进器原始设计。

> **项目定位：工程仿真平台，不是经过海试标定的实船数字孪生。** 没有完整实船数据时，使用明确标注来源和假设的参数。能够运行、画面精细、控制器能保持航向，都不能代替物理验证。
>
> **当前主要缺陷：高速零舵时被动航向不稳定。** 基线诊断中，舵机已跟踪零角度，船仍持续转向；这一问题优先于 ArduPilot / Nav2 集成。修复前请勿把航向保持直航当作船体自身稳定。

## 为什么做这个项目

希望先把“船”本身做好，再接自主航行系统：能合理漂浮和随浪、推力与功率相互约束、载荷改变吃水和操纵表现、油电消耗影响可用动力、燃油减少影响质量和惯性。之后再讨论飞控和导航算法的表现，避免控制器掩盖仿真对象的问题。

工程原则：

1. **外观、碰撞、浮力几何各有职责。** 外观保留精细结构，碰撞描述刚体接触，浮力采用封闭船体与分布式压力；不以整船网格直接替代所有物理。
2. **参数有唯一来源。** 质量和功耗使用逐件台账，区分工程估算、厂家参考、几何计算和未标定系数；历史参数快照不能冒充当前配置。
3. **力、能量与质量相互一致。** 推进受到电功率和转矩限制，舵机消耗能源；燃油用量与刚体质量、重心和完整惯性张量联动。
4. **用试验揭示问题。** 记录原始遥测、工况、源码版本与失败案例；不通过放宽检查或修改原始数据得到“通过”。
5. **先物理，后控制与导航。** 保留手动/ROS 控制以便验证，不急着堆叠飞控和任务系统。

## 已有能力与验证边界

| 模块 | 当前内容 | 不能据此宣称 |
|---|---|---|
| 模型与装配 | 两种 Blender 设计、精细推进器、碰撞网格和装配检查 | 制造净空、强度及适航认证 |
| 水动力 | 静水恢复、分布式波浪压力、流/风、阻尼、附加质量近似 | CFD 或海试标定、高速滑行和砰击准确 |
| 推进与转向 | 正反推、桨轴动态、转矩/功率限制、舵角与速率限制 | 真实推进器全工况性能图 |
| 油电 | 电池、负载、发电/充电、低电/温度保护、燃油消耗 | 已验证的真实航程或完整电气热模型 |
| 动态质量 | 燃油对应的质量、重心和惯性更新及后端回读 | 油液动态晃荡已建模 |
| 传感器与接口 | IMU、GPS、相机、点云、TF、健康诊断 | 所有传感器已做实机噪声标定 |
| 运行可靠性 | 连续就绪检测、失联停推、必需进程退出清理 | 全海况、任意机器长期稳定 |
| 飞控与导航 | 历史 ArduPilot 适配代码保留；Nav2 尚未实现 | 本阶段已经完成自主航行系统集成 |

## 性能基线：15 km/h 是哪个方案

项目保留 **6 kW 基准方案**，另有 **40 kW、六电池的概念升级方案**。下面是后者在满油 60 L、静水、固定桨轴转速请求、仅用舵保持航向时的仿真结果，不是实船性能保证，也不适用于 6 kW 配置。

| 附加载荷 | 满油门航速 | 电机电输入功率 |
|---:|---:|---:|
| 0 kg | 15.21 km/h | 40 kW |
| 150 kg | 14.75 km/h | 40 kW |
| 300 kg | 14.33 km/h | 40 kW |

![基线固定油门曲线](docs/validation/baseline_2026-11-21/throttle_curves.png)

基线完成 29 个 Gazebo/ROS 工况的运行检查，以及同一能源模型的 2 小时计算。**运行检查通过不等于物理模型全部验收**；零舵转圈已经作为明确失败项单独记录。两小时能源计算不是两小时完整六自由度航行。[公开基线报告与摘要](docs/validation/baseline_2026-11-21/README.md)。

## 环境与获取

当前验证环境是 ROS 2 Jazzy / Gazebo Harmonic。本地惯性适配固定使用 gz-sim 8.15.0、gz-physics 7.8.0 和对应 DART，不能直接假设其他版本兼容。

```bash
git lfs install
git clone https://github.com/luyunhong12/usv-experimental-6m.git
cd usv-experimental-6m
git lfs pull
mkdir -p dependencies/src
vcs import dependencies/src < dependencies/sources.repos
```

还需安装 ROS/Gazebo 系统依赖、Python 工具依赖及获取悉尼场景资产。详情见 [安装运行说明](docs/getting_started/README.md) 和 [依赖说明](dependencies/README.md)。第三方场景缓存、外部设备手册、构建产物和完整测试原始数据没有放进主仓库。**尚未完成全新机器的一键安装验收**，不要把下面的构建命令视为自动安装所有依赖。

## 打开水域与开船

在依赖已安装、场景资源可用的环境：

```bash
./usv build
./usv run                         # 悉尼水域图形窗口
# 或者：./usv run headless:=True   # 无窗口测试
```

看到 `Boat ready` 后，另开终端执行：

```bash
./usv drive --throttle 0.3 --duration 10
./usv drive --throttle -0.3 --duration 10
./usv drive --throttle 0.3 --steering-deg -20 --duration 10
./usv energy --duration 30
```

油门是有符号的归一化桨轴转速请求，不是功率百分比。舵角是物理转角，船尾推进器的负转角对应前进时左转。手动指令不自动保持航向；停止指令后停推，但船仍会滑行。默认分区为 `usv_platform15`，自定义时启动与操控终端必须一致。同一艘船不要同时发送多种控制指令。

## 文件结构

```text
usv-experimental-6m/
├── design/                 # 原始船模、设计依据和独立备份
├── ros_ws/src/             # 自有 ROS / Gazebo 包
│   ├── usv_description/    # 外观、碰撞、装配和物理配置
│   ├── usv_simulation/     # 水动力、推进与仿真场景
│   ├── usv_physics/        # DART 动态惯性适配
│   ├── usv_energy/         # 油电消耗与保护
│   ├── usv_control/        # ROS 控制与传感器接口
│   ├── usv_bringup/        # 整船配置、统一启动
│   └── usv_ardupilot/      # 历史适配，当前暂缓联调
├── dependencies/           # 固定上游版本、最小源码片段与补丁
├── tools/                  # 安装、建模、参数计算、分析和手动操作
├── tests/                  # 单元、集成、场景与回归
├── datasets/               # 构建输入与历史数据校验索引
├── docs/                   # 使用、架构、依据、验证和过程
└── artifacts/              # 本地原始记录与备份，不进入 Git
```

原始 Blender 文件和较大的网格使用 Git LFS。部分历史文档引用本地 artifacts 路径；这不表示文件已上传，公开可读的主要结果见上面的基线报告。完整过程没有被删除。

## 测试与复现

```bash
./usv test
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
python3 tests/integration/validate_platform_launcher.py \
  --output artifacts/my_startup_test --kill-controller
python3 tests/regression/run_performance_suite.py \
  --spec tests/scenarios/performance.json --output artifacts/my_performance_run
```

每次试验使用新的输出目录，保留场景快照、遥测 CSV、健康日志及判定。原生测试需要本机 ROS DDS / Gazebo Transport 通信权限。仓库整理后已重新构建 12 个包，通过 10 项 C++、12 项 Python、5 项设计接口测试，并完成启动和四个操纵工况回归；这是本地验证，不是假称 GitHub CI 已验收。

## 已知问题与下一步

1. **高速零舵被动航向不稳定**：首先检查侧向/偏航受力、附加质量耦合、受力位置及艉部附体，使用零舵扰动、左右对称及步长对照验证，不能只增大航向控制增益。
2. **水动力未标定**：当前阻力、附加质量、侧向力和偏航阻尼仍有工程假设；15 km/h 对输入参数敏感。
3. **高速与复杂海况不足**：兴波、滑行、砰击、螺旋桨出水/空化，以及更大风浪尚未充分覆盖。
4. **能源与热模型不完整**：没有完整电机、驱动器、线缆热模型和电芯差异；燃油采用准静态液面。
5. **装配与结构边界**：刚体碰撞不等于结构变形、柔性线束、损伤、进水或制造认证。
6. **发布可复现性**：仍需全新环境安装验证、第三方素材分发核查和大型数据附件发布。

物理问题稳定后，再建立定位融合与 Nav2 船舶适配，重新验收 ArduPilot。两种控制模式必须明确执行器所有权，不能同时叠加控制。[集成规划](docs/integration/README.md) · [当前问题清单](docs/validation/known_issues.md)。

## 文档、贡献与许可

[文档索引](docs/README.md) · [参数来源](docs/parameters/README.md) · [建模说明](docs/modeling/README.md) · [贡献约定](CONTRIBUTING.md)

自有代码沿用包中 Apache-2.0 声明；第三方源码保留原许可。模型、设备外观和外部资料不因代码许可而自动获得统一授权。未经确认可再分发的外部手册和场景缓存不上传。详细范围见 [第三方与资产说明](THIRD_PARTY_NOTICES.md)。不提供实船安全、适航或预测精度保证。
