# 6 米实验无人船 · VRX 工程仿真平台

一个以**可解释的船体物理、可追溯的工程参数和可重复测试**为目标的无人船仿真项目。基于 ROS 2 Jazzy、Gazebo Harmonic 和 VRX 悉尼赛艇水域，当前重点是中央单机、桨舵一体推进的 6 米船模，同时保留双固定推进器原始设计。

> **项目定位：工程仿真平台，不是经过海试标定的实船数字孪生。** 没有完整实船数据时，使用明确标注来源和假设的参数。能够运行、画面精细、控制器能保持航向，都不能代替物理验证。
>
> **本轮完成升级电池重布置：减少约 168 kg 配重，满油轻载约 15.10 km/h，22 项工况通过原门槛。** 倒车偏航略增而非改善，300 s 末段约 0.454°/s；本轮未接入 ArduPilot/Nav2。见 [布置与验收](docs/validation/layout_2026-11-21/README.md)，开船请看 [ROS 操作指南](docs/getting_started/README.md)。

## 船长什么样，结构是什么

**以下是设计渲染图和 Gazebo 仿真截图，目前没有实船照片。** 设计图用于说明结构；历史截图用于展示外观，不作为最新物理测试的证明。

![中央单机船体设计总览](design/azimuth_thruster/图片/第二船体_总览.png)

*中央单机方案原始设计总览，早于当前传感器装配。原始模型与导出方法见 [建模说明](docs/modeling/README.md)。*

| 桨舵一体结构细节 | VRX 悉尼水域实景截图 |
|---|---|
| ![中央推进器转向设计](design/azimuth_thruster/图片/中央推进器_转向35度.png) | ![悉尼水域中的仿真船](docs/assets/vrx_sydney_2026-11-20.png) |
| 原始设计中的 35° 转向姿态，展示安装架、转向轴、支腿和螺旋桨；不是运行关节测量。 | 2026-11-20 Gazebo 截图；这是仿真水域，不是实船拍摄。[图像来源](docs/validation/evidence/manifest.json)。 |

```mermaid
flowchart LR
    Hull[船体与固定设备] --> Steering[转向关节]
    Steering --> Unit[可偏转的支腿与电机组件]
    Unit --> Shaft[桨轴旋转关节]
    Shaft --> Prop[螺旋桨]
    Fuel[燃油消耗] --> Mass[质量、重心与惯性更新]
    Mass --> Hull
    Energy[电池与发电充电模型] --> Limits[可用功率与保护限制]
    Limits --> Shaft
```

这是机械关系与仿真耦合示意，不是制造图或完整接线图。船体、甲板和固定设备计入质量台账；外观、刚体接触和水动力采用各自合适的几何表示。[装配与碰撞记录](docs/history/2026-11-19_物理覆盖与精细碰撞.md) · [转向扫碰记录](docs/history/2026-11-19_转向桨叶扫碰与单机估算.md) · [逐件质量台账](ros_ws/src/usv_description/config/component_mass_ledger.json)。

## 为什么做这个项目

希望先把“船”本身做好，再接自主航行系统：能合理漂浮和随浪、推力与功率相互约束、载荷改变吃水和操纵表现、油电消耗影响可用动力、燃油减少影响质量和惯性。之后再讨论飞控和导航算法的表现，避免控制器掩盖仿真对象的问题。

工程原则：

1. **外观、碰撞、浮力几何各有职责。** 外观保留精细结构，碰撞描述刚体接触，浮力采用封闭船体与分布式压力；不以整船网格直接替代所有物理。
2. **参数有唯一来源。** 质量和功耗使用逐件台账，区分工程估算、厂家参考、几何计算和未标定系数；历史参数快照不能冒充当前配置。
3. **力、能量与质量相互一致。** 推进受到电功率和转矩限制，舵机消耗能源；燃油用量与刚体质量、重心和完整惯性张量联动。
4. **用试验揭示问题。** 记录原始遥测、工况、源码版本与失败案例；不通过放宽检查或修改原始数据得到“通过”。
5. **先物理，后控制与导航。** 保留手动/ROS 控制以便验证，不急着堆叠飞控和任务系统。

## 每个模块的依据与证据

我们希望尽可能接近真实船舶，但**理论有依据、参数有来源、代码通过测试，是三个不同层次**。厂家额定值只能支持相应设备的参考工况；工程估算和仿真通过都不能代替海试。下面每个模块都给出可追溯入口，完整公式、来源适用范围和未解决问题见 [科学依据与证据地图](docs/parameters/evidence_map.md)。

| 模块与实现思路 | 参数、过程与验证材料 | 尚未证明的部分 |
|---|---|---|
| **船体、装配与碰撞**：保留原始设计，分别生成外观、接触和封闭浮力几何 | [建模入口](docs/modeling/README.md)、[物理覆盖](docs/history/2026-11-19_物理覆盖与精细碰撞.md)、[扫碰检查](docs/history/2026-11-19_转向桨叶扫碰与单机估算.md) | 刚体接触不是强度、变形、制造净空认证 |
| **质量与功耗**：逐件质量加权求重心，平行轴定理合成惯性；按设备工作状态汇总功耗 | [权威台账](ros_ws/src/usv_description/config/component_mass_ledger.json)、[台账推导与组合工况](docs/history/2026-11-21_逐件台账与载荷油量风浪流验证.md)、[公开摘要](docs/validation/evidence/README.md) | 材料密度、结构厚度、安装位置和部分设备功耗仍是工程值 |
| **浮力与随浪**：封闭体积静水浮力、浸水表面压力积分，使用 VRX 波场 | [静水推导与验证](docs/history/2026-11-19_船体接入与静水试验.md)、[随浪方法](docs/history/2026-11-20_船体外观装配与随浪优化.md)、[随浪结果](docs/validation/evidence/waves_2026-11-20.json) | 没有实船波浪响应标定；未完整覆盖破波、砰击、滑行 |
| **阻力、附加质量与方向稳定性**：参考海洋航行器动力学框架，按几何求附体浸水面积及力臂，补全升阻力 | [理论与系数边界](docs/parameters/evidence_map.md)、[阻力配置](ros_ws/src/usv_description/config/resistance.json)、[附体参数](ros_ws/src/usv_description/config/appendage_hydrodynamics.json)、[15 项专项与对照数据](docs/validation/directional_stability_2026-11-21/README.md) | 系数未经 CFD/海试标定；仍有小幅配平偏航，附体阻力与集中船体阻力可能存在重复预算 |
| **推进与转向**：正反推、桨轴转矩/功率限制、舵角与速率约束 | [推进参数](ros_ws/src/usv_description/config/propulsion.json)、[厂家参考与建模假设](docs/history/2026-11-19_推进与操纵验证.md)、[当前操纵验收](docs/validation/layout_2026-11-21/README.md) | 6 kW 厂家参考不证明 40 kW 概念方案；缺少真实桨的全工况性能图与空化数据 |
| **油量、电量与保护**：能量平衡、电流积分、发电/充电限制、负载和保护状态 | [能源配置](ros_ws/src/usv_energy/config/energy.json)、[原理与设备来源](docs/history/2026-11-19_油电能源与感知验证.md)、[能源基线](docs/validation/baseline_2026-11-21/README.md) | 电池内阻、热参数及油耗曲线有估算；两小时能源计算不等于实际续航 |
| **燃油引起的动态质量**：准静态液面体积积分，更新质量、重心和完整惯性，回读物理后端 | [油箱参数](ros_ws/src/usv_description/config/fuel_tank.json)、[方法及后端验证](docs/history/2026-11-19_燃油质量重心惯性联动.md)、[公开测试摘要](docs/validation/evidence/fuel_mass_historical.json) | 未建模油液动态晃荡、进水及柔性结构 |
| **传感器与运行接口**：IMU、GPS、相机、点云、TF、健康检测；就绪检测、失联停推及退出清理 | [WAM-V 差距与接口改进](docs/history/2026-11-20_WAMV差距完善.md)、[启动与运行验证](docs/history/2026-11-21_阻力操纵与启动可靠性改进.md)、[完整场景回归](docs/validation/directional_stability_2026-11-21/full_sydney_smoke.json) | 传感器噪声未实机标定；未完成新机器安装与所有环境长期测试 |
| **ArduPilot / Nav2**：保留历史适配，先完成船体物理，再验收控制所有权与导航 | [集成规划与当前边界](docs/integration/README.md) | 本阶段没有完成这两套自主航行系统的集成验收 |

历史过程记录保留当时参数，不应与当前配置混用。当前参数入口是 [参数说明](docs/parameters/README.md)；已公开的历史附件及其校验值见 [证据清单](docs/validation/evidence/README.md)。

## 当前受力审计与操纵验证

[当前布置的四步验证](docs/validation/platform_readiness_2026-11-21/README.md)：第一步 19 项载荷/油量/风浪流短时筛查通过，覆盖 30% 直接油门，不代表高速或任意海况适用。第二步 9 项倒车距离任务及 40% 扰动恢复通过，60% 扰动恢复仍不通过，详见 [倒车任务验收](docs/validation/platform_readiness_2026-11-21/step2/README.md)。每步独立提交；本轮不接入 ArduPilot/Nav2。

最新 [配平复验](docs/validation/layout_2026-11-21/README.md) 给出分布式横流模型、实体配重推导、版本绑定、严格品质门槛及源码对应关系。初次人工操船建议 0.3 前进油门、-0.25 倒车油门；这不是自动限速或飞控航向保持。

对照试验区分了几何假不对称、估算质量分布和桨反扭矩的影响。修正镜像剖分后，对称质量且无外部桨力偶的诊断工况偏航接近数值零；正常船仍保留原质量与反扭矩。纵向阻力采用明确的船体/附体参考预算分配，横向及角向分解仍待识别。

新增直接操纵测试覆盖零舵长航、左右回转、停推、反推和倒车、航向触发蛇形操纵及重载低油量。判据、实际数值和限制见 [本轮报告](docs/validation/force_maneuvers_2026-11-21/README.md)。特别注意：较大倒车油门的持续零舵航行仍有明显偏航，不能把“能倒车、能转弯”当成“倒车自然直线稳定”。

## 上一轮方向稳定性修复（历史对照）

15 个专项工况通过，包括实际零舵、左右扰动释放、重载、低油量、1/2/4 ms 步长、旋转初始朝向和受力系数敏感性。另完成风浪流检查与完整悉尼场景操控回归。零舵无附加载荷末段航速约 **15.15 km/h**；60 秒对照中累计偏航从约 1,928° 降到约 4.8°。

![零舵受力修复对照](docs/validation/directional_stability_2026-11-21/before_after.png)

这是上一轮工程模型的验证，不是海试认证；最新几何和预算结果见上面的受力报告。原型阻力/附加质量未重新调参，补充的升阻力系数仍需实测或 CFD 校准。后面的“历史性能基线”章节保留修复前数据；当前配重方案另列。

## 当前设备布置与性能

40 kW 概念方案把四块升级电池及托架改为前后各一对，取消约 167.86 kg 纵向配重，保留约 32.05 kg 横向配重。满油总质量约 1571.17 kg；电池容量、推进功率与水动力预算不变，外形、碰撞和整船惯性同步更新。

| 当前静水工况 | 直接满油门、零舵航速 |
|---|---:|
| 无附加载荷、60 L 燃油 | 约 15.10 km/h |
| 300 kg 附加载荷、6 L 燃油 | 约 14.35 km/h |

满油轻载达到约 15 km/h，但不是所有载荷均能达到。300 s 倒车末段偏航从上一版约 0.429°/s 增至约 0.454°/s，不能声称倒车也改善了。[推导、包络与固定版本对照](docs/validation/layout_2026-11-21/README.md) · [质量台账](ros_ws/src/usv_description/config/component_mass_ledger.json)。

## 历史性能基线：15 km/h 是哪个方案

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

每次试验使用新的输出目录，保留场景快照、遥测 CSV、健康日志及判定；复用必须显式 `--resume` 且版本、工况与证据一致。原生测试需要本机 ROS DDS / Gazebo Transport 通信权限。最新 Code Supervisor full 与构建测试通过：12 项 C++、42 项 Python、5 项设计接口测试。操纵品质另行核对原有门槛；这是本地验证，不是假称 GitHub CI 或实船验收。完整证据、已执行范围和复现说明见 [接入前报告](docs/validation/layout_2026-11-21/README.md)。

## 已知问题与下一步

1. **小幅配平偏航与模型标定**：原来的高速零舵发散转圈已在专项工况中修复；镜像几何修正后仍有约 0.042°/秒的配平偏航，质量不对称、反扭矩和姿态耦合已通过反事实试验区分，但参数仍未实测。不能把有界稳定等同于绝对直线，也不能把工程升阻力系数当作海试数据。
2. **水动力未标定**：当前阻力、附加质量、侧向力和偏航阻尼仍有工程假设；15 km/h 对输入参数敏感。
3. **高速与复杂海况不足**：兴波、滑行、砰击、螺旋桨出水/空化，以及更大风浪尚未充分覆盖。
4. **能源与热模型不完整**：没有完整电机、驱动器、线缆热模型和电芯差异；燃油采用准静态液面。
5. **装配与结构边界**：刚体碰撞不等于结构变形、柔性线束、损伤、进水或制造认证。
6. **持续倒车方向稳定性**：60% 零舵倒车 300 s 末段约 0.454°/s，累计约 113.7°。通过 0.5°/s 门槛，但比上一版略差，不能称自然直航。
7. **布置与实装边界**：配重由约 199.91 kg 减至 32.05 kg，轻载速度约 15.10 km/h；实际线长、压降、安装强度和完整内部干涉仍待设计核查。详见 [布置复验](docs/validation/layout_2026-11-21/README.md)。
8. **发布可复现性**：仍需全新环境安装验证、第三方素材分发核查和大型数据附件发布。

物理问题稳定后，再建立定位融合与 Nav2 船舶适配，重新验收 ArduPilot。两种控制模式必须明确执行器所有权，不能同时叠加控制。[集成规划](docs/integration/README.md) · [当前问题清单](docs/validation/known_issues.md)。

## 遇到问题，去哪里查

| 想了解或排查的问题 | 优先阅读 |
|---|---|
| 安装依赖、打开悉尼水域、不能启动、如何开船 | [安装运行](docs/getting_started/README.md)、[依赖说明](dependencies/README.md) |
| 船体结构、模型导出、碰撞与推进器装配 | [建模说明](docs/modeling/README.md)、[扫碰记录](docs/history/2026-11-19_转向桨叶扫碰与单机估算.md) |
| 为什么取这个质量、密度、功率、油量或水动力系数 | [参数入口](docs/parameters/README.md)、[科学依据地图](docs/parameters/evidence_map.md)、[逐件台账](ros_ws/src/usv_description/config/component_mass_ledger.json) |
| 不随浪、漂浮姿态是否合理 | [静水试验](docs/history/2026-11-19_船体接入与静水试验.md)、[随浪试验](docs/history/2026-11-20_船体外观装配与随浪优化.md) |
| 高速转圈、零舵偏航、倒车/转向和航速 | [当前操纵品质](docs/validation/layout_2026-11-21/README.md)；[受力归因](docs/validation/force_maneuvers_2026-11-21/README.md)；对照旧版本看 [历史性能基线](docs/validation/baseline_2026-11-21/README.md) |
| 油电消耗、质量重心变化是否真的接入 | [能源建模](docs/history/2026-11-19_油电能源与感知验证.md)、[动态惯性验证](docs/history/2026-11-19_燃油质量重心惯性联动.md) |
| 还有什么没做好、下一步怎么接飞控/导航 | [已知问题](docs/validation/known_issues.md)、[集成规划](docs/integration/README.md) |
| 如何核对测试结果，为什么有的历史附件打不开 | [公开证据及来源](docs/validation/evidence/README.md)、[全部文档索引](docs/README.md)；完整 artifacts 仍在本地，不假称已全部公开 |

## 文档、贡献与许可

[文档索引](docs/README.md) · [参数来源](docs/parameters/README.md) · [建模说明](docs/modeling/README.md) · [贡献约定](CONTRIBUTING.md)

自有代码沿用包中 Apache-2.0 声明；第三方源码保留原许可。模型、设备外观和外部资料不因代码许可而自动获得统一授权。未经确认可再分发的外部手册和场景缓存不上传。详细范围见 [第三方与资产说明](THIRD_PARTY_NOTICES.md)。不提供实船安全、适航或预测精度保证。
