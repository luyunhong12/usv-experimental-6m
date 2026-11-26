# 大修前版本保存与范围

旧版保存后，用户已批准在`vrx-vessel-rebuild`分支先重建自己的船；官方飞控接口与双船切换后置。

## 回退基线

- 已提交基线：`f87434c1978012452fbe1cfee80fbe5a84f9c204`。
- 远程标签：`pre-overhaul-2026-11-26`。标签只包含已提交内容，不包含工作区未提交文件。
- 本地快照目录：`artifacts/backups/pre-overhaul-20261126T023151Z/`。完成标志为`manifest.json`：生成前必须通过归档文件逐项SHA256/符号链接核对与`git bundle verify`。
- `working-tree.tar.gz`保存Git跟踪文件及未忽略的未跟踪文件；`history.bundle`保存Git历史；补丁、状态和清单同时保留。
- 忽略的测试原数据、第三方源码、构建目录和容器镜像留在原处，没有复制进本次代码快照；不能称为完整可离线恢复的机器镜像。符号链接保留链接本身，外部目标不打包。
- 恢复时先在新目录克隆bundle，再覆盖工作区归档并依据manifest核对；不要直接覆盖现有工作区。

## 当前固定VRX船模属性

依据`dependencies/sources.repos`固定源码`7609d1bd90ce7edb29d040a082f949e8b089c864`：

- `wamv_description/urdf/wamv_base.urdf.xacro`：主体质量180 kg，主体惯性对角值120、393、446 kg·m²；不是装配后总质量。碰撞采用简化几何体。
- `wamv_description/urdf/battery.xacro`：每块电池23.5 kg，包含视觉、碰撞、惯性；不能据此声称有电量消耗仿真。
- `wamv_description/urdf/thrusters/engine.xacro`：每套发动机15 kg、螺旋桨0.5 kg；推进器有转向关节和桨旋转关节。
- `wamv_gazebo/urdf/dynamics/wamv_gazebo_dynamics_plugin.xacro`：双浮筒分别计算浮力/波面响应，配置浮筒长度4.9 m、半径0.213 m和作用点；六自由度运动配合线性/二次阻尼。该配置列出的附加质量xDotU、yDotV、nDotR均为0，不能把支持参数等同启用非零模型。
- `wamv_gazebo/urdf/thruster_layouts/wamv_gazebo_thruster_config.xacro`：推力系数0.004422、水密度1000 kg/m³、推进计算桨直径0.2 m，独立偏转控制。其推进直径与碰撞包络不是同一概念。
- `wamv_gazebo/urdf/wamv_gazebo.urdf.xacro`：可选GPS、IMU、相机、激光雷达、声学定位和真值输出；有开关，不是每次都全部启用。默认H布局为两套艉部推进器。
- `vrx_gz/worlds/sydney_regatta.sdf`：环境有风插件，是否产生风取决于配置；浪由场景波场与船体插件配合。

标准模型不是完整CFD、结构强度、材料破坏、发动机热力学或续航模型。本次只备份、核对源码与写文档，未改运行代码，无新增物理/导航通过结论。

阶段结果：[无鳍固定载荷候选](simple-v1/README.md)、[静水恢复](static-recovery/README.md)。

继续验证：[基准船倒车与随浪](baseline-dynamics/README.md)、[六部位碰撞](contacts/README.md)、[高功率倒车失败与对照](high-power-reverse/README.md)。当时的高功率无鳍方案未达标，原始失败记录保留；后续修正结果见下文。

## 高功率无鳍倒车的后续核算

[湿表面与横剖面重算](section-hydrodynamics/README.md)记录几何依据、积分收敛、可选横移/偏航附加惯性耦合及隔离候选。重算不等于验收通过；目前不得将新参数作为默认合格模型。

[船体分离区斜流修正](hull-separation/README.md)：按同源湿剖面增加下游压力恢复差额，未调大原阻尼；固定载荷静水60%倒车左右扰动、半步长及240 s零舵测试通过原门槛。仍有残余偏航和未覆盖工况，不切默认、不加鳍、不接飞控。
