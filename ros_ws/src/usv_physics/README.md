# usv_physics：运行时质量与惯性适配

本包在工作区内编译 `libUsvPhysics.so`，替代自定义试验世界的 Physics 系统插件。原安装的 Gazebo、DART、VRX 源码均不修改。

固定兼容版本：ROS Jazzy 环境实际加载的 gz-sim **8.15.0**、gz-physics **7.8.0**、DART vendor 6.13 系列。系统 apt 另有 gz-physics 7.9，不能据其版本推断实际加载版本。CMake 固定前两者，DART 使用 `/opt/ros/jazzy/opt/gz_dartsim_vendor`，运行日志和链接检查确认来源。

上游 Physics 系统源码保存在 `../../../dependencies/vendor/gz-sim-physics/`，实际修改副本为 `src/Physics.cc`，差异为 `../../../dependencies/patches/gz-sim8-runtime-inertia.patch`。DART 的 RetrieveWorld 原生接口头文件与许可保存在 `../../../dependencies/vendor/gz-physics7-dartsim/`。不另建物理后端，不通过符号注入或修改系统库绕过接口。

物理步顺序：

1. 船舶能源/质量系统在 PreUpdate 产生 `RuntimeInertiaCmd`，并同步 ECM 惯性供浮力/阻尼读取重心。
2. 本 Physics 系统在 Update 读取命令，经现有 DART RetrieveWorld 功能找到原生 BodyNode，调用 `setInertia` 更新质量、局部重心和完整惯性矩阵。
3. 从 BodyNode 读回真实设置值，写入 `RuntimeInertiaApplied`，删除命令，然后按原 Gazebo 流程计算重力、接触和动力学。
4. 船模在 PostUpdate 发布原生读回数据与整船组合属性；缺少实际应用回执会报错，不静默把 ECM 数字当成物理更新成功。

适配范围：顶层模型、当前开链三刚体船模。拒绝嵌套模型、带 `_welded_mirror_` 的分裂刚体和 fluid-added-mass；新增闭链、可拆卸连接或流体附加质量时必须扩展适配层。原 DART LinkInfo 的初始惯性缓存不用于本船更新路径，涉及拆分/焊接的路径因此明确禁止。

更新不改写广义位置与速度，不重建船、重置关节或强制瞬移。耗油采用低流率、随船速度离开系统的质量流假设，不加虚假的火箭推力。准静态油面变化没有动态液体相对运动量模型，不能作为大幅晃荡或高速操纵流体模型。

验证脚本 `tests/integration/validate_inertia_backend.py --mode force/torque（从项目根目录执行）` 使用独立无重力测试体检查 F=ma、力矩/角加速度、真实重心读回与更新时速度连续性。`UsvInertiaResponseProbe` 仅用于这些测试，不加载到生产船模。

本轮附加质量由 HullHydrostatics 的力级分区耦合实现，并不设置 SDF `fluid_added_mass` 或改变刚体重量；因此不改变上述运行时燃油惯性适配边界。默认参数及 4/2 ms 收敛和敏感性记录见 WAMV 差距完善文档。
