# 2026-11-25 导航验收修复

## 改动及原因

- 按主模型生成器重建静水及 0.3 增益的场景，恢复与平台默认配置一致。原 1739 kg / gain=1.0 自定义生成场景备份于 `artifacts/diagnostics/navigation_repair_2026-11-25/original-generated`，没有删除设计源文件。重新生成的默认场景清单为 1576.343 kg / gain=0.3；这是工程模型预算，不是实测重量。
- 候选镜像干净构建；主船模及设备质量台账与工作区对比，只有绝对路径前缀需要归一，见 [模型比较](candidate-model-comparison.json)。没有修改船体受力或用航向控制掩盖自然偏航。
- MANUAL 验收改为四段各 8 s 的模拟 RC 操作，实际检查前进速度、推进请求、左右舵机角及回中停推。舵机遥测须新鲜。证明的是模拟原生 RC 到物理执行器链，不是 QGC 鼠标或实体手柄的校准。
- GCS 失联文本只在故障阶段采集；要求末段至少连续 5 s HOLD/停推，物理采样间隔不超过 0.5 s；RC 健康采样要求足够数量、时长及间隔。加入相应反例单元测试。
- 航线误差改为到 MISSION_CURRENT 指定的当前航段，保留原 15 m 基础门槛；不通过放宽门槛掩盖误差。旧版整条路线最短距离与新版指标不可直接比较。
- 低速配置逐项读回，包括速度、半径、加减速及失联参数；检查心跳新鲜度。测试开始解析镜像 ID 后固定使用，避免标签在运行中变化。
- 桌面 Compose 加载低速覆盖，WP_SPEED=1.0、RTL_SPEED=0.8 m/s；使用新的 `sitl-navigation` 持久目录，旧 `sitl` EEPROM/日志全部保留。MANUAL 仍能使用原动力范围，WP_SPEED 不限制手动满油门。

## 验收证据

构建与单元测试见 [日志](build-test.log)，Code Supervisor quick 见 [日志](quick-review.txt)。quick 不是 full/deep；本次没有宣称重新完成全部物理、覆盖率或海况验收。

最终 `navigation_repair_final_01` **23/23 通过，入口退出码 0**。完整构建 12 包、12 个 C++ 测试、62 个 Python 测试及 5 个接口测试通过；Code Supervisor quick 通过。

| 项目 | 本轮结果 |
| --- | --- |
| AUTO 航速中位数 / 95%分位 | 1.0092 / 1.0826 m/s |
| 当前目标航段最大偏差 | 9.755 m，低于原 15 m 基础门槛；不是米级精度 |
| LOITER 有流最大位置差 | 3.695 m |
| 左右实际舵角（阶段末段中位数） | −20.861° / +20.912° |
| 手动回中 | 推进请求归零，舵角约 −0.043°；船仍按惯性减速 |
| RTL | 回到 5.962 m，满足 6 m 基础范围，不是精确停靠 |
| GCS 失联持续停推 | 最后 7.976 s 连续 HOLD/零推进，RC 仍健康 |
| 桥接日志最大滞后 | 3358 个记录样本均为 0 s；采样日志不能排除记录间瞬态 |
| 估计器 | 读回 AHRS_EKF_TYPE=3 / EK3_ENABLE=1，正常解锁、EKF 检查通过 |

证据：[原始判定](result.json)、[追加指标](additional-metrics.json)、[航迹](track.csv.gz)、[物理遥测](propulsion.csv.gz)、[EKF](estimator.json.gz)、[冻结输入](input-sha256.txt)、[运行输入](runtime-inputs.json)、[镜像](images.json)、[飞控日志](sitl-console.log)。原始 JSON 帧、DataFlash 等保留在 `artifacts/navigation/navigation_repair_final_01/`。

桌面已切换到本轮通过的 simulation 镜像 `sha256:4064415880c9aa5a8ff9941f524e119f0394aaa274419ac686b63f7a2474f80d`，飞控镜像仍为 `sha256:c94217aaf947e46f224d32512f84bb64067f87dabe1a3d45906a1da64b34fbcf`。旧 simulation 标签另存 `usv-simulation:pre-navigation-repair-20261125`。桌面实际读回全部 14 个目标参数并确认未解锁 HOLD，见[读回](desktop-readback.json)；第二执行器控制者仍被拒绝，见[锁检查](lease-check.json)。重新生成的桌面/工作区场景质量、重心、姿态、波浪增益一致，见[场景核对](desktop-scene-comparison.json)。桌面本次只做启动、参数和锁检查，完整自动航行证据来自隔离静水实例。

失败轮的[摘要](failed-run-summary.json)及原始 `artifacts/navigation/navigation_repair_01/` 全部保留。代码仍有本会话以前的未提交改动，本轮没有创建 Git 发布标签或推送；镜像与输入摘要绑定不等于已经形成发布提交。

## 尚未覆盖

QGC Radio 默认端点提示不应通过伪造校准消除，见[桌面指南](../../getting_started/desktop_sitl.md)。自动试验不代表实体遥控器已校准。未进行地图配准、Nav2、定位丢失全部故障矩阵或实船标定；静水导航结果不能替代完整悉尼有浪场景的自动避障。

## 复跑发现的 JSON 时间开关错误

`navigation_repair_01` 保留为失败：当前航段误差 18.369 m，MANUAL 物理响应失败，RTL 超时，未执行失联阶段。约 270 s 后 JSON 桥出现持续 0.49–1.09 s 的时间滞后，超过既有 0.1 s 执行器新鲜度门槛，保护正确停推。不是通过降低门槛或修改水动力来修复。

根因在传输格式：Python 默认 `json.dumps` 产生 `"no_time_sync": true`，本地固定 ArduPilot 4.7.3 的 `SIM_JSON.cpp` 把值指针直接移动到冒号后，BOOLEAN 分支不跳空格；`strncasecmp` 不能识别带前导空格的 true，数值回退将它解释为 0。旧日志列出字段但没有 `Forcing use_time_sync=0`。现改为统一紧凑 JSON 编码，保留标准 JSON 布尔值，并补 wire 格式和非有限值回归测试。

由于上游在 no_time_sync 生效时会把 AHRS_EKF_TYPE 的默认值改成 10，本项目显式配置并读回 AHRS_EKF_TYPE=3、EK3_ENABLE=1，继续验证 EKF3，而非直接采用仿真真值估计器。没有修改 ArduPilot 源码。
