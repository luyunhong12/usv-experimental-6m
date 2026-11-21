# Code Supervisor 接入

手动检查入口是 `./usv review`，不会在启动仿真、编辑文件或 Git 操作时自动运行。
每次结果以 `artifacts/validation/supervisor/` 下独立运行报告为准。

## 手动使用

默认使用同级目录 `code-supervisor/.venv/bin/supervisor`；其他安装位置可通过
`CODE_SUPERVISOR_BIN` 指定。入口会加载本机 ROS Jazzy 依赖搜索路径，不自动安装依赖。

```bash
./usv review --profile quick
./usv review --profile full --run-commands
./usv review --profile deep --run-commands
```

| 模式 | 执行范围 |
| --- | --- |
| quick | Python 中文文档、Black、Ruff、C++ 中文文档及参数说明 |
| full | quick + C++ 复杂度/重复代码、核心头文件行覆盖率、完整构建和原有单元测试 |
| deep | full + ASan/UBSan/泄漏检测、转向积分有限算子变异、原生惯性与推进集成回归 |

full/deep 必须显式传入 `--run-commands`。缺少授权开关、工具或运行环境时记为未执行或错误，不能视为通过。
ASan 的泄漏检测需要不受 ptrace 限制的运行环境；不要关闭泄漏检查来绕过沙箱限制。

报告包含 `report.txt`、`report.json`、`report.sarif` 及各工具证据。
退出码 0 表示本次配置范围通过，1 表示发现问题，2 表示工具错误、证据不足或源文件变化。
构建和仿真会写入生成物；本入口不执行 Git 提交或推送。

## 检查范围

- 自有 ROS 包、tools、tests，以及方位推进器 Python 仿真接口；包含未跟踪的自有源码。
- 两个主 SDF 和能源、推进、油箱、附体、设备台账、附加质量、阻力 7 份核心 JSON 的逐字段中文说明。
- C++ 复杂度阈值保持 10，含上游衍生 `Physics.cc`，没有按文件豁免。
- `./usv build && ./usv test` 串行运行；构建失败时不继续使用旧二进制执行单元测试。
- `tests/native/CMakeLists.txt` 复用生产头文件及现有 12 个 C++ 测试，在隔离副本中重新构建。
  覆盖率使用 GCC/gcovr，最低要求保持适配器默认的 80%；地址/未定义行为检测使用独立 Debug 构建。
  两者都不代替完整 colcon 构建。
- 覆盖率选择能源和传感器头文件，以及受力审计、附体、转向积分、阻力、附加质量、燃油质量、推进模型、
  转向参考、波场、波压和静水力头文件。准确清单以 `supervisor.toml` 的 `settings.cpp-coverage.files` 为准。
- 变异测试只覆盖 `SteeringIntegral.hh` 的全部有限二元算子候选；每个变异重新构建并执行实际断言。
  预算未完成、测试未执行和编译无效不算杀死变异；没有把少量候选宣称为全工程变异覆盖。
- deep 调用 `tools/analysis/deep_runtime.sh`，该命令先确认完整构建成功再运行：原生惯性 force/torque，以及 forward/reverse/left/right/reversal。
  每项使用独立 Gazebo 通信分区，证据写入新建 `artifacts/validation/deep-runtime/` 子目录。
  失败时返回非零并停止后续工况，后续项不能算作已通过。

## 上游衍生实现

`ros_ws/src/usv_physics/src/Physics.cc` 保留原有许可及英文说明，并补充中文接口文档。
复杂函数按实体创建、命令处理、状态回写等职责拆分；计算公式、比较容差及执行顺序保持原有定义。
对应差异同步记录在 `dependencies/patches/gz-sim8-runtime-inertia.patch`，固定上游原文仍保留在
`dependencies/vendor/gz-sim-physics/`。重构后的构建和运行证据不能替代所有上游 Gazebo 功能的回归测试。

## 证据边界

- 第三方 dependencies、构建产物、场景资产、历史数据和 Blender 连接工具不属于自有源码扫描范围。
- 核心头文件的行覆盖率不是整个插件或整个项目的覆盖率；ASan/UBSan 只验证实际执行的路径。
  未启用全工程 clang-tidy、线程检测或全工程变异测试。
- 其余 JSON、其他 SDF、XML、ArduPilot 参数、shell 和 TOML 尚未纳入逐参数文档检查。
- 五种推进工况检查方向、限幅、停指令和转向等现有判据，不能证明持续零舵倒车的自然航向稳定性。
- 水动力标定、质量分布真实性、高速/恶劣海况、热模型和动态晃荡等边界见
  [当前已知问题](known_issues.md)。实船或 CFD 标定需要外部数据，不能靠静态规则或提高测试覆盖率完成。
