# 第四步：深度检查与独立环境复现

执行范围：Code Supervisor deep，以及从公开 GitHub 检出、Git LFS 校验、固定 VRX 和官方素材下载、独立安装构建与 ROS/Gazebo 运行。不接 ArduPilot/Nav2。

安装过程见 [容器复现指南](../../../getting_started/container.md)；可复用配置是 [Dockerfile](../../../../tools/setup/Dockerfile)。先前实时源提供 gz-physics 7.9.0，保持 EXACT 检查后定位并通过官方签名快照固定至 7.8.0。原失败日志保留，不伪称首次直接安装成功。

独立容器使用源码提交 `ab1327c`，未挂载主机源码、ROS 或构建产物。软件渲染下的首轮测试因短墙钟窗口而未达到仿真时间/转向要求；修正测试观察时间配置与结束窗口后，需要以最终结果为准。原始失败证据与最终测试脚本版本分别记录。

## 已完成结果

- [Code Supervisor deep](deep/report.txt) 全部通过，退出码 0，源码在检查过程中未变化；[机器可读报告](deep/report.json) 保留 Git、源码与配置摘要。
- 12 项 C++、43 项 Python、5 项设计接口测试通过。选定核心头文件行覆盖率 **97.47%**，超过原 80% 门槛；不是整个插件或整个仓库的覆盖率。
- ASan/UBSan 测试通过，保持泄漏检测；[执行日志](deep/cpp-sanitizers/sanitizer-tests.log)。
- `SteeringIntegral.hh` 的 12 个有限二元算子变异全部被测试杀死；[结果](deep/cpp-mutation/cpp-mutation.json)。不宣称全工程变异覆盖或线程安全证明。
- 原生惯性 force/torque 与 forward/reverse/left/right/reversal 通过；[运行日志](deep/usv-runtime-regression.log)。
- 独立容器在提交 `7e33723` 清除旧 build/install/log 后，12 个包重新构建成功，[构建记录](container/build-final.log) 与 [单元测试](container/test-final.log) 保留。原始首次版本不匹配见 [失败记录](container/build.log)，依赖固定过程见 [安装日志](container/physics-pin.log)。
- 完整悉尼场景的公开入口前进、倒车、左右转向、释放后停推均通过；[结果](container/public_cli_final/result.json) 与 [实际遥测](container/public_cli_final/telemetry.json)。控制进程退出后平台自动结束、无活动子进程：[结果](container/controller_exit_final/launcher_controller_exit.json)。使用 180 s 启动观察、20 s 指令；物理门槛没有降低。首次失败结果分别保留在 `container/controller_exit/`、`container/public_cli/`。

安装配方 [实际构建记录](container/image-build.log)；系统 [软件包清单](container/packages.tsv)。容器首次源码为 `ab1327c`，诊断时只更新两个测试脚本，随后取回已提交的 `7e33723` 并全新重建。入口诊断脚本与 `7e33723` 内容一致；独立原生可靠性套件另有完整版本绑定，见第三步最终报告。Standalone 入口结果不是 `trial_evidence` 密封套件，不能与密封原生航行证据混称。

## 仍然存在的边界

deep 是已配置规则及执行路径检查，不是实船认证或全部代码无缺陷证明。高油门倒车扰动恢复仍不合格；参数仍为工程近似。容器验证的是 amd64 无窗口运行与接口，未验证任意 GPU/GUI、其他系统版本或长时间硬件故障。依赖配方固定关键物理包，其他系统包未完全冻结，未来软件源变化需要重新验证。

