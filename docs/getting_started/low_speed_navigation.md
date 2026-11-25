# 低速自动航行验收

这是接入 Nav2 前的 ArduRover 自动控制验收入口。使用固定容器中的船体基线，独立实例 43、Gazebo 分区和 ROS 域；不连接桌面实例 10，也不操作实体飞控。

```bash
./usv navigation-test
# 指定独立结果目录名：
./usv navigation-test low_speed_trial_01
# 只测纯 GCS 心跳丢失，仍维持独立 RC UDP 输入：
./usv navigation-test gcs_loss_trial_01 --failsafe-only
```

依赖本机已构建的 `usv-simulation:sitl-gui`、`usv-sitl:rover-4.7.3` 镜像、Docker 权限，以及 `dependencies/vendor/python` 中的 pymavlink。脚本不下载依赖，不重建镜像。默认墙钟超时 900 秒；较慢机器可在标签后加 `--timeout 1500`。

试验使用无障碍静水，并关闭相机/GPU 雷达渲染；保留物理、IMU、GPS、飞控估计和执行器。这样先隔离控制问题，不将卫星影像与仿真码头错位混入航线验收。结果不能代替完整悉尼场景避障、风浪流包络或实船验证。

过程依次为：

1. 读回 `WP_SPEED=1.0 m/s`、`RTL_SPEED=0.8 m/s`，等待 EKF 连续就绪 2 秒，然后正常解锁。
2. 上传相对出生位置的三航点：东 40 m、东 40/北 30 m、东 10/北 30 m，执行 AUTO。
3. LOITER 120 秒，其中施加东 0.15、北 0.1 m/s 水流；随后清除流场。
4. HOLD 30 秒，最后 10 秒检查推进请求归零、纵向速度低于 0.3 m/s；切 MANUAL 并确认模式，再 RTL。
5. 返回出生位置 6 m 内后进入 GUIDED，确认运动，在GUIDED连续实测运动后，再仅切断 GCS 心跳。保持独立 RC 和 JSON 物理链，检查 HOLD、停推、失联报告及 RC 健康。
6. 退出时请求 HOLD、上锁并清理本次两个容器。Ctrl+C 同样清理；不删除结果。

所有必需检查默认失败，空结果和未完成阶段不能通过。AUTO 预热后速度中位数门槛 0.5–1.3 m/s、95%分位不高于 1.5 m/s；沿用历史航线走廊 15 m、LOITER 半径 12 m 的基础验收门槛。这些较宽门槛仅用于第一阶段，不代表米级跟踪精度；应同时查看实际误差。

结果位于 `artifacts/navigation/<标签>/`：`run/result.json`、物理/能源/航迹 CSV、EKF 记录、JSON 桥日志、原始 DataFlash、镜像摘要和运行输入摘要。失败或中断使用新标签重跑，禁止覆盖旧目录。参数和测试代码在启动时复制到本次输入目录，再只读挂载。

低速覆盖参数位于 `ros_ws/src/usv_ardupilot/config/navigation_low_speed.parm`，在 `boat.parm,platform15.parm` 后加载。它包含低速工作点的速度前馈、规划与保持参数；不把局部低速拟合外推到高速方案。`run_sitl.py --profile navigation` 也支持此配置，但已有 EEPROM 会保留参数，因此验收使用独立数据目录与 `--wipe`，不要清除日常实例的数据。

这一步尚未提供 Nav2、代价地图或障碍物避让。手动阶段验证模拟RC到执行器的前进、左右转向和停推，不代表实体手柄、QGC摇杆校准或所有倒车工况均已验收。

## 2026-11-25 验收加严

手动阶段增加四个各 8 秒的原生 RC 输入：前进、左舵、右舵、回中停推；检查推进请求、船速和实际舵机关节角，而非仅检查 MANUAL 模式。它验证模拟 RC 到执行器的接管，不代表某一款实体手柄或 QGC 摇杆界面已经校准。

失联检查只使用故障窗口内的状态文字，并要求最后至少 5 秒连续 HOLD/停推，拒绝大于 0.5 秒的物理采样间隔；RC 健康记录也检查覆盖时长与间隔。航线误差改为到 MISSION_CURRENT 所指航段的距离，不能与旧版“整条折线最近距离”直接比较。

桌面 Compose 改为加载 navigation_low_speed.parm，使用独立持久目录 artifacts/container-runtime/sitl-navigation。原 sitl 目录保留，避免旧 EEPROM 覆盖新的低速默认值；再次使用这个新目录时仍应读回参数，不得把 defaults 当强制覆盖。船体采用主模型生成器，默认波浪增益沿用 platform.json 的 0.3；独立导航验收仍为静水。旧自定义生成场景已保存在 artifacts/diagnostics/navigation_repair_2026-11-25/original-generated。

测试入口启动时解析镜像不可变 ID，避免运行期间标签变化；可用 USV_SIM_IMAGE 指定待验收候选镜像。仅验收成功后才推广到桌面镜像。


## 双向低速跟踪配置

前一轮低速覆盖采用 ATC_STR_RAT_FF=0.3、PSC_POS_P=0.1、PSC_VEL_P=0.5、TURN_RADIUS=6，以及15°/s、5°/s²、0.03g限幅。它是历史对照，不能当作当前全部参数；本轮配置及采用状态见下方[首弯与保持报告](../validation/corner_hold_2026-11-25/README.md)。旧数据和未采用方案见[跟踪报告](../validation/navigation_tracking_2026-11-25/README.md)。

```bash
./usv navigation-test tracking_left_new
./usv navigation-test tracking_right_new --mirror-course
```

镜像只改变第二、第三航点的南北方向；每轮均使用独立新目录并执行完整验收。不得让两轮同时争用固定端口。新的 active_segment.csv 保存世界x/y、航段序号、仿真秒和实际航段距离，可用于分析峰值发生在哪一段。


## 首弯与有流保持增补

2026-11-25已采用参数与证据见[首弯与保持报告](../validation/corner_hold_2026-11-25/README.md)，以报告中的最终双向结果为采用依据。低速直航数据用于拟合 `CRUISE_SPEED=1 m/s`、`CRUISE_THROTTLE=9%`，并显式设置 `SPEED_MAX=1 m/s`，避免用局部前馈外推过高的摇杆速度。SPEED_MAX作用于速度控制模式，不能把它称为原始MANUAL油门的限速器。

已采用配置保留主动制动及前后回归，降低速度积分，提前规划减速；`LOIT_RADIUS=0`表示取消自由漂移死区，保持持续纠偏，不表示零定位误差或直接锁定船体位置。实际舵机速度、角度、力矩、船体惯性和水动力仍由原模型计算，高速platform15配置独立。

完整验收新增两个必需检查，共25项：记录飞控实际保持目标，稳定段（第15–120秒）至少连续覆盖100秒、相邻观测缺口不超过1秒、目标移动小于0.25 m；GPS到该目标最大距离不超过3 m。GPS及目标消息也必须保持新鲜。这是当前工况的工程门槛，不是实船精度保证。原切入点12 m门槛保留，两种距离不能混用。`result.json` 保存目标观测，`actuators.csv` 保存实际舵角、参考角和舵机功耗。

初始化逐项限速读回参数，避免批量请求漏回；仍须全部参数一致才正常解锁。失联准备至少运行12秒，然后确认连续至少2秒GUIDED实测纵向速度大于0.3 m/s、推进请求大于0.05，再断开心跳；60秒内未准备好则失败。失联后的18秒检查窗口不变。新结果额外保存 `outbound_motion_samples`，可审计故障注入前是否确实处于运动中。

## 恒流水流包络

通过 `--current-east`、`--current-north` 指定世界坐标水流分量（m/s），默认仍为东0.15、北0.10且仅LOITER阶段开启。添加 `--current-navigation` 后，相同水流也作用于AUTO和RTL；HOLD、手动接管及失联阶段清零，不能把其结果称为有流停船验收。例如：

```bash
./usv navigation-test north_current_trial --current-navigation --current-east 0 --current-north 0.15 --timeout 1200
```

方向、作用阶段和切换时刻保存在结果中。参数分量须有限且绝对值不超过1 m/s，此输入范围不代表模型已通过相应强流验证。实际结果、失败项和复现输入见[恒流方向与强度试验](../validation/current_envelope_2026-11-25/README.md)。
