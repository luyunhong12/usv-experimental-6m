#!/usr/bin/env python3
"""根据性能记录生成航速、转向与制动报告。

Generate evidence-based curves, manoeuvre tables and an engineering report.
"""

# Resolve the checkout independently of the current working directory.
from pathlib import Path as _ProjectPath
import sys

# 项目脚本可从任意工作目录启动；建模工具保持在共享库之前。
sys.path.insert(
    0,
    str(
        next(
            p
            for p in _ProjectPath(__file__).resolve().parents
            if (p / "project.toml").is_file()
        )
        / "tools/lib"
    ),
)
sys.path.insert(
    0,
    str(
        next(
            p
            for p in _ProjectPath(__file__).resolve().parents
            if (p / "project.toml").is_file()
        )
        / "tools/modeling"
    ),
)
from project_paths import PROJECT_ROOT, WORKSPACE, tool_path
import argparse
import json
import hashlib
import os
from pathlib import Path
import numpy as np

os.environ.setdefault("MPLCONFIGDIR", "/tmp/usv_matplotlib")
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

p = argparse.ArgumentParser()
p.add_argument("--output", type=Path, required=True)
a = p.parse_args()
o = a.output.resolve()
w = WORKSPACE
spec = json.loads((o / "spec.json").read_text())
r = {k: json.loads((o / k / "result.json").read_text()) for k in spec}
energy = json.loads((o / "energy_2hour/result.json").read_text())
launcher = json.loads((o / "launcher_controller_exit.json").read_text())
before = json.loads((o / "attempts_clock_before/result.json").read_text())
cancel = json.loads((o / "cancellation_test/result.json").read_text())
abrupt = json.loads((o / "abrupt_exit_test/result.json").read_text())
fig, ax = plt.subplots(1, 2, figsize=(11, 4.5), constrained_layout=True)
for mass in [0, 150, 300]:
    m = [r[f"curve_{mass}_{t}"]["metrics"]["fixed"] for t in [25, 50, 75, 100]]
    ax[0].plot(
        [25, 50, 75, 100],
        [v["tail_speed_m_s"] * 3.6 for v in m],
        "o-",
        label=f"Payload {mass} kg",
    )
    ax[1].plot(
        [25, 50, 75, 100],
        [v["mean_motor_w"] / 1000 for v in m],
        "o-",
        label=f"Payload {mass} kg",
    )
ax[0].axhline(15, color="gray", ls="--", label="15 km/h reference")
ax[0].set_ylabel("Steady forward ground speed (km/h)")
ax[1].set_ylabel("Motor electrical input (kW)")
for a1 in ax:
    a1.set_xlabel("Fixed shaft-speed command (%)")
    a1.grid(alpha=0.3)
    a1.legend()
fig.suptitle("Calm water, full fuel; heading hold adjusts steering only")
fig.savefig(o / "throttle_curves.png", dpi=160)
plt.close(fig)
fig, axes = plt.subplots(1, 2, figsize=(11, 4.5), constrained_layout=True)
for axis, mass in zip(axes, [0, 300]):
    for side in ["left", "right"]:
        key = f"turn_{mass}_{side}"
        d = np.loadtxt(o / key / "propulsion_state.csv", delimiter=",")
        st = json.loads((o / key / "stages.json").read_text())[1]
        v = d[(d[:, 0] >= st["start_sim"]) & (d[:, 0] <= st["end_sim"])]
        yaw = v[0, 14]
        xy = v[:, 12:14] - v[0, 12:14]
        xy = xy @ np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        axis.plot(xy[:, 0], xy[:, 1], label=side)
    axis.set_title(f"Payload {mass} kg, command 55%, steering ±20°")
    axis.set_xlabel("Initial forward axis (m)")
    axis.set_ylabel("Initial port axis (m)")
    axis.axis("equal")
    axis.grid(alpha=0.3)
    axis.legend()
fig.savefig(o / "turning_paths.png", dpi=160)
plt.close(fig)
fig, axes = plt.subplots(2, 1, figsize=(9, 6), constrained_layout=True)
for mass in [0, 300]:
    key = f"brake_{mass}"
    d = np.loadtxt(o / key / "propulsion_state.csv", delimiter=",")
    st = json.loads((o / key / "stages.json").read_text())
    axes[0].plot(
        d[:, 0] - st[1]["start_sim"], d[:, 10] * 3.6, label=f"{mass} kg payload"
    )
    key = f"curve_{mass}_100"
    d = np.loadtxt(o / key / "propulsion_state.csv", delimiter=",")
    st = json.loads((o / key / "stages.json").read_text())
    v = d[d[:, 0] >= st[1]["start_sim"]]
    axes[1].plot(
        v[:, 0] - st[1]["start_sim"], v[:, 10] * 3.6, label=f"{mass} kg payload"
    )
axes[0].set_xlim(-5, 40)
axes[0].set_title("Reverse braking: full command to -60%")
axes[1].set_title("Unpowered coast after full command")
for axis in axes:
    axis.axhline(0, color="gray", lw=0.6)
    axis.set_xlabel("Time from command change (s)")
    axis.set_ylabel("Forward speed (km/h)")
    axis.grid(alpha=0.3)
    axis.legend()
fig.savefig(o / "braking.png", dpi=160)
plt.close(fig)
e = np.loadtxt(o / "endurance_10min/energy_truth.csv", delimiter=",")
t = e[:, 0] - r["endurance_10min"]["start_sim"]
fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True, constrained_layout=True)
axes[0].plot(t, e[:, 7] / 1000, label="Motor")
axes[0].plot(t, e[:, 10] / 1000, label="Generator AC")
axes[0].set_ylabel("Power (kW)")
axes[0].legend()
axes[1].plot(t, e[:, 1] * 100)
axes[1].set_ylabel("Battery SOC (%)")
axes[2].plot(t, e[:, 5])
axes[2].set_ylabel("Fuel (L)")
axes[2].ticklabel_format(useOffset=False)
axes[2].set_xlabel("Simulation time from ready (s)")
for axis in axes:
    axis.grid(alpha=0.3)
fig.savefig(o / "endurance.png", dpi=160)
plt.close(fig)
# Independent evaluations, deliberately separate from implementation/finite-state checks.
neutral = r["neutral_rudder"]["metrics"]["neutral"]
nd = np.loadtxt(o / "neutral_rudder/actuators_state.csv", delimiter=",")
ns = r["neutral_rudder"]["start_sim"]
late = nd[(nd[:, 0] > ns + 35) & (nd[:, 0] < ns + 44)]
assessment = {
    "all_runtime_checks_passed": all(v["passed"] for v in r.values()),
    "launcher_failure_cleanup_passed": launcher["passed"],
    "suite_cancellation_cleanup_passed": cancel["passed"],
    "abrupt_child_cleanup_passed": abrupt["passed"],
    "energy_2hour_passed": energy["passed"],
    "clock_failure_reproduced_before": not before["passed"],
    "clock_independent_after": r["clock_independent"]["passed"],
    "passive_heading_stable": abs(neutral["heading_change_rad"]) < 0.1,
    "neutral_heading_change_deg": float(np.rad2deg(neutral["heading_change_rad"])),
    "neutral_final_window_max_servo_error_deg": float(
        np.rad2deg(abs(late[:, 1]).max())
    ),
    "monotonic_speed_with_command": all(
        all(
            r[f"curve_{mass}_{a}"]["metrics"]["fixed"]["tail_speed_m_s"]
            < r[f"curve_{mass}_{b}"]["metrics"]["fixed"]["tail_speed_m_s"]
            for a, b in [(25, 50), (50, 75), (75, 100)]
        )
        for mass in [0, 150, 300]
    ),
    "full_command_speed_decreases_with_payload": all(
        r[f"curve_{a}_100"]["metrics"]["fixed"]["tail_speed_m_s"]
        > r[f"curve_{b}_100"]["metrics"]["fixed"]["tail_speed_m_s"]
        for a, b in [(0, 150), (150, 300)]
    ),
}
(o / "assessment.json").write_text(json.dumps(assessment, indent=2))
(o / "summary.json").write_text(
    json.dumps(
        {
            "passed": assessment["all_runtime_checks_passed"],
            "overall_physical_validation_passed": assessment[
                "all_runtime_checks_passed"
            ]
            and assessment["passive_heading_stable"],
            "scope": "Runtime/energy/command checks; passive heading stability remains a failed separate physical assessment",
            "assessment": assessment,
            "cases": r,
        },
        indent=2,
    )
)
rel = "../../artifacts/validation/" + o.name + "/"
md = [
    "# 2026-11-21 阻力、操纵与启动可靠性改进",
    "",
    "## 结论与未解决项",
    "",
    f"完成 {len(r)} 个本轮原生 Gazebo/ROS 工况，固定油门直航、回转、倒车制动、连续转向、10 分钟风浪流航行及保护测试；运行/数值检查{'全部通过' if assessment['all_runtime_checks_passed'] else '仍有失败项'}。另完成同一 C++ 能源模型的 2 小时计算。本轮不运行或扩展 ArduPilot。",
    "",
    "**仍未解决：高速零舵的被动航向稳定性。** 零舵试验的舵角跟踪通过，但船仍持续转弯。因此不能把所有运行检查通过解释成“船体物理模型已经全部验收”。当前正常直航应使用航向保持，后续需改进侧向/偏航水动力、尾部附体作用及系数辨识；不能靠提高航向控制增益或修改通过阈值掩盖这一问题。",
    "",
    "## 已修改的部分",
    "",
    "1. 控制器新增 `/usv/control/health`，待机也持续发布能源、推进反馈、舵机及导航数据的新鲜度、缺失项、通信重试次数和通信分区。启动入口等待连续就绪且物理时间确实推进，未就绪就超时退出。",
    "2. 控制调度/健康检查改为 `STEADY_TIME` 定时器，控制积分和动态计算仍使用原生能源状态中的物理时间。避免 ROS `/clock` 未桥接或暂停时连看门狗都不执行。",
    "3. 必需的 ROS 控制器或桥接节点退出时，launch 自动关闭其余接口，平台清理 Gazebo。每次性能试验采用唯一 Gazebo 分区，验证健康消息来源，记录初始 SOC，并在退出时清理分区内残留进程。",
    "4. 阻力增加实际浸水几何依赖：线性和转动阻尼按浸水面积比例缩放；平动二次阻尼按对应方向的浸水表面法向投影面积比例缩放。名义参考固定，绝不为每个载荷重新归一化。",
    "5. 舵机增加 `Ki=100 Nm/(rad·s)` 的积分力矩，受原有力矩、速率和功率限制，带饱和防积分累积与断电/复位清零。修正比例/微分控制在持续负载下留下的角度误差；这不是厂家标定参数。",
    "6. 新增 `/usv/actuators/heading_hold_throttle`（ROS Float64，−1..1）：保持给定桨轴转速请求，只调舵保持航向，不闭环调航速。固定油门曲线采用此模式。原直接油门/舵角和 `cmd_vel` 接口仍保留。",
    "",
    "## 阻力依据和适用边界",
    "",
    "参考面积由现有封闭船体几何在名义满油静水平衡位置裁切得到：浸水面积约 10.3363 m²，X/Y/Z 方向半法向面积积分约 [0.36034, 1.24373, 4.49332] m²。参数在 `resistance.json`，完整来源和几何哈希随记录保存。",
    "",
    "在固定系数近似下，二次阻力随参考面积变化；[ITTC 阻力试验规程](https://ittc.info/media/1816/75-02-02-01.pdf) 也用浸水面积进行阻力系数化。这里的分方向投影和线性/转动面积缩放是额外的工程假设，不能称为 ITTC 认证模型。没有额外叠加一套摩擦阻力，以免与原有总阻力预算重复计算。",
    "",
    "阻力形式为 `F_i = −D_i·(S/S_ref)·v_i −Q_i·(A_i/A_i_ref)·v_i·|v_i|`，速度相对于当地水流。系数非负，阻尼相对于水的耗散功率非负，代码有对应解析单元测试和运行时记录。波浪/流仍可向船传递能量，这不等于整个船的世界坐标机械能只会下降。仍缺少实测/CFD阻力、频率相关附加质量、高速兴波、滑行、砰击和空化标定。",
    "",
    "## 固定油门直航曲线",
    "",
    "满油 60 L，附加载荷重心 [0,0,0.6] m，静水；各点给定固定油门 45 秒，取末 8 秒均值，再零油门滑行 35 秒。油门是归一化桨轴转速请求，**不是功率百分比**。航速为船体纵向对地速度，直航只通过舵控制航向。",
    "",
    "| 附加载荷 kg | 25% km/h | 50% km/h | 75% km/h | 100% km/h | 满油门电功率 kW |",
    "|---:|---:|---:|---:|---:|---:|",
]
for mass in [0, 150, 300]:
    vals = [r[f"curve_{mass}_{x}"]["metrics"]["fixed"] for x in [25, 50, 75, 100]]
    md.append(
        f"| {mass} | "
        + " | ".join(f"{v['tail_speed_m_s']*3.6:.3f}" for v in vals)
        + f" | {vals[-1]['mean_motor_w']/1000:.3f} |"
    )
md += [
    "",
    f"![固定油门曲线]({rel}throttle_curves.png)",
    "",
    "无附加载荷的满油整船约 1539 kg；载荷越大，浸水面积/投影面积和阻力随之变化。没有为了让全部工况达到 15 km/h 而增加功率或修改阻力系数。以上是 40 kW、六电池概念配置的仿真结果，不能拿来宣称 6 kW 基础配置也具有同样性能。",
    "",
    "| 阻尼敏感性 | 满油门航速 km/h |",
    "|---|---:|",
]
for name in ["drag_low", "curve_0_100", "drag_high"]:
    md.append(f"| {name} | {r[name]['metrics']['fixed']['tail_speed_m_s']*3.6:.3f} |")
md += [
    "",
    "drag_low / drag_high 为原阻尼系数的 0.75 / 1.25 倍；这是输入敏感性，不是概率置信区间。表格保留的小数位用于数值复现，不代表实船预测精度。",
    "",
    "## 滑行、倒车制动与转弯",
    "",
    "| 载荷 kg | 满油门后滑行至 |u|<0.1 m/s 时间/距离 | −60%反推制动时间/距离 | 左/右回转半径 m |",
    "|---:|---|---|---|",
]
for mass in [0, 300]:
    coast = r[f"curve_{mass}_100"]["metrics"]["coast"]
    brake = r[f"brake_{mass}"]["metrics"]["reverse_brake"]
    turns = [
        r[f"turn_{mass}_{side}"]["metrics"]["turn"]["tail_turn_radius_m"]
        for side in ["left", "right"]
    ]

    def stop_text(m):
        """把停止时间和距离格式化为报告文本。

        Args:
            m: 包含停止指标的工况结果映射
        """
        return (
            f"{m['time_to_abs_speed_0_1_s']:.2f} s / {m['distance_to_abs_speed_0_1_m']:.2f} m"
            if m.get("time_to_abs_speed_0_1_s") is not None
            else "观察窗口内未达到阈值"
        )

    md.append(
        f"| {mass} | {stop_text(coast)} | {stop_text(brake)} | {turns[0]:.2f} / {turns[1]:.2f} |"
    )
md += [
    "",
    "回转采用 55%固定油门、±20°舵角，取末 8 秒实际轨迹长度/航向变化量计算船体 link 原点的局部回转半径，不是重心半径或标准战术回转直径；因侧向/偏航模型尚未验收，不能当成实船最小转弯半径。另测 0/300 kg 载荷下 ±15°交替转向、持续倒车和松油门。这里的制动阈值是船体纵向速度，不是“绝对静止”；风浪流中的漂移也不应按静水停船理解。",
    "",
    f"![左右回转]({rel}turning_paths.png)",
    "",
    f"![制动与滑行]({rel}braking.png)",
    "",
    "## 零舵诊断",
    "",
    f"无附加载荷、100%油门、零舵指令的 45 秒诊断中，后段最大实际舵角偏差约 {assessment['neutral_final_window_max_servo_error_deg']:.4f}°，但累计航向变化约 {assessment['neutral_heading_change_deg']:.1f}°。**舵机跟踪改善没有消除被动航向不稳定。** 此试验用于揭示侧向/偏航模型的不足，不作为直航极速数据，也不作为已解决项。",
    "",
    "先前仅有比例/微分项时，同类零舵诊断约有 2°持续角度偏差；原始曲线在 attempts_before_heading_and_servo 中保留。禁止把零舵转圈时的船体纵向速度当作满油门直航航速。",
    "",
    "## 长时间油电与故障保护",
    "",
    f"10 分钟实际 Gazebo 航行：150 kg载荷、30 L油、wave gain 0.3、世界坐标风 [0,6,0] m/s、流 [0.15,0.1,0] m/s，指令 2 m/s、偏航角速度 0.06 rad/s，辅助用电采用采集工况。发电机在 60–220 s、300–500 s 两次启用，实际经历启动、暖机和运行。结果：SOC {r['endurance_10min']['summary']['soc_start_end'][0]:.5f} → {r['endurance_10min']['summary']['soc_start_end'][1]:.5f}，耗油 {r['endurance_10min']['summary']['fuel_consumed_l']:.6f} L，电池最高温度 {r['endurance_10min']['summary']['temperature_max_c']:.3f}°C。",
    "",
    f"![10分钟油电联动]({rel}endurance.png)",
    "",
    "- 暂停所属 ROS 传感器桥接后，导航过期会停推；恢复桥接并继续发送新指令后恢复航行。",
    "- 初始 SOC=0.155 的 180秒工况检查低电量降功率；初始电池44.8°C、环境40°C的180秒工况检查温度降功率。这些是明确注入的边界条件，不是假称自然工况一定会过热。",
    "- 初始燃油1.005 L、强制发电机启用的120秒工况检查拾油储备1 L处停机/故障；电池供电仍可继续推进，燃油耗尽不等于动力电池耗尽。",
    "",
    "另运行相同 C++ 能源模型 2 小时：固定请求40 kW、6电池、采集辅助负载、8 W舵机代表值。它不计算长时间6自由度运动，不可当作真实续航/航程预测。",
    "",
    "| 发电机 | 首次无法满足40 kW请求 min | 电池最高温度 °C | 2h耗油 L |",
    "|---|---:|---:|---:|",
]
for key, label in [("0", "关闭"), ("1", "持续开启")]:
    v = energy["cases"][key]
    md.append(
        f"| {label} | {v['first_power_shortfall_min']:.2f} | {v['max_temperature_c']:.3f} | {v['fuel_consumed_l']:.4f} |"
    )
md += [
    "",
    "上述温度模型只覆盖现有集中参数电池模型；电机、驱动器、线缆温升及电芯不均衡尚未建模。25 A充电器限制实际补能，不能把4 kW发电机铭牌功率当作持续40 kW推进供电。",
    "",
    "## 启动问题的复现、修复和证据",
    "",
    "- 关闭时钟桥接：旧版无健康状态、未开始试验；改为独立调度后，相同条件可进入就绪并行驶。见 attempts_clock_before 与 clock_independent。",
    "- 某次原生 fuel_mass 状态订阅未发现发布端：首轮拒绝开始，保存缺失流；新增有界重订阅后仍要求实际数据到达，未用占位值代替。",
    "- 中止重试后发现同名分区残留实例，出现启动时物理时间已达约205秒、SOC非初值，导致混合数据。此轮数据废弃留档，改用每次唯一分区、健康消息分区匹配、初始SOC核对、完整退出清理后重跑。",
    "- 调度器收到中止信号、测试子进程被强制杀死的独立试验均确认无残留分区进程；后者由父调度器兜底清理，不能仅依赖子进程 finally。",
    "- 控制器强制退出试验：整个启动栈自动关闭，没有遗留子进程。无控制器的就绪检查按时超时拒绝，不会显示“可开船”。",
    "- 故障注入脚本曾向 ROS double 字段传入整数而触发原生类型断言。已在构造消息时显式转换 float，保留失败记录并重做该工况；增加异常退出兜底清理。此问题属于测试工具，不是船体物理故障。",
    "- 部分早期结果导出遇到 numpy.bool_ 无法JSON编码：原始CSV、阶段时间和就绪记录已保存。只重做后处理，并以 postprocess_recovered 标记；没有修改原始运动数据。",
    "",
    "## 复现、接口与文件",
    "",
    "在 ros_ws 目录，先 source /opt/ros/jazzy/setup.bash 和 install/setup.bash。正常开船仍用 `bash ../ros_ws/src/usv_bringup/scripts/run_platform.sh`；看到 Boat ready 后再发送控制指令。健康状态可用 `ros2 topic echo /usv/control/health` 查看。固定油门航向保持可持续发布 Float64 至 `/usv/actuators/heading_hold_throttle`，停止发布后看门狗停推；不得同时混用直接舵角和该模式。",
    "",
    "```bash",
    f"python3 ../tests/regression/run_performance_suite.py --spec ../artifacts/validation/{o.name}/spec.json --output ../artifacts/validation/{o.name}",
    f"python3 ../tools/analysis/report_vessel_performance.py --output ../artifacts/validation/{o.name}",
    "```",
    "",
    "调度器跳过已通过的结果。完整重跑请使用新的输出目录并复制spec.json，避免覆盖历史证据。每个工况保存模型/世界快照、原始CSV、健康日志、就绪状态、阶段定义、来源哈希及结果。CSV无表头，字段见 schemas.json。读取已有CSV的独立后处理入口为 analyze_performance_run.py。",
    "",
    "10项C++物理/能源/舵机测试、3项就绪窗口测试通过；实际运行证据见 summary.json、assessment.json、launcher_controller_exit.json 和 energy_2hour/result.json。",
    "",
    "## 后续优先项",
    "",
    "优先完善高速侧向/偏航水动力和艉部附体对方向稳定性的作用，做虚拟斜航/回转辨识与步长敏感性对照，再评估是否需要修改模型或实际几何。当前没有实船数据，不应直接给阻尼“调到看起来稳定”就当成真实。随后再扩展更长时间、更大风浪和设备热模型。继续暂缓 ArduPilot。",
]
# Avoid unescaped vertical bars inside a table cell.
md = [line.replace("滑行至 |u|<0.1", "滑行至纵向速度绝对值<0.1") for line in md]
(w / "../docs/history/2026-11-21_阻力操纵与启动可靠性改进.md").write_text(
    "\n".join(md) + "\n"
)
files = (
    [
        tool_path(name)
        for name in [
            "validate_vessel_performance.py",
            "performance_analysis.py",
            "run_performance_suite.py",
            "report_vessel_performance.py",
            "readiness.py",
            "wait_control_ready.py",
            "run_platform.sh",
            "stop_test_partition.py",
        ]
    ]
    + list((w / "src/usv_description/config").glob("*.json"))
    + [
        w / "src/usv_control/scripts/boat_control.py",
        w / "src/usv_control/launch/interfaces.launch.py",
        w / "src/usv_simulation/src/HullHydrostatics.cc",
        w / "src/usv_simulation/src/AzimuthPropulsion.cc",
        w / "src/usv_simulation/include/Resistance.hh",
        w / "src/usv_simulation/include/SteeringIntegral.hh",
    ]
)
(o / "source_hashes.json").write_text(
    json.dumps(
        {
            str(f.resolve().relative_to(PROJECT_ROOT)): hashlib.sha256(
                f.read_bytes()
            ).hexdigest()
            for f in files
        },
        indent=2,
    )
)
print(json.dumps(assessment, indent=2))
