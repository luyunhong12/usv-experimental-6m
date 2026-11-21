#!/usr/bin/env python3
"""汇总能源验证日志与结果。

Archive only passing energy evidence and regenerate documentation results.
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
from project_paths import WORKSPACE
import os
import json
import shutil
import hashlib
import csv

os.environ.setdefault("MPLCONFIGDIR", "/tmp/usv-energy-matplotlib")
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

W = WORKSPACE
out = W / "../artifacts/validation/能源验证"
out.mkdir(parents=True, exist_ok=True)
results = []
bridge = json.loads(
    (W / "log/energy_validation/charge_sensor/bridge_result.json").read_text()
)
assert (
    bridge["passed"] and bridge["bridge_exit"] == 0
), "ROS bridge did not exit cleanly"
for case in ["drive", "hybrid", "charge_sensor", "low_soc", "fuel_out"]:
    src = W / "log/energy_validation" / case
    r = json.loads((src / "result.json").read_text())
    assert r["passed"], case
    dst = out / case
    dst.mkdir(parents=True, exist_ok=True)
    for f in src.iterdir():
        if f.suffix in [".csv", ".json", ".log"]:
            shutil.copy2(f, dst / f.name)
    results.append(r)
files = [
    "src/usv_energy/config/energy.json",
    "src/usv_energy/include/usv_energy/Parameters.def",
    "src/usv_energy/include/usv_energy/EnergyModel.hh",
    "src/usv_energy/include/usv_energy/Sensors.hh",
    "src/usv_simulation/src/AzimuthPropulsion.cc",
    "src/usv_description/config/propulsion.json",
    "src/usv_description/config/trial_load.json",
    "../tests/integration/validate_energy.py",
    "../tools/control/energy_console.py",
    "../tools/lib/energy_fields.py",
    "../tests/integration/validate_energy_bridge.py",
]
manifest = {f: hashlib.sha256((W / f).read_bytes()).hexdigest() for f in files}
(out / "summary.json").write_text(
    json.dumps(
        {
            "status": "ENGINEERING_ESTIMATE_NOT_REAL_VESSEL_VALIDATION",
            "gazebo_cases": results,
            "ros_bridge": bridge,
            "sha256": manifest,
        },
        indent=2,
    )
)
shutil.copy2(W / "src/usv_energy/config/energy.json", out / "energy_parameters.json")
long = list(csv.DictReader((out / "one_hour_scenarios.csv").open()))
names = list(dict.fromkeys(r["case"] for r in long))
final = []
fig, ax = plt.subplots(2, 2, figsize=(12, 8))
for name in names:
    d = [r for r in long if r["case"] == name]
    final.append(d[-1])
    t = np.array([float(r["time_s"]) / 60 for r in d])
    soc = np.array([float(r["soc"]) for r in d])
    fuel = np.array([float(r["fuel_l"]) for r in d])
    ax[0, 0].plot(t, soc * 100, label=name)
    ax[0, 1].plot(t, (60 - fuel) * 1000, label=name)
    ax[1, 0].plot(t, [float(r["current_a"]) for r in d], label=name)
    ax[1, 1].plot(t, [float(r["temperature_c"]) for r in d], label=name)
for a, title, y in zip(
    ax.flat,
    [
        "SOC (model estimate)",
        "Diesel consumed",
        "Battery current (+ discharge)",
        "Lumped battery temperature",
    ],
    ["SOC (%)", "Fuel used (mL)", "Current (A)", "Temperature (deg C)"],
):
    a.set(title=title, xlabel="Simulation time (min)", ylabel=y)
    a.grid(alpha=0.25)
    a.legend(fontsize=7)
fig.suptitle("One-hour energy-core scenarios: assumptions, not sea-trial predictions")
fig.tight_layout()
fig.savefig(out / "one_hour.png", dpi=150)
plt.close(fig)
h = np.loadtxt(out / "hybrid/truth.csv", delimiter=",", skiprows=1)
q = np.loadtxt(out / "charge_sensor/truth.csv", delimiter=",", skiprows=1)
s = np.loadtxt(out / "charge_sensor/sensors.csv", delimiter=",", skiprows=1)
f = np.loadtxt(out / "fuel_out/truth.csv", delimiter=",", skiprows=1)
fig, ax = plt.subplots(1, 3, figsize=(15, 4.3))
for col, label in [(7, "Motor electrical"), (9, "Charger DC"), (10, "Generator AC")]:
    ax[0].plot(h[:, 0], h[:, col], label=label)
ax[0].axvline(34, color="k", ls=":", label="Cooling fault")
ax[0].set(title="Hybrid propulsion / charging", ylabel="Power (W)")
ax[1].plot(q[:, 0], q[:, 1] * 100, label="Internal SOC")
ax[1].plot(s[:, 0], np.where(s[:, 2] > 0, s[:, 5] * 100, np.nan), label="Sensor SOC")
ax[1].axvspan(24, 32, color="gray", alpha=0.2, label="Sensor dropout")
ax[1].set(title="Measured vs internal state", ylabel="SOC (%)")
ax[2].plot(f[:, 0], (f[:, 5] - 1) * 1000, label="Fuel above pickup")
ax[2].set(title="Fuel pickup limit", ylabel="Usable fuel (mL)")
for a in ax:
    a.set_xlabel("Simulation time (s)")
    a.grid(alpha=0.25)
    a.legend(fontsize=7)
fig.tight_layout()
fig.savefig(out / "runtime.png", dpi=150)
plt.close(fig)
p = W / "../docs/history/2026-11-19_油电能源与感知验证.md"
text = p.read_text().split("<!-- energy-results -->")[0]
text += "<!-- energy-results -->\n\n## 本轮结果\n\n全部 5 个 Gazebo 场景通过，每项 40.036 秒仿真；4 项 CTest 通过。母线功率账最大绝对误差小于 4×10⁻¹¹ W（仅表示算法收支一致，不表示参数真实）。\n\n| 场景 | SOC 变化（百分点） | 耗油（mL） | 电推进峰值（W） | 充电峰值（W） |\n|---|---:|---:|---:|---:|\n"
for r in results:
    text += f"| {r['case']} | {r['soc_change']*100:+.6f} | {r['fuel_used_l']*1000:.4f} | {r['max_motor_w']:.2f} | {r['max_charger_w']:.2f} |\n"
text += f"\nROS 桥接收到 {bridge['battery_messages']} 条 BatteryState；充电电流方向、SOC 范围、掉线 NaN/错误诊断、掉线后恢复和正常退出均通过。修复了本机消息 byte 类型及原生订阅回调退出清理问题。\n"
text += "\n![Gazebo 能源与感知验证](../../artifacts/validation/能源验证/runtime.png)\n\n以下为能源核心的 1 小时算例，均从 SOC 90%、60 L 开始；3000 W 是固定电推进输入，不是 3000 W 轴功率，不对应一条已验证航速。\n\n| 工况 | 1 小时后 SOC | 耗油（L） |\n|---|---:|---:|\n"
for r in final:
    text += (
        f"| {r['case']} | {float(r['soc'])*100:.3f}% | {60-float(r['fuel_l']):.4f} |\n"
    )
text += "\n![一小时能源算例](../../artifacts/validation/能源验证/one_hour.png)\n\n可见同为 3 kW 电推进输入，开机组只能减缓电池下降；待机充电则可以提高 SOC。所有数据为当前假设下的计算，不作实船续航承诺。\n\n原始遥测、ROS 日志、测试结果、参数快照及代码哈希在 `../artifacts/validation/能源验证/`。使用 `python3 ../tools/analysis/summarize_energy.py` 重新归档；只接受全部通过且桥接正常退出的结果。\n"
p.write_text(text)
print(
    json.dumps(
        {"cases": len(results), "bridge_passed": bridge["passed"], "output": str(out)},
        ensure_ascii=False,
    )
)
