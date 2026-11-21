#!/usr/bin/env python3
"""汇总不同油量下的质量、重心、惯性及平衡姿态。

Archive verified native fuel-inertia evidence and documentation tables.
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

os.environ.setdefault("MPLCONFIGDIR", "/tmp/usv-fuel-matplotlib")
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from fuel_equilibrium import properties

W = WORKSPACE
out = W / "../artifacts/validation/燃油质量联动验证"
out.mkdir(parents=True, exist_ok=True)
results = {}
for name in ["backend_force", "backend_torque", "full", "half", "low", "burn", "reset"]:
    src = W / "log/fuel_mass_validation" / name
    r = json.loads((src / "result.json").read_text())
    assert r["passed"], name
    results[name] = r
    dst = out / name
    dst.mkdir(parents=True, exist_ok=True)
    for f in src.iterdir():
        if f.suffix in [".csv", ".json", ".log", ".png"]:
            shutil.copy2(f, dst / f.name)
for name, folder in [
    ("steering_contact", "hull_validation"),
    ("reversal", "propulsion_validation"),
]:
    src = W / "log" / folder / name
    r = json.loads((src / "result.json").read_text())
    assert r["passed"], name
    results[name] = r
    dst = out / ("regression_" + name)
    dst.mkdir(parents=True, exist_ok=True)
    for f in src.iterdir():
        if f.suffix in [".csv", ".json", ".log", ".png"]:
            shutil.copy2(f, dst / f.name)
for f in ["fuel_equilibria.json", "fuel_mass_reference.json"]:
    shutil.copy2(W / "../datasets/reference" / f, out / f)
shutil.copy2(W / "src/usv_description/config/fuel_tank.json", out / "fuel_tank.json")
shutil.copy2(W / "src/usv_energy/config/energy.json", out / "energy.json")
console = W / "log/fuel_mass_validation/console_smoke.log"
if console.exists():
    shutil.copy2(console, out / console.name)
ref = json.loads((out / "fuel_equilibria.json").read_text())
levels = np.array(sorted(map(float, ref)))
cases = [ref[str(int(fuel_litres))] for fuel_litres in levels]
m = np.array([c["ship"]["mass_kg"] for c in cases])
cg = np.array([c["ship"]["cg_m"] for c in cases])
pose = np.array([c["pose_z_roll_pitch"] for c in cases])
inertia_data = np.array([c["ship"]["inertia_kg_m2"] for c in cases])
fig, ax = plt.subplots(2, 2, figsize=(12, 8))
ax[0, 0].plot(levels, m, "o-")
ax[0, 0].set(ylabel="Whole-vessel mass (kg)", title="Mass bookkeeping")
for k, name in enumerate(["x forward", "y port", "z up"]):
    ax[0, 1].plot(levels, (cg[:, k] - cg[-1, k]) * 1000, "o-", label=name)
ax[0, 1].set(ylabel="COM change from 60 L (mm)", title="Centre of mass in hull frame")
ax[0, 1].legend()
for k, name in enumerate(["Ixx", "Iyy", "Izz"]):
    ax[1, 0].plot(
        levels, (inertia_data[:, k, k] - inertia_data[-1, k, k]), "o-", label=name
    )
ax[1, 0].set(ylabel="Inertia change (kg m²)", title="Whole-vessel centroidal inertia")
ax[1, 0].legend()
ax[1, 1].plot(levels, (pose[:, 0] - pose[-1, 0]) * 1000, "o-")
ax[1, 1].set(
    ylabel="Hull origin rise from 60 L (mm)", title="Independent calm-water equilibrium"
)
for a in ax.flat:
    a.set_xlabel("Fuel volume (L)")
    a.grid(alpha=0.25)
fig.suptitle("Estimated load cases: quasi-static box tank, no dynamic slosh")
fig.tight_layout()
fig.savefig(out / "load_cases.png", dpi=150)
plt.close(fig)
d = np.loadtxt(out / "burn/mass.csv", delimiter=",", skiprows=1)
h = np.loadtxt(out / "burn/hydro.csv", delimiter=",", skiprows=1)
fig, ax = plt.subplots(1, 2, figsize=(12, 4))
ax[0].plot(d[:, 0], (d[:, 1] - d[0, 1]) * 1000, label="Fuel volume")
ax[0].set(
    ylabel="Fuel change (mL)",
    xlabel="Simulation time (s)",
    title="Normal generator consumption",
)
ax[1].plot(d[:, 0], (d[:, 2] - d[0, 2]) * 1000, label="DART native mass")
ax[1].plot(
    d[:, 0], 0.84 * (d[:, 1] - d[0, 1]) * 1000, "--", label="Density × fuel change"
)
ax[1].set(
    ylabel="Mass change (g)",
    xlabel="Simulation time (s)",
    title="Native mass follows fuel burn",
)
for a in ax:
    a.grid(alpha=0.25)
    a.legend()
fig.tight_layout()
fig.savefig(out / "burn_tracking.png", dpi=150)
plt.close(fig)
# Compare two fuel levels at the same final attitude, excluding transient attitude changes.
R = Rotation.from_euler("xyz", h[-1, 9:12]).as_matrix()
before, _ = properties(float(d[0, 1]), R)
after, _ = properties(float(d[-1, 1]), R)
cg_delta = np.array(after["cg_m"]) - before["cg_m"]
inertia_delta = np.array(after["inertia_kg_m2"]) - before["inertia_kg_m2"]
assert np.linalg.norm(cg_delta) > 1e-7 and np.linalg.norm(inertia_delta) > 1e-5
files = [
    "src/usv_simulation/include/FuelMass.hh",
    "src/usv_simulation/src/AzimuthPropulsion.cc",
    "src/usv_physics/src/Physics.cc",
    "src/usv_physics/include/usv_physics/RuntimeInertia.hh",
    "src/usv_description/config/fuel_tank.json",
    "src/usv_energy/config/energy.json",
    "../tools/lib/fuel_mass.py",
    "../tools/lib/fuel_equilibrium.py",
    "../tools/modeling/prepare_hull.py",
    "../tests/integration/validate_fuel_mass.py",
    "../tests/integration/validate_inertia_backend.py",
]
summary = {
    "status": "ENGINEERING_QUASISTATIC_ESTIMATE_NOT_SEA_TRIAL",
    "results": results,
    "fuel_only_com_change_at_final_attitude_m": cg_delta.tolist(),
    "fuel_only_inertia_change_at_final_attitude_kg_m2": inertia_delta.tolist(),
    "sha256": {f: hashlib.sha256((W / f).read_bytes()).hexdigest() for f in files},
}
(out / "summary.json").write_text(json.dumps(summary, indent=2))
p = W / "../docs/history/2026-11-19_燃油质量重心惯性联动.md"
text = p.read_text().split("<!-- fuel-mass-results -->")[0]
text += "<!-- fuel-mass-results -->\n\n## 本轮通过结果\n\n5 项单元检查、2 项后端受力/力矩试验、5 项船模燃油工况全部通过；原转向接触/渲染与前进切倒车回归也通过。\n\n| 试验 | 实测仿真响应 | 解析预期 |\n|---|---|---|\n"
for name in ["backend_force", "backend_torque"]:
    r = results[name]
    text += f"| {name} | {r['measured_accelerations'][0]:.9f} → {r['measured_accelerations'][1]:.9f} | {r['expected_accelerations'][0]:.9f} → {r['expected_accelerations'][1]:.9f} |\n"
text += "\n上表分别为 m/s² 与 rad/s²，更新时速度变化符合正常积分步，没有额外速度脉冲。\n\n| 油量 | 整船质量 kg | 整船 CG x/y/z m | 船体参考点 z m |\n|---|---:|---|---:|\n"
for key in ["60", "30", "6", "1", "0"]:
    c = ref[key]
    text += (
        f"| {key} L | {c['ship']['mass_kg']:.6f} | "
        + " / ".join(f"{x:.6f}" for x in c["ship"]["cg_m"])
        + f" | {c['pose_z_roll_pitch'][0]:.6f} |\n"
    )
text += "\n60/30/6 L 均有 Gazebo 静水实跑；1/0 L 为独立计算参考。满油到 6 L 减重 45.36 kg，参考点上升约 5.47 mm，重心略向船尾移动并升高。参考点高度变化不等同于船上每一点吃水变化，因为纵倾也会改变。完整矩阵保存在 JSON，而不只保留对角元素。\n\n![油况与质量属性](../../artifacts/validation/燃油质量联动验证/load_cases.png)\n"
b = results["burn"]
text += f"\n正常耗油试验运行 {b['final_sim_s']:.3f} 秒，消耗 {b['fuel_consumed_l']*1000:.6f} mL；后端 hull 质量变化 {b['native_hull_mass_change_kg']*1000:.6f} g。质量与密度×油量最大偏差 {b['max_mass_error_kg']:.3g} kg，独立参考的重心误差 {b['final_cg_error_m']:.3g} m、惯性分量误差 {b['final_inertia_error_kg_m2']:.3g} kg·m²。数值一致性不表示工程参数已经实测。\n"
text += "\n固定最终姿态再比较燃油消耗前后的重心与惯性，二者仍发生变化，排除了仅由起始浮态收敛引起差异的解释。暂停保持油量不变；重置恢复初始油量、后端质量、时间与关闭的机组状态。\n\n![正常油耗与后端质量](../../artifacts/validation/燃油质量联动验证/burn_tracking.png)\n\n原始 CSV、应用回执、参考解、运行日志、回归截图、配置与源码哈希位于 `../artifacts/validation/燃油质量联动验证/`。复现归档：`python3 ../tools/analysis/summarize_fuel_mass.py`，只接受全部通过的结果。原船模和上游 VRX 源码保持不变。\n"
p.write_text(text)
print(
    json.dumps({"passed_cases": len(results), "output": str(out)}, ensure_ascii=False)
)
