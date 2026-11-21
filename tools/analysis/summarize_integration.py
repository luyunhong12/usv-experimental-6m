#!/usr/bin/env python3
"""汇总整船集成试验的证据。

Archive successful upgrade validations, schemas, configuration and hashes.
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
from project_paths import PROJECT_ROOT, WORKSPACE
import hashlib
import json
import shutil
import sys
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from energy_fields import TRUTH, MASS

w = WORKSPACE
src = w / "log/integration_upgrade"
dest = w / "../artifacts/validation/六项整改验证"
dest.mkdir(parents=True, exist_ok=True)
cases = [
    "full_speed",
    "speed15",
    "heading",
    "sensors",
    "waves",
    "current",
    "wind",
    "power_loss",
    "multi",
    "collision",
    "task",
    "ardupilot",
    "algorithm_checks",
]
results = {}
for name in cases:
    r = json.loads((src / name / "result.json").read_text())
    if not r.get("passed"):
        raise RuntimeError("Failed validation: " + name)
    results[name] = r
    shutil.copytree(src / name, dest / name, dirs_exist_ok=True)
for name in ["burn", "reset"]:
    folder = w / "log/fuel_mass_validation" / name
    r = json.loads((folder / "result.json").read_text())
    if not r.get("passed"):
        raise RuntimeError("Failed fuel validation: " + name)
    results["fuel_" + name] = r
    shutil.copytree(folder, dest / ("fuel_" + name), dirs_exist_ok=True)
for name in [
    "unit_tests.log",
    "build.log",
    "control_build.log",
    "pymavlink_install.log",
]:
    if (src / name).exists():
        shutil.copy2(src / name, dest / name)
for name in ["usv_description", "usv_energy", "usv_ardupilot"]:
    shutil.copytree(
        w / "src" / name / "config", dest / "configuration" / name, dirs_exist_ok=True
    )
(dest / "summary.json").write_text(json.dumps(results, indent=2, ensure_ascii=False))
prop = "time,command,actual_rpm,reference_rpm,thrust_N,motor_torque_Nm,water_torque_Nm,shaft_power_W,wet_fraction,axial_velocity,body_u,body_v,world_x,world_y,yaw,roll,pitch".split(
    ","
)
schemas = {
    "propulsion_state.csv": prop,
    "escort.csv": prop,
    "energy_truth.csv": TRUTH,
    "fuel_mass_state.csv": MASS,
    "hydrostatics.csv": "time,volume,cbx,cby,cbz,buoyancy,x,y,z,roll,pitch,yaw".split(
        ","
    ),
    "actuators_state.csv": "time,actual_angle,actual_rate,reference_angle,torque_Nm,servo_W,servo_Wh,payload_powered".split(
        ","
    ),
    "environment_state.csv": "time,wave_height,wave_dx,wave_dy,water_vx,water_vy,water_vz,air_vx,air_vy,air_vz,body_drag_x,body_drag_y,body_drag_z,body_air_x,body_air_y,body_air_z".split(
        ","
    ),
    "controller.csv": "time,desired_speed,measured_speed,heading_error,yaw_rate,throttle,angle,saturated".split(
        ","
    ),
}
(dest / "csv_schemas.json").write_text(json.dumps(schemas, indent=2))
fig, axes = plt.subplots(2, 2, figsize=(11, 7), layout="constrained")
for name, label in [
    ("full_speed", "6 kW baseline (full throttle)"),
    ("speed15", "40 kW concept (15 km/h control)"),
]:
    d = np.loadtxt(dest / name / "propulsion_state.csv", delimiter=",")
    e = np.loadtxt(dest / name / "energy_truth.csv", delimiter=",")
    axes[0, 0].plot(d[:, 0], d[:, 10] * 3.6, label=label)
    axes[0, 1].plot(e[:, 0], e[:, 7] / 1000, label=label)
axes[0, 0].axhline(15, color="gray", ls="--")
axes[0, 0].set(ylabel="Body forward speed (km/h)", xlabel="Simulation time (s)")
axes[0, 0].legend(fontsize=8)
axes[0, 1].set(ylabel="Motor electrical power (kW)", xlabel="Simulation time (s)")
h = np.loadtxt(dest / "heading/propulsion_state.csv", delimiter=",")
axes[1, 0].plot(h[:, 0], np.rad2deg(h[:, 14]))
axes[1, 0].set(
    ylabel="Heading (degrees)",
    xlabel="Simulation time (s)",
    title="Baseline IMU/GPS heading hold, target 1 m/s",
)
e = np.loadtxt(dest / "waves/environment_state.csv", delimiter=",")
h = np.loadtxt(dest / "waves/hydrostatics.csv", delimiter=",")
axes[1, 1].plot(e[:, 0], e[:, 1], label="Local surface")
axes[1, 1].plot(h[:, 0], h[:, 8] - np.mean(h[:, 8]), label="Hull heave (mean removed)")
axes[1, 1].set(
    ylabel="Vertical displacement (m)",
    xlabel="Simulation time (s)",
    title="VRX wave gain 0.3",
)
axes[1, 1].legend(fontsize=8)
for ax in axes.flat:
    ax.grid(alpha=0.25)
fig.suptitle("Simulation evidence — uncalibrated engineering scenarios")
fig.savefig(dest / "validation_overview.png", dpi=160)
plt.close(fig)
(dest / "README.md").write_text(
    """# 六项整改验证归档

全部是仿真或合成算法数据，不是实船测量。summary.json 汇总判据；各目录含 result.json、原始 CSV、启动日志及模型快照。CSV 大多无表头，字段见 csv_schemas.json，SI 单位，角度 rad；energy 的 Ah/Wh/L 字段按名称解释。fuel_burn/fuel_reset 的 CSV 自带表头；ardupilot/propulsion.csv 与 propulsion_state 同序。空 CSV 表示该工况不采集该信号，不代表零测量值。

validation_overview.png 展示速度、功率、闭环航向与波浪响应。曲线中的 40 kW 配置是假设升级，不能证明实船所需功率。baseline 是满油门；concept 是闭环 15 km/h 请求，不能把两者都称作极限速度测量。

运行快照保留原始绝对路径用于溯源；迁移后用 ../tools/setup/build.sh / prepare_sydney.py 重建，不直接运行归档世界。configuration 是参数快照；source_sha256.json 记录本轮自定义源码与脚本，data_sha256.json 记录归档文件。上游 VRX 来源见上级 vrx_source_manifest.json。前期旧数据/失败探索未混入通过结果。

CTest 日志和断电、风、浪、流、碰撞、多船、ROS 传感器/TF、飞控、燃油回归均保留。部分较早 launch 日志在测试主动 SIGINT 结束时有上游 monitor_sim KeyboardInterrupt；通过判据取停止前完整采样，并非启动/运行失败。当前 ROS 控制器已处理重复 SIGINT 清理。

复现归档：验证通过后运行 python3 ../tools/analysis/summarize_integration.py。它会拒绝任何 passed=false 的当前结果。详细步骤与真实性边界见 ../../过程记录/2026-11-19_六项整改与WAMV接口及15公里时速方案.md。
"""
)
files = []
for folder in [PROJECT_ROOT / "tools", *w.glob("src/usv_*")]:
    for f in folder.rglob("*"):
        if (
            f.is_file()
            and f.suffix
            in {".py", ".cc", ".hh", ".json", ".xml", ".txt", ".sh", ".md", ".parm"}
            and "__pycache__" not in f.parts
        ):
            files.append(f)
(dest / "source_sha256.json").write_text(
    json.dumps(
        {
            str(f.resolve().relative_to(PROJECT_ROOT)): hashlib.sha256(
                f.read_bytes()
            ).hexdigest()
            for f in sorted(files)
        },
        indent=2,
        ensure_ascii=False,
    )
)
(dest / "data_sha256.json").write_text(
    json.dumps(
        {
            str(f.relative_to(dest)): hashlib.sha256(f.read_bytes()).hexdigest()
            for f in sorted(dest.rglob("*"))
            if f.is_file() and f.name != "data_sha256.json"
        },
        indent=2,
        ensure_ascii=False,
    )
)
print("Archived", len(results), "passed validation groups to", dest)
