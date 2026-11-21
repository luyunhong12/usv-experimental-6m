#!/usr/bin/env python3
"""使用同一能源模型计算长时能源变化，不等于长时六自由度航行。

Long-duration energy-only integration using the SAME C++ model as Gazebo.

No claim of long-duration 6-DOF navigation, or of real battery endurance.
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
import json
import subprocess
import re
import argparse
from pathlib import Path
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("--output", type=Path)
parser.add_argument("--motor-w", type=float, default=40000.0)
parser.add_argument(
    "--power-mode", choices=["normal", "survey", "peak"], default="survey"
)
args = parser.parse_args()
if not 0 < args.motor_w <= 40000:
    parser.error("Motor demand must be in (0,40000] W")
w = WORKSPACE
out = args.output or w / "log/platform_validation/energy_duration"
out.mkdir(parents=True, exist_ok=True)
p = json.loads((w / "src/usv_energy/config/energy.json").read_text())["parameters"]
profile = json.loads(
    (w / "src/usv_description/config/profiles/speed15_concept.json").read_text()
)
f = profile["battery_pack_multiplier"]
for key in [
    "capacity_ah",
    "max_discharge_a",
    "thermal_capacity_j_k",
    "thermal_conductance_w_k",
]:
    p[key] *= f
p["resistance_ohm"] /= f
p["motor_limit_w"] = profile["motor_electrical_w"]


def _load_runtime_dependencies():
    """在当前路径和运行环境配置完成后加载依赖，保持原导入时机。"""
    from equipment_ledger import auxiliary_w

    return (auxiliary_w,)


(auxiliary_w,) = _load_runtime_dependencies()

p["aux_output_w"] = auxiliary_w(args.power_mode, "speed15_concept")
allowed = set(
    re.findall(
        r"P\((\w+),",
        (w / "src/usv_energy/include/usv_energy/Parameters.def").read_text(),
    )
)
assert set(p) <= allowed
assign = "\n".join(f"p.{key}={float(value)};" for key, value in p.items())
source = (
    """#include <usv_energy/EnergyModel.hh>
#include <iostream>
#include <iomanip>
int main(){usv::energy::Parameters p;
"""
    + assign
    + """
std::cout<<"generator,time_s,soc,temperature_c,motor_w,budget_w,voltage_v,current_a,fuel_l,balance_w\\n"<<std::setprecision(12);
for(int gen=0;gen<2;++gen){usv::energy::Model m(p);for(int i=0;i<=72000;++i){double dt=.1;m.Prepare(dt,gen,true);double servo=std::min(8.,m.ActuatorBudget(dt));double budget=m.MotorBudget(dt,servo);m.Step(dt,std::min(38739.,budget),servo);if(i%100==0)std::cout<<gen<<","<<i*dt<<","<<m.s.soc<<","<<m.s.temperature<<","<<m.s.motorW<<","<<budget<<","<<m.s.voltage<<","<<m.s.current<<","<<m.s.fuel<<","<<m.s.balanceW<<"\\n";}}
}
"""
)
source = source.replace("38739.", str(args.motor_w))
cpp = out / "energy_duration.cc"
cpp.write_text(source)
binary = out / "energy_duration"
subprocess.run(
    [
        "g++",
        "-std=c++17",
        "-O2",
        "-I" + str(w / "src/usv_energy/include"),
        str(cpp),
        "-o",
        str(binary),
    ],
    check=True,
)
with (out / "energy.csv").open("w") as fp:
    subprocess.run([str(binary)], stdout=fp, check=True)
d = np.genfromtxt(out / "energy.csv", delimiter=",", names=True)
r = {
    "scope": f"2-hour energy-only engineering simulation, constant requested motor input {args.motor_w} W; actual load reduces with protection. Not a vessel range prediction.",
    "passed": True,
    "cases": {},
}
for gen in [0, 1]:
    v = d[d["generator"] == gen]
    limited = v[v["motor_w"] < args.motor_w - 1]
    cut = v[v["motor_w"] < 1]
    checks = {
        "finite": all(np.isfinite(v[k]).all() for k in v.dtype.names),
        "balanced": bool(np.max(abs(v["balance_w"])) < 1e-6),
        "soc_bounds": bool(np.min(v["soc"]) >= 0.02 - 1e-9 and np.max(v["soc"]) <= 0.9),
        "power_bounds": bool(
            np.min(v["motor_w"]) >= 0 and np.max(v["motor_w"]) <= 40000
        ),
        "generator_consumes_fuel": (
            bool(v["fuel_l"][-1] < 60) if gen else bool(v["fuel_l"][-1] == 60)
        ),
    }
    r["cases"][str(gen)] = {
        "checks": checks,
        "first_power_shortfall_min": (
            float(limited["time_s"][0] / 60) if len(limited) else None
        ),
        "first_motor_zero_min": float(cut["time_s"][0] / 60) if len(cut) else None,
        "max_temperature_c": float(np.max(v["temperature_c"])),
        "final_soc": float(v["soc"][-1]),
        "fuel_consumed_l": float(60 - v["fuel_l"][-1]),
    }
    r["passed"] &= all(checks.values())
(out / "parameters.json").write_text(json.dumps(p, indent=2))
(out / "result.json").write_text(json.dumps(r, indent=2))
print(json.dumps(r, indent=2))
raise SystemExit(0 if r["passed"] else 1)
