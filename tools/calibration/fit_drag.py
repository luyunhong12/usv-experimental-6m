#!/usr/bin/env python3
"""从明确标记的试验数据拟合非负阻力系数，不自动修改模型。

Fit nonnegative straight-line drag coefficients from explicitly labelled data.

This is an identification aid, not an automatic change of physical parameters.
CSV: time_s,relative_speed_m_s,thrust_n,steering_rad,yaw_rate_rad_s
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
import argparse
import json
import numpy as np
from scipy.optimize import nnls
from scipy.signal import savgol_filter

p = argparse.ArgumentParser()
p.add_argument("csv")
p.add_argument("--mass-kg", type=float, required=True)
p.add_argument("--data-kind", choices=["measured", "simulation"], required=True)
p.add_argument("--output", required=True)
a = p.parse_args()
d = np.genfromtxt(a.csv, delimiter=",", names=True)
if a.mass_kg <= 0 or len(d) < 21:
    raise ValueError("Positive mass and at least 21 samples required")
t = d["time_s"]
u = d["relative_speed_m_s"]
thrust = d["thrust_n"]
if not all(np.isfinite(d[name]).all() for name in d.dtype.names):
    raise ValueError("Nonfinite data")
dt = np.diff(t)
if np.any(dt <= 0) or np.std(dt) / np.mean(dt) > 0.02:
    raise ValueError("Resample to uniform positive time spacing first")
if (
    np.max(abs(d["steering_rad"])) > np.deg2rad(1)
    or np.max(abs(d["yaw_rate_rad_s"])) > 0.01
):
    raise ValueError("Only calm, approximately straight runs supported")
accel = savgol_filter(u, 11, 3, deriv=1, delta=float(np.mean(dt)))
X = np.column_stack([u, u * abs(u)])[5:-5]
y = (thrust - a.mass_kg * accel)[5:-5]
if np.linalg.cond(X) > 1e4:
    raise ValueError(
        "Insufficient speed variation to identify linear and quadratic drag"
    )
coeff, _ = nnls(X, y)
residual = y - X @ coeff
report = {
    "data_kind": a.data_kind,
    "mass_kg": a.mass_kg,
    "linear_drag_n_s_m": float(coeff[0]),
    "quadratic_drag_n_s2_m2": float(coeff[1]),
    "rmse_n": float(np.sqrt(np.mean(residual**2))),
    "samples": len(y),
    "limitations": [
        "Known external thrust required; unknown motor thrust and drag cannot both be identified from speed alone",
        "Mass excludes unmodelled added mass; prefer steady runs for physical calibration",
        "No automatic update of production model",
    ],
}
with open(a.output, "w") as f:
    json.dump(report, f, indent=2)
print(json.dumps(report, indent=2))
