#!/usr/bin/env python3
"""整理波浪响应试验的图表与判据。

Plot measured Gazebo wave response; no synthetic motion samples.
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
import os

os.environ.setdefault("MPLCONFIGDIR", "/tmp/usv_matplotlib")
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

w = WORKSPACE
out = w / "../artifacts/validation/船体三项优化_2026-11-20"
cases = ["calm", "longitudinal", "beam", "quarter", "strong"]
results = {}
fig, axes = plt.subplots(5, 2, figsize=(12, 13), sharex=True)
for i, c in enumerate(cases):
    folder = out / "waves" / c
    r = json.loads((folder / "result.json").read_text())
    results[c] = r
    d = np.loadtxt(folder / "wave_response.csv", delimiter=",", skiprows=1)
    tail = d[d[:, 0] > 25]
    z0 = np.mean(tail[:, 6])
    ax = axes[i, 0]
    ax.plot(d[:, 0], d[:, 1], label="Local water elevation", lw=1)
    ax.plot(d[:, 0], d[:, 6] - z0, label="Hull heave (mean removed)", lw=1)
    ax.set_ylabel(c + "\nHeight [m]")
    ax.grid(alpha=0.3)
    ax = axes[i, 1]
    ax.plot(d[:, 0], np.rad2deg(d[:, 7]), label="Roll", lw=1)
    ax.plot(d[:, 0], np.rad2deg(d[:, 8]), label="Pitch", lw=1)
    ax.set_ylabel("Attitude [deg]")
    ax.grid(alpha=0.3)
axes[0, 0].legend()
axes[0, 1].legend()
axes[-1, 0].set_xlabel("Simulation time [s]")
axes[-1, 1].set_xlabel("Simulation time [s]")
fig.suptitle("Measured Gazebo response — zero throttle, 5 s VRX PMS spectrum")
fig.tight_layout()
fig.savefig(out / "wave_response.png", dpi=150)
plt.close(fig)
ratio = (
    results["strong"]["metrics"]["heave_peak_to_peak_m"]
    / results["quarter"]["metrics"]["heave_peak_to_peak_m"]
)
checks = {
    "all_cases_passed": all(r["passed"] for r in results.values()),
    "stronger_wave_increases_heave": 1.3 < ratio < 3,
    "beam_roll_exceeds_longitudinal_roll": results["beam"]["metrics"][
        "roll_peak_to_peak_deg"
    ]
    > results["longitudinal"]["metrics"]["roll_peak_to_peak_deg"],
}
summary = {
    "passed": all(checks.values()),
    "checks": checks,
    "strong_to_quarter_heave_ratio": ratio,
    "cases": results,
}
(out / "wave_summary.json").write_text(json.dumps(summary, indent=2))
print(json.dumps({k: v for k, v in summary.items() if k != "cases"}, indent=2))
raise SystemExit(0 if summary["passed"] else 1)
