"""分析设备质量不确定性对整船属性的影响。

Static equilibrium sensitivity; not sea-trial calibration or confidence bounds.
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
from project_paths import WORKSPACE, tool_path
import json
import importlib.util
import copy
import numpy as np
from scipy.optimize import root
from mass_properties import aggregate

W = WORKSPACE
spec = importlib.util.spec_from_file_location(
    "prepared_hull", tool_path("prepare_hull.py")
)
model = importlib.util.module_from_spec(spec)
spec.loader.exec_module(model)
report = json.loads(
    (W / "src/usv_description/config/engineering_mass_estimate.json").read_text()
)
scenarios = dict(report["sensitivity_cases"])
for liters in [6, 30, 60]:
    parts = copy.deepcopy(report["parts"])
    for p in parts:
        if p["name"] == "60 L diesel trial load":
            p["mass_kg"] = liters * 0.84
            p["cg_m"] = [0.08, 0, 0.23 + 0.125 * liters / 60]
            p["inertia_at_cg_kg_m2"] = np.diag(
                [
                    p["mass_kg"] * (0.4**2 + (0.25 * liters / 60) ** 2) / 12,
                    p["mass_kg"] * (0.6**2 + (0.25 * liters / 60) ** 2) / 12,
                    p["mass_kg"] * (0.6**2 + 0.4**2) / 12,
                ]
            ).tolist()
    scenarios[f"fuel_{liters}L"] = aggregate(parts)
rows = {}
for name, cfg in scenarios.items():
    model.total = cfg["mass_kg"]
    model.cg = np.array(cfg["cg_m"])
    sol = root(model.residual, [-0.24, 0.02, 0])
    assert sol.success and np.linalg.norm(model.residual(sol.x)) < 1e-7
    rows[name] = {
        "mass_kg": cfg["mass_kg"],
        "cg_m": cfg["cg_m"],
        "z_roll_pitch": sol.x.tolist(),
        "residual": model.residual(sol.x),
        "note": "No fuel free-surface/slosh correction; sensitivity inputs, not measured endpoints",
    }
(W / "../datasets/reference/mass_sensitivity.json").write_text(
    json.dumps(rows, indent=2)
)
print("SENSITIVITY", json.dumps({k: round(v["mass_kg"], 2) for k, v in rows.items()}))
