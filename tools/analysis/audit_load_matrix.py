#!/usr/bin/env python3
"""审计载荷矩阵中的质量、重心与惯性一致性。

Independent post-run audit of native inertia, command environment and energy ledger.
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
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
from scipy.spatial.transform import Rotation
from fuel_mass import configuration, combine, tank_properties
from equipment_ledger import auxiliary_w

p = argparse.ArgumentParser()
p.add_argument("--output", type=Path, required=True)
a = p.parse_args()
audit = {}
for result in sorted(a.output.glob("*/result.json")):
    case = result.parent
    j = json.loads(result.read_text())
    spec = j["spec"]
    m = np.loadtxt(case / "fuel_mass_state.csv", delimiter=",")
    d = np.loadtxt(case / "propulsion_state.csv", delimiter=",")
    env = np.loadtxt(case / "environment_state.csv", delimiter=",")
    e = np.loadtxt(case / "energy_truth.csv", delimiter=",")
    act = np.loadtxt(case / "actuators_state.csv", delimiter=",")
    boat = E.parse(case / "boat.sdf")
    fc = boat.find(".//plugin[@name='usv::AzimuthPropulsion']/fuel_mass")
    dry = dict(
        mass_kg=float(fc.findtext("dry_mass")),
        cg_m=list(map(float, fc.findtext("dry_cg").split())),
        inertia_kg_m2=np.array(list(map(float, fc.findtext("dry_inertia").split())))
        .reshape(3, 3)
        .tolist(),
    )
    cfg = configuration()
    errs = []
    for row in m[np.linspace(0, len(m) - 1, 7).astype(int)]:
        # Native applied inertia is from the preceding pre-update pose; tolerate one 4ms step.
        pose = d[np.argmin(abs(d[:, 0] - row[0]))]
        R = Rotation.from_euler("xyz", [pose[15], pose[16], pose[14]]).as_matrix()
        p = combine(dry, tank_properties(cfg, row[1], R.T @ np.array([0, 0, 1.0])))
        inertia_data = np.array(p["inertia_kg_m2"])
        want = inertia_data[[0, 1, 2, 0, 0, 1], [0, 1, 2, 1, 2, 2]]
        errs.append([np.max(abs(row[3:6] - p["cg_m"])), np.max(abs(row[6:12] - want))])
    cg, ie = np.max(errs, axis=0)
    control = np.loadtxt(case / "controller.csv", delimiter=",")
    checks = {
        "controller_telemetry_received": bool(len(control) > 100),
        "native_hull_cg_matches_independent_fuel_geometry": bool(cg < 1e-4),
        "native_hull_inertia_matches_independent_fuel_geometry": bool(ie < 0.02),
        "wind_received": bool(
            np.max(abs(env[env[:, 0] > 10, 7:10] - spec["wind"])) < 1e-8
        ),
    }
    if spec["wave_gain"] == 0:
        checks["current_received"] = bool(
            np.max(abs(env[env[:, 0] > 10, 4:7] - spec["current"])) < 1e-8
        )
    else:
        checks["wave_height_changes"] = bool(np.ptp(env[env[:, 0] > 10, 1]) > 0.02)
    # auxiliary_w telemetry includes DC/DC loss and dynamic steering electrical demand.
    aw = auxiliary_w(spec.get("power_mode", "normal"), "speed15_concept") / 0.87
    steering = np.interp(e[:, 0], act[:, 0], act[:, 5])
    power_err = float(np.max(abs(e[:, 8] - steering - aw)))
    checks["aux_power_matches_itemized_budget"] = power_err < 1.0
    audit[case.name] = {
        "checks": checks,
        "passed": all(checks.values()),
        "max_cg_error_m": float(cg),
        "max_inertia_error_kg_m2": float(ie),
        "max_aux_power_error_w": power_err,
    }
(a.output / "independent_audit.json").write_text(
    json.dumps(
        {"passed": all(x["passed"] for x in audit.values()), "cases": audit}, indent=2
    )
)
print(json.dumps(audit, indent=2))
