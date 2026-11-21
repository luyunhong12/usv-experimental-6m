#!/usr/bin/env python3
"""汇总平台几何、功率与性能的工程估算。

Traceable geometry/power sanity calculations, not a CFD resistance prediction.
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
import math
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial import ConvexHull

w = WORKSPACE
out = w / "log/platform_validation/design"
out.mkdir(parents=True, exist_ok=True)
cfg = json.loads((w / "src/usv_bringup/config/platform.json").read_text())
scene = json.loads((w / "log/platform_validation/nominal/scene.json").read_text())
profile = scene["profile_details"]["usv"]
pose = scene["spawn_pose"]
R = Rotation.from_euler("xyz", pose[3:6]).as_matrix()
tri = np.loadtxt(w / "src/usv_description/config/hull_triangles.txt").reshape(-1, 3, 3)
tri = tri @ R.T + np.array([0, 0, pose[2]])
area = 0.0
wet = []
for t in tri:
    poly = []
    for i in range(3):
        a, b = t[i], t[(i + 1) % 3]
        inside_a = a[2] <= 0
        inside_b = b[2] <= 0
        if inside_a:
            poly.append(a)
        if inside_a != inside_b:
            poly.append(a + (b - a) * (-a[2] / (b[2] - a[2])))
    if len(poly) >= 3:
        wet.extend(poly)
        for i in range(1, len(poly) - 1):
            area += (
                np.linalg.norm(np.cross(poly[i] - poly[0], poly[i + 1] - poly[0])) / 2
            )
length = float(np.ptp(np.array(wet)[:, 0]))
u = cfg["design_speed_km_h"] / 3.6
rho = 1000.0
nu = 1.004e-6
re = u * length / nu
cf = 0.075 / (math.log10(re) - 2) ** 2
fr = u / math.sqrt(9.81 * length)
load = json.loads((w / "src/usv_description/config/trial_load.json").read_text())
prop = json.loads((w / "src/usv_description/config/propulsion.json").read_text())


def _load_additional_dependencies():
    """在当前路径和运行环境配置完成后加载依赖，保持原导入时机。"""
    from resistance_budget import allocate_resistance

    return (allocate_resistance,)


(allocate_resistance,) = _load_additional_dependencies()

budget = allocate_resistance(
    json.loads((w / "src/usv_description/config/resistance.json").read_text()),
    json.loads(
        (w / "src/usv_description/config/appendage_hydrodynamics.json").read_text()
    ),
)
# Full-wet reference total: hull residual plus separately represented appendages.
drag = (
    load["linear_damping_N_s_m"][0] * u
    + (budget["hull_quadratic_drag_N_s2_m2"][0] + budget["allocated_axial_N_s2_m2"])
    * u
    * u
)
friction = 0.5 * rho * u * u * area * cf
frontal = float(ConvexHull(np.array(wet)[:, 1:3]).volume)
cd_equivalent = drag / (0.5 * rho * u * u * frontal)
r = {
    "passed": True,
    "status": "GEOMETRY_AND_ENGINEERING_SANITY_ONLY",
    "design_speed_m_s": u,
    "estimated_waterline_length_m": length,
    "clipped_hull_wetted_area_m2": area,
    "water_density_kg_m3": rho,
    "water_kinematic_viscosity_m2_s": nu,
    "reynolds_number": re,
    "froude_number": fr,
    "ittc1957_cf": cf,
    "ittc_flat_plate_friction_n": friction,
    "submerged_frontal_convex_area_m2": frontal,
    "equivalent_total_drag_coefficient_at_design_speed": cd_equivalent,
    "configured_total_drag_n": drag,
    "effective_towing_power_w": drag * u,
    "configured_drag_bracket_n": [drag * 0.75, drag * 1.25],
    "ship_mass_kg": profile["ship_mass_kg"],
    "ship_cg_m": profile["ship_cg_m"],
    "equilibrium_z_roll_pitch": profile["pose_z_roll_pitch"],
    "basis": {
        "geometry": "Clipped sealed hull triangles at solved upgraded-load equilibrium; excludes appendages and roughness",
        "friction": "ITTC-1957 Cf=0.075/(log10(Re)-2)^2; freshwater viscosity assumed near 20 C",
        "total_drag": "Existing positive linear+quadratic engineering proxy. ITTC friction is a component-only check, NOT its calibration",
        "uncertainty": "+/-25% engineering stress bracket, not measured confidence bounds",
    },
    "limitations": [
        "Froude number near transition makes wave-making/dynamic trim significant; empirical proxy does not resolve these",
        "No automatic tuning to force the target speed; actual 40 kW actuator limit remains active",
        "No real vessel certification or purchase specification",
    ],
    "sources": [
        "https://www.ittc.info/media/10176/75-02-02-021.pdf",
        "https://github.com/osrf/vrx/wiki/wamv_params_tutorial",
    ],
}
checks = {
    "positive_mass_area_length": profile["ship_mass_kg"] > 0
    and area > 0
    and length > 0,
    "positive_passive_drag": drag > 0,
    "total_drag_exceeds_friction_component": drag > friction,
    "finite": all(math.isfinite(v) for v in [length, area, cf, re, fr, drag]),
}
checks = {k: bool(v) for k, v in checks.items()}
r["checks"] = checks
r["passed"] = all(checks.values())
(out / "result.json").write_text(json.dumps(r, indent=2))
print(json.dumps(r, indent=2))
