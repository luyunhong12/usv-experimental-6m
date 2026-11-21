#!/usr/bin/env python3
"""准备固定参考工况的浸水几何与阻力依据。

Freeze a declared nominal geometry reference, not a speed calibration.
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
import hashlib
import numpy as np
from scipy.spatial.transform import Rotation

w = WORKSPACE


def _load_additional_dependencies():
    """在当前路径和运行环境配置完成后加载依赖，保持原导入时机。"""
    import argparse

    return (argparse,)


(argparse,) = _load_additional_dependencies()

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument(
    "--scene",
    type=_ProjectPath,
    required=True,
    help="Explicit nominal speed15 / 60 L / zero payload scene.json",
)
args = parser.parse_args()
source = args.scene
pose = json.loads(source.read_text())["spawn_pose"]
q = pose[2:5]
R = Rotation.from_euler("xyz", [q[1], q[2], 0]).as_matrix()
file = w / "src/usv_description/config/hull_triangles.txt"
tri = np.loadtxt(file).reshape(-1, 3, 3)
area = 0.0
projected = np.zeros(3)
for t in tri:
    h = -(t @ R.T + np.array([0, 0, q[0]]))[:, 2]
    poly = []
    for i in range(3):
        j = (i + 1) % 3
        if h[i] >= 0:
            poly.append(t[i])
        if (h[i] >= 0) != (h[j] >= 0):
            poly.append(t[i] + h[i] / (h[i] - h[j]) * (t[j] - t[i]))
    for j in range(1, len(poly) - 1):
        n = np.cross(poly[j] - poly[0], poly[j + 1] - poly[0]) / 2
        area += np.linalg.norm(n)
        projected += abs(n) / 2
config_path = w / "src/usv_description/config/resistance.json"
cfg = json.loads(config_path.read_text())
cfg.update(
    {
        "status": "GEOMETRY_SCALED_ENGINEERING_DAMPING_NOT_CALIBRATED",
        "enabled": True,
        "reference_profile": "speed15_concept nominal / 60 L / zero payload / static equilibrium",
        "reference_pose_z_roll_pitch": q,
        "reference_wet_area_m2": float(area),
        "reference_projected_area_m2": projected.tolist(),
        "geometry_sha256": hashlib.sha256(file.read_bytes()).hexdigest(),
        "law": "linear translation and angular damping scale by wet_area/reference; quadratic translation scales by half-integral(abs(body normal_i))*wet_area divided by same reference directional area",
        "basis": "Quadratic drag is proportional to a reference area for fixed coefficient; preserve existing nominal damping coefficients, explicitly add geometric immersion dependence. This is a surrogate, not resistance identification or a wave-making prediction.",
        "sources": ["https://ittc.info/media/1816/75-02-02-01.pdf"],
        "limitations": cfg["limitations"]
        + [
            "Projected area uses a half normal integral of the wetted hull, not a measured drag coefficient",
            "Linear/angular area scaling is an engineering assumption; no frequency-dependent radiation",
            "No separate ITTC skin friction is added: existing lumped drag already includes unknown friction/form/wave contributions",
            "Reference remains fixed across load and fuel cases; never normalize each case back to itself",
        ],
    }
)
cfg["limitations"] = list(dict.fromkeys(cfg["limitations"]))
cfg["reference_scene"] = str(source.resolve().relative_to(w.parent))
config_path.write_text(json.dumps(cfg, indent=2) + "\n")
print(json.dumps(cfg, indent=2))
