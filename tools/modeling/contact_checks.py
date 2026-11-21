"""根据接触记录核对碰撞方向与预期部位。

Evaluate contact trajectories; a sphere may roll on a sloping inverted keel.
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
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation


def evaluate(rows, cases, hull):
    """从各碰撞工况的记录中检查接触位置与方向。

    Args:
        rows: 采集的接触记录。
        cases: 工况定义与预期接触条件。
        hull: 用于定位与判断的船体几何。
    """
    result = {}
    for idx, (name, rot, xy, lo, hi) in enumerate(cases):
        a = np.array(rows[name])
        ok = len(a) > 100
        z = float(a[-1, 2]) if len(a) else None
        spread = float(np.ptp(a[-50:, 2])) if len(a) > 50 else None
        entry = {
            "samples": len(a),
            "final_z_m": z,
            "last_50_z_range_m": spread,
            "expected_contact_surface_range_m": [lo, hi],
            "passed": bool(ok and lo - 0.03 < z < hi + 0.05 and spread < 0.01),
        }
        if name == "bottom" and ok:
            # Fixed body transformed by known test pose. Probe radius is 0.025 m.
            local = (a[-50:] - [idx * 20.0, 0, 0]) @ Rotation.from_euler(
                "xyz", rot
            ).as_matrix()
            mesh = trimesh.Trimesh(hull["vertices"], hull["faces"], process=False)
            distance = trimesh.proximity.closest_point_naive(mesh, local)[1]
            entry["criterion"] = (
                "Rolling contact: last 50 samples sphere-centre distance to hull within radius +/- 5 mm; not rest on keel ridge"
            )
            entry["surface_distance_min_max_m"] = [
                float(distance.min()),
                float(distance.max()),
            ]
            entry["passed"] = bool(np.all(np.abs(distance - 0.025) < 0.005))
        result[name] = entry
    return result
