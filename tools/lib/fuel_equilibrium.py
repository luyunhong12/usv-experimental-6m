"""独立计算油量与姿态联动的静水平衡参考。

Independent static reference for fuel-linked hull and attitude-dependent liquid COM.
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
import json
import numpy as np
from scipy.optimize import root
from scipy.spatial.transform import Rotation
from fuel_mass import configuration, tank_properties, combine, W

cfg = json.loads((W / "src/usv_description/config/trial_load.json").read_text())
orig = json.loads((W / "../datasets/reference/boat_export.json").read_text())["origins"]
tank = configuration()
triangles = np.loadtxt(W / "src/usv_description/config/hull_triangles.txt").reshape(
    -1, 3, 3
)


def displaced(q):
    """计算给定位姿下的排水体积、浮心和旋转矩阵。

    Args:
        q: 高度 m、滚转 rad、俯仰 rad 组成的三元姿态。
    """
    z, roll, pitch = q
    R = Rotation.from_euler("xyz", [roll, pitch, 0]).as_matrix()
    tri = triangles @ R.T + [0, 0, z]
    volume = 0
    first = np.zeros(3)
    for t in tri:
        poly = []
        for a, b in zip(t, np.roll(t, -1, axis=0)):
            if a[2] <= 0:
                poly.append(a)
            if (a[2] <= 0) != (b[2] <= 0):
                poly.append(a + (b - a) * (-a[2]) / (b[2] - a[2]))
        for j in range(1, len(poly) - 1):
            a, b, c = poly[0], poly[j], poly[j + 1]
            dv = np.dot(a, np.cross(b, c)) / 6
            volume += dv
            first += dv * (a + b + c) / 4
    return volume, first / volume, R


others = []
for name in ["azimuth", "propeller"]:
    others.append(
        dict(
            mass_kg=cfg[name + "_mass_kg"],
            cg_m=(np.array(orig[name]) + cfg[name + "_cg_m"]).tolist(),
            inertia_kg_m2=cfg[name + "_inertia_matrix_kg_m2"],
        )
    )


def properties(litres, R):
    """合并指定姿态和油量下的船体及整船质量属性。

    Args:
        litres: 油箱内燃油体积，单位 L。
        R: 模型坐标到世界坐标的三维旋转矩阵。
    """
    hull = combine(
        tank["dry_hull"], tank_properties(tank, litres, R.T @ np.array([0.0, 0.0, 1.0]))
    )
    ship = hull
    for p in others:
        ship = combine(ship, p)
    return hull, ship


def solve(litres):
    """求解指定油量的静水平衡姿态，并返回质量与排水属性。

    Args:
        litres: 燃油体积，单位 L。
    """

    def residual(q):
        """计算排水体积及水平浮心重心对齐的残差。

        Args:
            q: 待求高度 m、滚转 rad、俯仰 rad。
        """
        v, cb, R = displaced(q)
        h, ship = properties(litres, R)
        cg = R @ np.array(ship["cg_m"]) + [0, 0, q[0]]
        return [
            v - ship["mass_kg"] / cfg["fluid_density_kg_m3"],
            cb[0] - cg[0],
            cb[1] - cg[1],
        ]

    sol = root(residual, [-0.239, 0.027, -0.003])
    assert sol.success and np.linalg.norm(residual(sol.x)) < 1e-8
    R = Rotation.from_euler("xyz", [sol.x[1], sol.x[2], 0]).as_matrix()
    hull, ship = properties(litres, R)
    return dict(
        fuel_l=litres,
        pose_z_roll_pitch=sol.x.tolist(),
        hull=hull,
        ship=ship,
        displacement_m3=ship["mass_kg"] / cfg["fluid_density_kg_m3"],
    )


if __name__ == "__main__":
    print(
        json.dumps(
            {str(fuel_litres): solve(fuel_litres) for fuel_litres in [60, 30, 6, 1, 0]},
            indent=2,
        )
    )
