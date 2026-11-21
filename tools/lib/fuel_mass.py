"""积分准静态箱形油箱的质量属性并与干船体合并。

Quasi-static tank properties; independent convex-polytope integration for preparation.
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
import itertools
import json
import numpy as np
from scipy.spatial import ConvexHull

W = WORKSPACE


def shift(v):
    """计算平行轴定理所需的位移矩阵。

    Args:
        v: 参考点到目标点的三维位移，单位 m。
    """
    return np.dot(v, v) * np.eye(3) - np.outer(v, v)


def combine(a, b):
    """用平行轴定理合并两个同坐标系下的质量属性。

    Args:
        a: 第一组质量 kg、重心 m 和关于重心的惯性 kg·m²。
        b: 第二组质量属性，使用与 a 相同的坐标轴。
    """
    m = a["mass_kg"] + b["mass_kg"]
    c = (a["mass_kg"] * np.array(a["cg_m"]) + b["mass_kg"] * np.array(b["cg_m"])) / m
    inertia_data = (
        np.array(a["inertia_kg_m2"])
        + np.array(b["inertia_kg_m2"])
        + a["mass_kg"] * shift(np.array(a["cg_m"]) - c)
        + b["mass_kg"] * shift(np.array(b["cg_m"]) - c)
    )
    return dict(mass_kg=m, cg_m=c.tolist(), inertia_kg_m2=inertia_data.tolist())


def tank_properties(cfg, litres, up=(0, 0, 1)):
    """计算箱形油箱内准静态燃油的质量、重心和惯性。

    Args:
        cfg: 油箱内部尺寸 m、中心 m、密度 kg/m³ 的配置。
        litres: 燃油体积，单位 L，必须在油箱容量内。
        up: 油箱坐标系中的向上方向；内部归一化后用于水平面裁切。
    """
    size = np.array(cfg["inner_size_m"])
    centre = np.array(cfg["center_m"])
    rho = cfg["density_kg_m3"]
    vol = litres / 1000
    assert 0 <= vol <= np.prod(size) + 1e-12
    if vol < 1e-15:
        return dict(
            mass_kg=0.0, cg_m=centre.tolist(), inertia_kg_m2=np.zeros((3, 3)).tolist()
        )
    if abs(vol - np.prod(size)) < 1e-12:
        return dict(
            mass_kg=rho * vol,
            cg_m=centre.tolist(),
            inertia_kg_m2=np.diag(rho * vol / 12 * (sum(size**2) - size**2)).tolist(),
        )
    n = np.array(up, dtype=float)
    n /= np.linalg.norm(n)
    vertices = np.array(list(itertools.product([-1, 1], repeat=3))) * size / 2
    edges = [
        (i, j)
        for i in range(8)
        for j in range(i)
        if np.sum(vertices[i] != vertices[j]) == 1
    ]

    def moments(h, second=False):
        """裁切箱体并积分燃油多面体的体积与几何矩。

        Args:
            h: 沿向上方向的裁切平面高度，单位 m。
            second: 是否同时积分二阶几何矩。
        """
        d = vertices @ n - h
        pts = list(vertices[d <= 0])
        for i, j in edges:
            if (d[i] <= 0) != (d[j] <= 0):
                pts.append(
                    vertices[i] + (vertices[j] - vertices[i]) * d[i] / (d[i] - d[j])
                )
        if len(pts) < 4:
            return (0, np.zeros(3), np.zeros((3, 3)))
        pts = np.unique(np.round(pts, 14), axis=0)
        hull = ConvexHull(pts)
        ref = pts.mean(axis=0)
        v = 0.0
        f = np.zeros(3)
        s = np.zeros((3, 3))
        for tri, eq in zip(hull.simplices, hull.equations):
            a, b, c = pts[tri]
            if np.dot(np.cross(b - a, c - a), eq[:3]) < 0:
                b, c = c, b
            dv = np.dot(a - ref, np.cross(b - ref, c - ref)) / 6
            ss = ref + a + b + c
            v += dv
            f += dv * ss / 4
            if second:
                s += (
                    dv
                    * (np.outer(ss, ss) + sum(np.outer(x, x) for x in [ref, a, b, c]))
                    / 20
                )
        return v, f, s

    extent = abs(n) @ size / 2
    lo = -extent
    hi = extent
    for _ in range(42):
        mid = (lo + hi) / 2
        if moments(mid)[0] < vol:
            lo = mid
        else:
            hi = mid
    v, f, s = moments((lo + hi) / 2, True)
    c = f / v
    cov = s / v - np.outer(c, c)
    inertia_data = rho * vol * (np.trace(cov) * np.eye(3) - cov)
    return dict(
        mass_kg=rho * vol,
        cg_m=(centre + c).tolist(),
        inertia_kg_m2=inertia_data.tolist(),
    )


def configuration():
    """读取油箱配置，并优先从启用的设备台账取得干船体质量属性。"""
    cfg = json.loads((W / "src/usv_description/config/fuel_tank.json").read_text())
    load = json.loads((W / "src/usv_description/config/trial_load.json").read_text())
    if (W / "src/usv_description/config/component_mass_ledger.json").exists():
        from equipment_ledger import load as load_ledger, dry_links

        if load_ledger().get("active"):
            cfg["dry_hull"] = dry_links()["hull"]
            return cfg
    ref = cfg["reference_fuel"]
    m = load["hull_mass_kg"]
    c = np.array(load["hull_cg_m"])
    inertia_data = np.array(load["hull_inertia_matrix_kg_m2"])
    fm = ref["mass_kg"]
    fc = np.array(ref["cg_m"])
    fi = np.array(ref["inertia_kg_m2"])
    dm = m - fm
    dc = (m * c - fm * fc) / dm
    di = inertia_data + m * shift(c) - fi - fm * shift(fc) - dm * shift(dc)
    assert np.linalg.eigvalsh(di).min() > 0
    cfg["dry_hull"] = dict(mass_kg=dm, cg_m=dc.tolist(), inertia_kg_m2=di.tolist())
    return cfg
