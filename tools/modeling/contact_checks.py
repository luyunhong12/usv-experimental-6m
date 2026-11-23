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


def sphere_mesh_gaps(points, vertices, faces, radius):
    """计算球面到凸实体的有符号间隙，负数表示穿透。

    Args:
        points: 实体局部坐标中的球心序列，形状 N×3，单位 m。
        vertices: 凸实体顶点，形状 V×3，单位 m。
        faces: 三角形顶点索引，形状 F×3。
        radius: 球半径，单位 m。

    Returns:
        每个球心的间隙数组，单位 m；球心进入实体内部也会判为穿透。
    """
    from scipy.spatial import ConvexHull

    mesh = trimesh.Trimesh(vertices, faces, process=False)
    plane = ConvexHull(vertices).equations
    distances = []
    for start in range(0, len(points), 128):
        part = np.asarray(points[start : start + 128])
        distance = trimesh.proximity.closest_point_naive(mesh, part)[1]
        inside = np.all(part @ plane[:, :3].T + plane[:, 3] <= 1e-10, axis=1)
        distances.extend(np.where(inside, -distance, distance) - radius)
    return np.asarray(distances)


def evaluate_fin_precision(rows, times, index, rotation, vertices, faces, step, height):
    """按法向导轨探针核验稳态误差、冲击离散误差和全程采样。

    Args:
        rows: 探针世界坐标，形状 N×3，单位 m。
        times: 与坐标一一对应的仿真时间，单位 s。
        index: 船模阵列位置索引，每个模型沿世界 x 间隔 20 m。
        rotation: 固定船体的 XYZ 欧拉角，单位 rad。
        vertices: 真实薄鳍顶点，单位 m。
        faces: 真实薄鳍三角形索引。
        step: 场景最大积分步长，单位 s。
        height: 球心初始距接触面的高度，单位 m。

    Returns:
        明确的数值、误差上限、采样检查与通过状态；缺采样不能通过。
    """
    if len(rows) < 2 or len(rows) != len(times):
        return {"passed": False, "reason": "Missing contact samples"}
    points = (np.asarray(rows) - [index * 20.0, 0, 0]) @ Rotation.from_euler(
        "xyz", rotation
    ).as_matrix()
    t = np.asarray(times)
    gaps = sphere_mesh_gaps(points, vertices, faces, 0.025)
    tail = gaps[t >= t[-1] - 1]
    # 离散检测允许至多一个步长的自由飞行；加 0.2 mm 求解误差余量。
    impact_speed = np.sqrt(2 * 9.81 * (height - 0.025))
    peak_limit = impact_speed * step + 0.5 * 9.81 * step**2 + 0.0002
    checks = {
        "finite": bool(np.isfinite(points).all() and np.isfinite(gaps).all()),
        "duration": bool(t[-1] - t[0] >= 8),
        "step_sampling": bool(
            np.all(np.diff(t) > 0) and np.max(np.diff(t)) <= 1.5 * step
        ),
        "steady_gap": bool(np.max(np.abs(tail)) <= 0.0005),
        "impact_penetration": bool(gaps.min() >= -peak_limit),
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "samples": len(rows),
        "duration_s": float(t[-1] - t[0]),
        "maximum_sample_interval_s": float(np.max(np.diff(t))),
        "tail_gap_min_max_m": [float(tail.min()), float(tail.max())],
        "peak_penetration_m": float(max(0, -gaps.min())),
        "peak_limit_m": float(peak_limit),
        "steady_limit_m": 0.0005,
        "impact_speed_m_s": float(impact_speed),
        "step_s": step,
        "fixture": "frictionless normal guide; no normal support except collision",
    }
