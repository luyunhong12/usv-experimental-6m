"""扫描推进装配运动范围内的几何净空。

Read-only audit of the exact three-rigid-body geometry exported to Gazebo.
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
from project_paths import PROJECT_ROOT, WORKSPACE
import bpy
import math
import json
import os
import time
import hashlib
import sys
from mathutils import Vector, Matrix
from mathutils.bvhtree import BVHTree

W = WORKSPACE
R = W.parent
bpy.ops.wm.open_mainfile(
    filepath=str(R / "design/azimuth_thruster/模型/第二船体_中央单机整体转向.blend")
)
bpy.context.scene.frame_set(1)
sys.path.insert(0, str(PROJECT_ROOT / "tools/modeling"))


def _load_runtime_dependencies():
    """在当前路径和运行环境配置完成后加载依赖，保持原导入时机。"""
    from refine_visual_geometry import refine_scene

    return (refine_scene,)


(refine_scene,) = _load_runtime_dependencies()

refine_scene(True)
dg = bpy.context.evaluated_depsgraph_get()
manifest = json.loads((W / "../datasets/reference/collision_export.json").read_text())
origins = manifest["origins"]
az = Vector(origins["azimuth"])
pc = Vector(origins["propeller"])


def merged(names):
    """合并指定 Blender 对象的顶点和面索引。

    Args:
        names: 待合并对象名称集合
    """
    verts = []
    faces = []
    owners = []
    for name in names:
        o = bpy.data.objects[name]
        e = o.evaluated_get(dg)
        m = e.to_mesh()
        m.calc_loop_triangles()
        base = len(verts)
        verts.extend(e.matrix_world @ v.co for v in m.vertices)
        for t in m.loop_triangles:
            faces.append(tuple(base + i for i in t.vertices))
            owners.append(name)
        e.to_mesh_clear()
    return verts, faces, owners


profile = os.environ.get("USV_SWEEP_PROFILE", "baseline")
if profile not in ["baseline", "speed15_concept"]:
    raise ValueError("Unknown sweep profile")
scale = (
    json.loads(
        (W / "src/usv_description/config/profiles/speed15_concept.json").read_text()
    )["propeller_geometry_scale"]
    if profile == "speed15_concept"
    else 1.0
)
parts = {
    g: merged(x["object_names"] + (["C方案船体"] if g == "hull" else []))
    for g, x in manifest["groups"].items()
}


def tree(group, transform):
    """按指定变换生成装配部件的碰撞查询树。

    Args:
        group: 已登记的部件分组名称
        transform: 施加到该组顶点的坐标变换
    """
    v, f, n = parts[group]
    return BVHTree.FromPolygons(
        [transform @ (pc + (p - pc) * scale if group == "propeller" else p) for p in v],
        f,
        all_triangles=True,
    )


fixed = tree("hull", Matrix.Identity(4))
rows = []
allpairs = {}
start = time.monotonic()
clearance = {
    "distance_m": float("inf"),
    "method": "sampled moving-propeller vertices to fixed hull triangle surfaces; not a continuous minimum-clearance certificate",
    "samples": 0,
}


def pairs(a, b, ga, gb):
    """列出两个查询树发生重叠的原始对象名称对。

    Args:
        a: 第一个 BVH 查询树
        b: 第二个 BVH 查询树
        ga: 第一个树的部件组名
        gb: 第二个树的部件组名
    """
    return sorted(set((parts[ga][2][i], parts[gb][2][j]) for i, j in a.overlap(b)))


for steering in range(-35, 36):
    rz = (
        Matrix.Translation(az)
        @ Matrix.Rotation(math.radians(steering), 4, "Z")
        @ Matrix.Translation(-az)
    )
    moving = tree("azimuth", rz)
    bodypairs = pairs(moving, fixed, "azimuth", "hull")
    hits = []
    for spin in range(0, 360, 5):
        rx = (
            Matrix.Translation(pc)
            @ Matrix.Rotation(math.radians(spin), 4, "X")
            @ Matrix.Translation(-pc)
        )
        prop = tree("propeller", rz @ rx)
        if steering % 5 == 0 and spin % 15 == 0:
            transform = rz @ rx
            for vertex in parts["propeller"][0]:
                point = transform @ (pc + (vertex - pc) * scale)
                nearest = fixed.find_nearest(point)
                clearance["samples"] += 1
                if nearest[0] is not None and nearest[3] < clearance["distance_m"]:
                    clearance.update(
                        distance_m=float(nearest[3]),
                        steering_deg=steering,
                        spin_deg=spin,
                    )
        p = pairs(prop, fixed, "propeller", "hull") + pairs(
            prop, moving, "propeller", "azimuth"
        )
        if p:
            hits.append({"spin_deg": spin, "pairs": p})
    row = {"steering_deg": steering, "body_pairs": bodypairs, "propeller_hits": hits}
    rows.append(row)
    for p in bodypairs + [p for hit in hits for p in hit["pairs"]]:
        key = " | ".join(p)
        if key not in allpairs:
            allpairs[key] = {"first_angle": steering, "last_angle": steering}
        allpairs[key]["last_angle"] = steering
    if steering % 10 == 0:
        print(
            "SWEEP",
            steering,
            "pairs",
            len(allpairs),
            "seconds",
            round(time.monotonic() - start),
            flush=True,
        )
report = {
    "propeller_to_fixed_sampled_clearance": clearance,
    "profile": profile,
    "propeller_geometry_scale": scale,
    "method": "Triangle surface intersection BVH, evaluated exported rigid groups; no material deformation. Closed-solid containment and continuous clearance not certified.",
    "steering_step_deg": 1,
    "spin_step_deg": 5,
    "samples": len(rows) * 72,
    "source_sha256": manifest["source_sha256"],
    "collision_manifest_sha256": hashlib.sha256(
        (W / "../datasets/reference/collision_export.json").read_bytes()
    ).hexdigest(),
    "exclusion_policy_sha256": hashlib.sha256(
        (W / "src/usv_description/config/collision_exclusions.json").read_bytes()
    ).hexdigest(),
    "pair_summary": allpairs,
    "poses": rows,
    "elapsed_s": time.monotonic() - start,
}
(
    W
    / (
        "../datasets/reference/steering_propeller_sweep_raw.json"
        if profile == "baseline"
        else "../datasets/reference/platform15_propeller_sweep_raw.json"
    )
).write_text(json.dumps(report, ensure_ascii=False, indent=2))
print("SWEEP_DONE", len(allpairs), flush=True)
os._exit(0)
