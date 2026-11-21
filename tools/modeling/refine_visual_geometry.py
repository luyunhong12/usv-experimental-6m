"""细化导出几何，保持原始 Blender 文件不变。

Non-destructive export refinement; source .blend is never saved.

Blade geometry is shared by visual/collision exports and the sweep validator.
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
import bpy
import math
import json
from mathutils import Vector


def refine_scene(collision_mode=False):
    """细化导出场景的几何，保持源 Blender 文件不被保存。

    Args:
        collision_mode: 是否生成适用于碰撞导出的几何。
    """
    cfg = json.loads(
        (WORKSPACE / "src/usv_description/config/visual_refinement.json").read_text()
    )
    records = []
    for o in list(bpy.context.scene.objects):
        if o.type != "MESH" or o.hide_render or not o.visible_get():
            continue
        if o.name.startswith("V2概念单机 · 曲面桨叶"):
            old = o.data
            lo = Vector(tuple(min(v.co[i] for v in old.vertices) for i in range(3)))
            hi = Vector(tuple(max(v.co[i] for v in old.vertices) for i in range(3)))
            original_count = len(old.vertices)
            sub = o.modifiers.new("Simulation blade fairing", "SUBSURF")
            sub.subdivision_type = "CATMULL_CLARK"
            sub.levels = cfg["blade_subdivision_levels"]
            sub.render_levels = sub.levels
            bpy.context.view_layer.update()
            dg = bpy.context.evaluated_depsgraph_get()
            refined = bpy.data.meshes.new_from_object(o.evaluated_get(dg), depsgraph=dg)
            low = Vector(
                tuple(min(v.co[i] for v in refined.vertices) for i in range(3))
            )
            high = Vector(
                tuple(max(v.co[i] for v in refined.vertices) for i in range(3))
            )
            # Preserve the original blade's local envelope and assembly transform.
            for v in refined.vertices:
                for i in range(3):
                    v.co[i] = (
                        lo[i]
                        + (v.co[i] - low[i]) * (hi[i] - lo[i]) / (high[i] - low[i])
                        if high[i] > low[i]
                        else lo[i]
                    )
            o.modifiers.clear()
            o.data = refined
            for f in refined.polygons:
                f.use_smooth = True
            refined.update()
            records.append(
                dict(
                    object=o.name,
                    method="Catmull-Clark 2; original local envelope restored",
                    vertices_before=original_count,
                    vertices_after=len(refined.vertices),
                    same_collision_geometry=True,
                )
            )
        elif o.name == "C方案船体" and not collision_mode:
            # Remove display-only vertex splitting before evaluating the small
            # edge bevel. Physical sealed-hull geometry remains the base mesh.
            for m in list(o.modifiers):
                if m.type == "EDGE_SPLIT":
                    o.modifiers.remove(m)
            bevel = o.modifiers.new("2 mm plate-edge highlight", "BEVEL")
            bevel.width = cfg["hull_cosmetic_bevel_m"]
            bevel.segments = cfg["hull_cosmetic_bevel_segments"]
            bevel.limit_method = "ANGLE"
            bevel.angle_limit = math.radians(30)
            bevel.use_clamp_overlap = True
            bevel.harden_normals = True
            records.append(
                dict(
                    object=o.name,
                    method="2 mm cosmetic bevel, 3 segments; hard chines retained",
                    collision_geometry_unchanged=True,
                )
            )
    bpy.context.view_layer.update()
    return records
