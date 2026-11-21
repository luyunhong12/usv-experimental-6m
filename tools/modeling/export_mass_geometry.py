# Resolve the checkout independently of the current working directory.
"""导出质量估算所需的几何与来源信息。"""

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
import json
import os

W = WORKSPACE
R = W.parent
bpy.ops.wm.open_mainfile(
    filepath=str(R / "design/azimuth_thruster/模型/第二船体_中央单机整体转向.blend")
)
bpy.context.scene.frame_set(1)
dg = bpy.context.evaluated_depsgraph_get()
# Actual added structural solids; electronics, bearing/gear assemblies use lumped budgets.
keys = [
    "V2 中央艉板",
    "V2 艉伸安装梁",
    "V2 舱内纵向补强",
    "V2 轴承横梁",
    "V2 执行器上托梁",
    "V2 回转轴",
]
rows = []
for o in bpy.context.scene.objects:
    if (
        not o.visible_get()
        or o.hide_render
        or o.type != "MESH"
        or not any(o.name.startswith(k) for k in keys)
    ):
        continue
    e = o.evaluated_get(dg)
    m = e.to_mesh()
    m.calc_loop_triangles()
    rows.append(
        {
            "name": o.name,
            "vertices": [list(e.matrix_world @ v.co) for v in m.vertices],
            "faces": [list(t.vertices) for t in m.loop_triangles],
        }
    )
    e.to_mesh_clear()
(W / "../datasets/reference/added_structural_geometry.json").write_text(
    json.dumps(rows, ensure_ascii=False)
)
print("STRUCTURAL_SOLIDS", len(rows), flush=True)
os._exit(0)
