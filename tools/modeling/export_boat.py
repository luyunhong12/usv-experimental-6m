"""导出船舶显示与碰撞网格及对应几何记录。

Read-only Blender source export: evaluated visible meshes, three rigid groups.
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
from project_paths import PROJECT_ROOT
import bpy
import json
import hashlib
import os
import sys
from pathlib import Path
from mathutils import Vector
from collections import defaultdict
import xml.etree.ElementTree as E

R = PROJECT_ROOT
W = R / "ros_ws"
out = W / "src/usv_description/meshes"
collision_mode = os.environ.get("USV_COLLISION_EXPORT") == "1"
source = R / "design/azimuth_thruster/模型/第二船体_中央单机整体转向.blend"
bpy.ops.wm.open_mainfile(filepath=str(source))
bpy.context.scene.frame_set(1)
sys.path.insert(0, str(Path(__file__).resolve().parent))


def _load_runtime_dependencies():
    """在当前路径和运行环境配置完成后加载依赖，保持原导入时机。"""
    from refine_visual_geometry import refine_scene

    return (refine_scene,)


(refine_scene,) = _load_runtime_dependencies()

refinement = refine_scene(collision_mode)
visual_cfg = json.loads(
    (W / "src/usv_description/config/visual_refinement.json").read_text()
)
az = bpy.data.objects["V2_AZIMUTH · 中央垂直转向轴"]
prop = bpy.data.objects["V2_PROP_SPIN · 桨轴自转"]
origins = {
    "hull": Vector((0, 0, 0)),
    "azimuth": az.matrix_world.translation.copy(),
    "propeller": prop.matrix_world.translation.copy(),
}
exclusion_config = (
    json.loads((W / "src/usv_description/config/collision_exclusions.json").read_text())
    if (W / "src/usv_description/config/collision_exclusions.json").exists()
    else {"rules": []}
)
excluded = []
groups = defaultdict(list)
deps = bpy.context.evaluated_depsgraph_get()
for o in bpy.context.scene.objects:
    if o.type not in ("MESH", "CURVE") or o.hide_render or not o.visible_get():
        continue
    if collision_mode:
        rule = next(
            (r for r in exclusion_config["rules"] if o.name.startswith(r["prefix"])),
            None,
        )
        if rule:
            excluded.append({"object": o.name, "reason": rule["reason"]})
            continue
    ancestors = []
    p = o.parent
    while p:
        ancestors.append(p)
        p = p.parent
    group = (
        "propeller" if prop in ancestors else "azimuth" if az in ancestors else "hull"
    )
    if collision_mode and group == "hull":
        bounds = [o.matrix_world @ Vector(v) for v in o.bound_box]
        if o.name == "C方案船体":
            continue
        # Exterior selection by actual geometry bounds, not material names.
        if not (
            max(v.z for v in bounds) >= 0.86
            or max(abs(v.y) for v in bounds) >= 1.1
            or min(v.x for v in bounds) < -3.0
        ):
            continue
    groups[group].append(o)
report = {
    "refinement": refinement,
    "normal_export": "evaluated per-corner split normals; inverse transpose; mirrored winding corrected",
    "material_export": "Original viewport palette; named finish overrides; COLLADA Phong approximation",
    "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
    "units": "m",
    "frame": "FLU",
    "origins": {k: list(v) for k, v in origins.items()},
    "groups": {},
}
for group, objects in groups.items():
    vertices = []
    normals = []
    normal_ids = {}
    triangles = defaultdict(list)
    for o in objects:
        ev = o.evaluated_get(deps)
        m = ev.to_mesh()
        m.calc_loop_triangles()
        base = len(vertices)
        mat = ev.matrix_world
        vertices.extend(list(mat @ v.co - origins[group]) for v in m.vertices)
        normalmat = mat.to_3x3().inverted_safe().transposed()
        corners = m.corner_normals
        mirrored = mat.to_3x3().determinant() < 0
        material_keys = []
        for material in m.materials:
            color = (0.45, 0.49, 0.52, 1.0)
            metallic = 0.0
            roughness = 0.6
            if material:
                color = tuple(material.diffuse_color)
                metallic = material.metallic
                roughness = material.roughness
                for rule in visual_cfg["material_overrides"]:
                    if rule["contains"] in material.name:
                        metallic = rule["metallic"]
                        roughness = rule["roughness"]
                        break
            material_keys.append(
                tuple(round(float(x), 4) for x in (*color, metallic, roughness))
            )
        for t in m.loop_triangles:
            key = (
                material_keys[t.material_index]
                if t.material_index < len(material_keys)
                else (0.45, 0.49, 0.52, 1.0, 0.0, 0.6)
            )
            indices = list(zip(t.vertices, t.loops))
            if mirrored:
                indices.reverse()
            pairs = []
            for vi, li in indices:
                normal = tuple(
                    round(x, 7) for x in (normalmat @ corners[li].vector).normalized()
                )
                ni = normal_ids.get(normal)
                if ni is None:
                    ni = len(normals)
                    normal_ids[normal] = ni
                    normals.append(normal)
                pairs.append((base + vi, ni))
            triangles[key].append(pairs)
        ev.to_mesh_clear()
    doc = E.Element(
        "COLLADA", xmlns="http://www.collada.org/2005/11/COLLADASchema", version="1.4.1"
    )
    asset = E.SubElement(doc, "asset")
    E.SubElement(asset, "unit", name="meter", meter="1")
    E.SubElement(asset, "up_axis").text = "Z_UP"
    effects = E.SubElement(doc, "library_effects")
    materials = E.SubElement(doc, "library_materials")
    geo = E.SubElement(E.SubElement(doc, "library_geometries"), "geometry", id="mesh")
    mesh = E.SubElement(geo, "mesh")
    for name, data in [("pos", vertices), ("norm", normals)]:
        src = E.SubElement(mesh, "source", id=name)
        E.SubElement(
            src, "float_array", id=name + "-array", count=str(len(data) * 3)
        ).text = " ".join(f"{x:.7g}" for v in data for x in v)
        acc = E.SubElement(
            E.SubElement(src, "technique_common"),
            "accessor",
            source="#" + name + "-array",
            count=str(len(data)),
            stride="3",
        )
        for a in "XYZ":
            E.SubElement(acc, "param", name=a, type="float")
    E.SubElement(
        E.SubElement(mesh, "vertices", id="verts"),
        "input",
        semantic="POSITION",
        source="#pos",
    )
    for i, (key, ts) in enumerate(triangles.items()):
        color = key[:4]
        metallic, roughness = key[4:]
        effect = E.SubElement(effects, "effect", id=f"e{i}")
        tech = E.SubElement(
            E.SubElement(effect, "profile_COMMON"), "technique", sid="common"
        )
        lam = E.SubElement(tech, "phong")
        E.SubElement(E.SubElement(lam, "diffuse"), "color").text = " ".join(
            map(str, color)
        )
        spec = [0.04 * (1 - metallic) + c * metallic for c in color[:3]] + [1.0]
        E.SubElement(E.SubElement(lam, "specular"), "color").text = " ".join(
            map(str, spec)
        )
        E.SubElement(E.SubElement(lam, "shininess"), "float").text = str(
            min(128, max(2, 2 / max(0.05, roughness) ** 2 - 2))
        )
        E.SubElement(
            E.SubElement(materials, "material", id=f"m{i}", name=f"m{i}"),
            "instance_effect",
            url=f"#e{i}",
        )
        tr = E.SubElement(mesh, "triangles", count=str(len(ts)), material=f"m{i}")
        E.SubElement(tr, "input", semantic="VERTEX", source="#verts", offset="0")
        E.SubElement(tr, "input", semantic="NORMAL", source="#norm", offset="1")
        E.SubElement(tr, "p").text = " ".join(
            str(x) for t in ts for pair in t for x in pair
        )
    scene = E.SubElement(
        E.SubElement(doc, "library_visual_scenes"), "visual_scene", id="Scene"
    )
    node = E.SubElement(scene, "node", id="boat")
    inst = E.SubElement(node, "instance_geometry", url="#mesh")
    bind = E.SubElement(E.SubElement(inst, "bind_material"), "technique_common")
    for i in range(len(triangles)):
        E.SubElement(bind, "instance_material", symbol=f"m{i}", target=f"#m{i}")
    E.SubElement(E.SubElement(doc, "scene"), "instance_visual_scene", url="#Scene")
    E.ElementTree(doc).write(
        out / f'{"collision_" if collision_mode else ""}{group}.dae',
        encoding="utf-8",
        xml_declaration=True,
    )
    report["groups"][group] = {
        "objects": len(objects),
        "vertices": len(vertices),
        "triangles": sum(map(len, triangles.values())),
        "bounds_local": [
            [min(v[i] for v in vertices) for i in range(3)],
            [max(v[i] for v in vertices) for i in range(3)],
        ],
        "object_names": [o.name for o in objects],
    }
    print(group, report["groups"][group]["triangles"], flush=True)
if collision_mode:
    report["excluded_internal_or_flexible"] = excluded
    report["selection_rule"] = (
        "Fixed visible geometry reaching z>=0.86 m, |y|>=1.1 m, or x<-3 m; all moving azimuth and propeller geometry. Sealed main hull added separately. Not complete interior collision coverage."
    )
    (W / "../datasets/reference/collision_export.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2)
    )
    print("COLLISION_EXPORT_COMPLETE", flush=True)
    os._exit(0)
# Base hull before display edge-splitting modifiers: closed displacement envelope.
o = bpy.data.objects["C方案船体"]
m = o.data
m.calc_loop_triangles()
verts = [list(o.matrix_world @ v.co) for v in m.vertices]
faces = [list(t.vertices) for t in m.loop_triangles]
(W / "src/usv_description/config/hull_geometry.json").write_text(
    json.dumps(
        {
            "vertices": verts,
            "faces": faces,
            "source": "C方案船体 base mesh; geometry only, not shell material volume",
        },
        indent=2,
    )
)
(W / "../datasets/reference/boat_export.json").write_text(
    json.dumps(report, ensure_ascii=False, indent=2)
)
print("EXPORT_COMPLETE", flush=True)
os._exit(0)
