"""从台账生成小型被动鳍的同源显示、碰撞、浮力及升阻力几何。"""

import json
from pathlib import Path
import xml.etree.ElementTree as E

import numpy as np
from scipy.spatial import ConvexHull

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "ros_ws/src/usv_description/config"


def configuration():
    """读取唯一的被动鳍台账条目，尺寸和材料属性不另存副本。"""
    data = json.loads((CONFIG / "component_mass_ledger.json").read_text())
    return next(p for p in data["parts"] if p["id"] == "passive_stabilizer")


def geometry(part):
    """生成模型 FLU 坐标中的闭合外向网格及侧向轮廓，单位 m。

    Args:
        part: 台账部件；椭圆截面、根部坐标、末端线性收缩尺寸须有效。

    Returns:
        顶点、三角形索引及 x-z 凸轮廓。不会写文件或修改台账。

    Raises:
        ValueError: 非有限尺寸、无效截面数或收缩范围。
    """
    g = part["geometry"]
    size = np.asarray(g["size_m"], dtype=float)
    root = np.asarray(g["root_center_m"], dtype=float)
    taper, scale, count = g["tip_taper_m"], g["tip_scale"], g["sections"]
    if (
        size.shape != (3,)
        or root.shape != (3,)
        or not np.isfinite([*size, *root, taper, scale]).all()
        or min(size) <= 0
        or not 0 < taper < size[2]
        or not 0 < scale < 1
        or not isinstance(count, int)
        or not 16 <= count <= 256
        or count % 4
    ):
        raise ValueError("Invalid passive stabilizer geometry")
    angle = np.arange(count) * (2 * np.pi / count)
    levels = [(0, 1), (-size[2] + taper, 1), (-size[2], scale)]
    vertices = (
        np.array(
            [
                [size[0] * k * np.cos(t) / 2, size[1] * k * np.sin(t) / 2, z]
                for z, k in levels
                for t in angle
            ]
        )
        + root
    )
    hull = ConvexHull(vertices)
    faces = hull.simplices.copy()
    for f, normal in zip(faces, hull.equations[:, :3]):
        if (
            np.dot(
                np.cross(
                    vertices[f[1]] - vertices[f[0]], vertices[f[2]] - vertices[f[0]]
                ),
                normal,
            )
            < 0
        ):
            f[1], f[2] = f[2], f[1]
    side = np.unique(vertices[:, [0, 2]], axis=0)
    outline = side[ConvexHull(side).vertices]
    return vertices, faces, outline


def mass_properties(part):
    """按定向四面体积分计算名义材料质量、重心及重心处惯性。

    Args:
        part: 含几何尺寸与正材料密度 kg/m³ 的台账条目。

    Returns:
        体积 m³、质量 kg、模型重心 m 和惯性 kg·m²。
    """
    vertices, faces, _ = geometry(part)
    t = vertices[faces]
    volume = np.einsum("ij,ij->i", t[:, 0], np.cross(t[:, 1], t[:, 2])) / 6
    total = volume.sum()
    sums = t.sum(axis=1)
    cg = (volume[:, None] * sums / 4).sum(axis=0) / total
    second = sum(
        v * (np.outer(s, s) + tri.T @ tri) / 20 for v, s, tri in zip(volume, sums, t)
    )
    covariance = second - total * np.outer(cg, cg)
    density = part["geometry"]["material_density_kg_m3"]
    if not np.isfinite(density) or density <= 0:
        raise ValueError("Invalid stabilizer density")
    inertia = density * (np.trace(covariance) * np.eye(3) - covariance)
    return total, density * total, cg, inertia


def add_stabilizer(model, part, directory):
    """写入同源网格并替换固定鳍显示、碰撞和被动水动力插件。

    质量由调用方通过设备台账合并；本函数不二次添加惯性。
    新鳍阻力额外计入，不从旧船体轴向预算中扣除。

    Args:
        model: 原地更新的 SDF model。
        part: 当前方案已选择的被动鳍台账条目。
        directory: 生成网格目录；文件按确定性内容更新。
    """
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    vertices, faces, outline = geometry(part)
    mesh = directory / "passive_stabilizer.obj"
    triangles_geometry = vertices[faces]
    normals = np.cross(
        triangles_geometry[:, 1] - triangles_geometry[:, 0],
        triangles_geometry[:, 2] - triangles_geometry[:, 0],
    )
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    mesh.write_text(
        "\n".join(
            [
                *("v " + " ".join(f"{x:.12g}" for x in v) for v in vertices),
                *("vn " + " ".join(f"{x:.12g}" for x in n) for n in normals),
                *(
                    "f " + " ".join(f"{i + 1}//{j + 1}" for i in f)
                    for j, f in enumerate(faces)
                ),
            ]
        )
        + "\n"
    )
    triangles = directory / "passive_stabilizer_triangles.txt"
    np.savetxt(triangles, vertices[faces].reshape(-1, 9), fmt="%.12g")
    hull = model.find("link[@name='hull']")
    for kind in ["visual", "collision"]:
        name = "passive_stabilizer_" + kind
        old = hull.find(f"{kind}[@name='{name}']")
        if old is not None:
            hull.remove(old)
        element = E.SubElement(hull, kind, name=name)
        geo = E.SubElement(E.SubElement(element, "geometry"), "mesh")
        E.SubElement(geo, "uri").text = str(mesh.resolve())
        if kind == "visual":
            material = E.SubElement(element, "material")
            E.SubElement(material, "diffuse").text = ".28 .30 .32 1"
    old = model.find("plugin[@name='usv::PassiveStabilizer']")
    if old is not None:
        model.remove(old)
    plugin = E.SubElement(
        model,
        "plugin",
        name="usv::PassiveStabilizer",
        filename="libUsvAppendageHydrodynamics.so",
    )
    for key, value in {
        "link_name": "hull",
        "density": 1000,
        "water_level": 0,
        "topic_namespace": "usv",
        "telemetry_group": "stabilizer",
        "buoyancy_triangles_file": str(triangles.resolve()),
    }.items():
        E.SubElement(plugin, key).text = str(value)
    sf = E.SubElement(plugin, "surface", name="passive_stabilizer")
    E.SubElement(sf, "cd0").text = str(
        part["hydrodynamics"]["profile_drag_coefficient"]
    )
    E.SubElement(sf, "efficiency").text = str(
        part["hydrodynamics"]["oswald_efficiency"]
    )
    for x, z in outline:
        E.SubElement(sf, "point").text = f"{x:.12g} 0 {z:.12g}"
    field = E.SubElement(plugin, "wavefield")
    E.SubElement(field, "topic").text = "/vrx/wavefield/parameters"
    wave = E.SubElement(field, "wave")
    E.SubElement(wave, "gain").text = "0"
    E.SubElement(wave, "steepness").text = "0"
