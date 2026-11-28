"""生成裸船壳的保形 CFD 预检网格；不求解流场、不改 Gazebo 参数。

依赖 Gmsh 4.15.0、Trimesh、NumPy、SciPy。输入是已置于静态吃水的封闭
STL，单位米；z=0 为静水面。外域固定为 x±24、y±12、z[-12,6] m。
共面邻接三角形合并为平面多边形，避免人工内部对角线限制体网格质量。
生成完成不代表网格合格，仍须转换后执行 OpenFOAM 完整质量检查。
"""

import argparse
import hashlib
import json
from pathlib import Path
import re

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
import trimesh


def foam_list(path):
    """读取明确为 ASCII 的 OpenFOAM 列表，兼容紧凑标量列表。

    Args:
        path: 包含 FoamFile 头与一个主列表的文件 Path。

    Returns:
        声明条目数和去掉外层括号的文本；调用方校验具体元素数量。

    Raises:
        ValueError: 非 ASCII、缺少列表或格式不完整。
    """
    text = path.read_text()
    match = re.search(r"\n(\d+)\s*\(", text)
    if "format      ascii;" not in text and "format ascii;" not in text:
        raise ValueError(f"需要 ASCII 列表：{path}")
    if match is None or text.rfind(")") < match.end():
        raise ValueError(f"无完整列表：{path}")
    return int(match[1]), text[match.end() : text.rfind(")")]


def boundary_loop(triangles):
    """提取一致定向三角面集合的单个外环，拒绝孔洞或分叉。

    Args:
        triangles: 顶点索引数组，形状 N×3，共享边必须反向。

    Returns:
        沿原面法向定向的多边形顶点索引列表。

    Raises:
        ValueError: 集合没有唯一、闭合且无分叉的边界。
    """
    edges = set()
    for face in triangles:
        for a, b in zip(face, (*face[1:], face[0])):
            if (b, a) in edges:
                edges.remove((b, a))
            elif (a, b) in edges:
                raise ValueError("共享边定向不一致")
            else:
                edges.add((a, b))
    successors = dict(edges)
    if not edges or len(successors) != len(edges):
        raise ValueError("边界为空或存在分叉")
    start = min(successors)
    loop = [start]
    current = successors[start]
    while current != start:
        if current in loop or current not in successors:
            raise ValueError("边界未闭合")
        loop.append(current)
        current = successors[current]
    if len(loop) != len(edges):
        raise ValueError("多环或孔洞不能合并为单面")
    return loop


def planar_polygons(mesh):
    """合并共面邻面，并用独立点到面距离检查链式合并的误差。

    Args:
        mesh: 有限坐标、封闭、法向一致、向外定向的 Trimesh 船壳。

    Returns:
        多边形索引列表；合并面所有顶点距原参考平面不超过 1 微米。

    Raises:
        ValueError: 输入不封闭、退化、反向或合并面不共面。
    """
    if (
        not np.isfinite(mesh.vertices).all()
        or not mesh.is_watertight
        or not mesh.is_winding_consistent
        or mesh.volume <= 0
        or np.min(mesh.area_faces) <= 0
    ):
        raise ValueError("需要封闭、有限、正体积且法向一致的非退化船壳")
    adjacency = mesh.face_adjacency
    normals = mesh.face_normals
    alignment = np.einsum(
        "ij,ij->i", normals[adjacency[:, 0]], normals[adjacency[:, 1]]
    )
    pairs = adjacency[alignment > 1 - 1e-12]
    graph = coo_matrix(
        (np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])),
        shape=(len(mesh.faces), len(mesh.faces)),
    )
    _, labels = connected_components(graph, directed=False)
    polygons = []
    for group in np.unique(labels):
        indices = np.flatnonzero(labels == group)
        vertices = mesh.vertices[np.unique(mesh.faces[indices])]
        distances = (vertices - mesh.triangles[indices[0], 0]) @ normals[indices[0]]
        if np.max(np.abs(distances)) > 1e-6:
            raise ValueError("合并面偏离原平面超过 1 微米")
        polygons.append(boundary_loop(mesh.faces[indices]))
    return polygons


def add_surfaces(geometry, vertices, polygons, size):
    """通过共享边建立平面壳，不移动输入顶点。

    Args:
        geometry: 已初始化的 Gmsh 内置几何 API。
        vertices: 顶点坐标，m，形状 N×3。
        polygons: 一致定向的面顶点索引列表。
        size: 顶点处网格目标尺寸，m，正数。

    Returns:
        Gmsh 平面标签列表。
    """
    points = [geometry.addPoint(*v, size) for v in vertices]
    edges = {}
    surfaces = []
    for face in polygons:
        lines = []
        for a, b in zip(face, (*face[1:], face[0])):
            key = tuple(sorted((int(a), int(b))))
            if key not in edges:
                edges[key] = geometry.addLine(points[key[0]], points[key[1]])
            lines.append(edges[key] * (1 if a < b else -1))
        surfaces.append(geometry.addPlaneSurface([geometry.addCurveLoop(lines)]))
    return surfaces


def waterline_domain(gmsh, mesh, polygons, scale):
    """建立被静水面分开的两个流体体积，保持内部水面共形。

    Args:
        gmsh: 已初始化并创建模型的 Gmsh API。
        mesh: 已通过封闭性检查的船壳，坐标单位 m。
        polygons: 共面合并后的定向多边形索引。
        scale: 正的目标网格尺寸比例。

    Returns:
        两个流体体积标签，以及船壳和六个外边界的面标签字典。

    Raises:
        ValueError: 船壳不跨静水面，或分区/边界不满足预期。
    """
    if not mesh.bounds[0, 2] < 0 < mesh.bounds[1, 2]:
        raise ValueError("水线分区要求船壳跨过 z=0")
    geometry = gmsh.model.occ
    hull = add_surfaces(geometry, mesh.vertices, polygons, 0.125 * scale)
    ship = geometry.addVolume([geometry.addSurfaceLoop(hull, sewing=True)])
    box = geometry.addBox(-24, -12, -12, 48, 24, 18)
    fluid, _ = geometry.cut([(3, box)], [(3, ship)])
    plane = geometry.addRectangle(-24, -12, 0, 48, 24)
    parts, _ = geometry.fragment(fluid, [(2, plane)])
    geometry.synchronize()
    volumes = [tag for dimension, tag in parts if dimension == 3]
    if len(volumes) != 2:
        raise ValueError("静水面没有生成恰好两个流体体积")
    boundaries = {
        name: [] for name in ("hull", "xmin", "xmax", "ymin", "ymax", "bottom", "top")
    }
    planes = [
        (0, -24, "xmin"),
        (0, 24, "xmax"),
        (1, -12, "ymin"),
        (1, 12, "ymax"),
        (2, -12, "bottom"),
        (2, 6, "top"),
    ]
    for dimension, tag in gmsh.model.getEntities(2):
        adjacent, _ = gmsh.model.getAdjacencies(dimension, tag)
        if len(adjacent) != 1:
            continue  # 两侧均为流体的水面保持内部面，不能变成壁面。
        bounds = gmsh.model.getBoundingBox(dimension, tag)
        name = "hull"
        for axis, position, label in planes:
            if (
                max(abs(bounds[axis] - position), abs(bounds[axis + 3] - position))
                < 1e-5
            ):
                name = label
                break
        boundaries[name].append(tag)
    if not all(boundaries.values()):
        raise ValueError("外边界或船壳标记缺失")
    for dimension, tag in gmsh.model.getEntities(0):
        x, y, z = gmsh.model.getValue(dimension, tag, [])
        near_hull = abs(x) < 3.01 and abs(y) < 1.2 and abs(z) < 1
        gmsh.model.mesh.setSize(
            [(dimension, tag)], (0.125 if near_hull else 0.75) * scale
        )
    return volumes, boundaries


def add_waterline_refinement(gmsh, background, scale):
    """在本船艏部折角与静水面交点两侧对称加密，不改变实体。

    Args:
        gmsh: 已初始化的 Gmsh API。
        background: 原自由面区域的网格尺寸场标签。
        scale: 正的目标尺寸比例；区域范围不随网格细化缩放。

    Returns:
        与原背景场取最小值的组合尺寸场标签。
    """
    fields = [background]
    for sign in (-1, 1):
        field = gmsh.model.mesh.field.add("Box")
        settings = {
            "VIn": 0.01 * scale,
            "VOut": 0.75 * scale,
            "XMin": 1.85,
            "XMax": 2.35,
            "YMin": min(sign * 0.4, sign * 0.7),
            "YMax": max(sign * 0.4, sign * 0.7),
            "ZMin": -0.07,
            "ZMax": 0.07,
            "Thickness": 0.05,
        }
        for name, value in settings.items():
            gmsh.model.mesh.field.setNumber(field, name, value)
        fields.append(field)
    combined = gmsh.model.mesh.field.add("Min")
    gmsh.model.mesh.field.setNumbers(combined, "FieldsList", fields)
    return combined


def unpartitioned_domain(gmsh, mesh, polygons, scale):
    """建立不按静水面分区的原几何网格路径，用于保留对照。

    Args:
        gmsh: 已初始化并创建模型的 Gmsh API。
        mesh: 封闭船壳，坐标单位 m。
        polygons: 共面合并后的定向多边形索引。
        scale: 正的目标网格尺寸比例。

    Returns:
        流体体积标签列表及所有外边界面标签字典。
    """
    geometry = gmsh.model.geo
    hull = add_surfaces(geometry, mesh.vertices, polygons, 0.125 * scale)
    vertices = [
        [-24, -12, -12],
        [24, -12, -12],
        [24, 12, -12],
        [-24, 12, -12],
        [-24, -12, 6],
        [24, -12, 6],
        [24, 12, 6],
        [-24, 12, 6],
    ]
    faces = {
        "xmin": [0, 4, 7, 3],
        "xmax": [1, 2, 6, 5],
        "ymin": [0, 1, 5, 4],
        "ymax": [3, 7, 6, 2],
        "bottom": [0, 3, 2, 1],
        "top": [4, 5, 6, 7],
    }
    outer = add_surfaces(geometry, vertices, faces.values(), 0.75 * scale)
    volume = geometry.addVolume(
        [geometry.addSurfaceLoop(outer), geometry.addSurfaceLoop(hull)]
    )
    geometry.synchronize()
    return [volume], {"hull": hull, **{n: [s] for n, s in zip(faces, outer)}}


def generate_mesh(source, output, scale, waterline=False):
    """在新目录生成网格及来源清单，失败时保留诊断目录但不写完成清单。

    Args:
        source: 输入 STL 的 Path；已经施加静态吃水变换。
        output: 不得存在的输出目录 Path。
        scale: 全部目标网格尺寸的正比例因子；1 为本轮粗网格。
        waterline: 是否在 z=0 做内部共形分区并加密本船水线交点。

    Raises:
        ValueError: 比例、输入船壳或外域约束不满足。
        FileExistsError: 输出目录已存在，拒绝覆盖证据。
    """
    import gmsh  # 可选依赖：普通工程单测不需要安装 CFD 工具。

    if not np.isfinite(scale) or scale <= 0:
        raise ValueError("网格比例必须为有限正数")
    mesh = trimesh.load(source, process=True, force="mesh")
    polygons = planar_polygons(mesh)
    lower, upper = mesh.bounds
    if np.any(lower <= [-24, -12, -12]) or np.any(upper >= [24, 12, 6]):
        raise ValueError("船壳必须完全位于外域内部")
    tool_digest = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    output.mkdir(parents=True, exist_ok=False)
    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.NumThreads", 4)
        gmsh.model.add("bare-hull")
        if waterline:
            volumes, boundaries = waterline_domain(gmsh, mesh, polygons, scale)
        else:
            volumes, boundaries = unpartitioned_domain(gmsh, mesh, polygons, scale)
        gmsh.model.addPhysicalGroup(3, volumes, name="fluid")
        for name, surfaces in boundaries.items():
            gmsh.model.addPhysicalGroup(2, surfaces, name=name)
        field = gmsh.model.mesh.field.add("Box")
        settings = {
            "VIn": 0.25 * scale,
            "VOut": 0.75 * scale,
            "XMin": -9,
            "XMax": 9,
            "YMin": -3,
            "YMax": 3,
            "ZMin": -0.5,
            "ZMax": 0.5,
            "Thickness": 0.5,
        }
        for name, value in settings.items():
            gmsh.model.mesh.field.setNumber(field, name, value)
        if waterline:
            field = add_waterline_refinement(gmsh, field, scale)
        gmsh.model.mesh.field.setAsBackgroundMesh(field)
        gmsh.option.setNumber("Mesh.MshFileVersion", 2.2)
        gmsh.option.setNumber("Mesh.OptimizeNetgen", 1)
        gmsh.option.setNumber("Mesh.MeshSizeMax", 0.75 * scale)
        gmsh.model.mesh.generate(3)
        if waterline:
            gmsh.option.setNumber("Mesh.OptimizeThreshold", 0.7)
            gmsh.model.mesh.optimize("", niter=10)
            gmsh.model.mesh.optimize("Relocate3D", niter=10)
            gmsh.model.mesh.optimize("Netgen", niter=2)
        gmsh.write(str(output / "mesh.msh"))
        if hashlib.sha256(Path(__file__).read_bytes()).hexdigest() != tool_digest:
            raise RuntimeError("网格生成期间工具源码变化，拒绝写入完成清单")
        manifest = {
            "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
            "tool_sha256": tool_digest,
            "gmsh_version": gmsh.__version__,
            "scale": scale,
            "waterline_partition": waterline,
            "input_triangles": len(mesh.faces),
            "planar_surfaces": len(polygons),
            "input_volume_m3": float(mesh.volume),
            "mesh_sha256": hashlib.sha256(
                (output / "mesh.msh").read_bytes()
            ).hexdigest(),
            "quality_status": "requires_openfoam_checkMesh",
        }
        (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    finally:
        gmsh.finalize()


def cell_vertices(cell, faces, owner, neighbor):
    """从面邻接获取四面体顶点，拒绝非四面体。

    Args:
        cell: 非负单元索引。
        faces: 三角面顶点索引数组。
        owner: 全部面的 owner 单元索引。
        neighbor: 内部面的 neighbor 索引，与 faces 前段对齐。

    Returns:
        四个不同的顶点索引。
    """
    indices = np.r_[np.flatnonzero(owner == cell), np.flatnonzero(neighbor == cell)]
    vertices = np.unique(faces[indices])
    if len(indices) != 4 or len(vertices) != 4:
        raise ValueError(f"单元 {cell} 不是四面体")
    return vertices


def select_sliver_faces(case):
    """选择同相凸单元对并写 faceSet，不执行合并或宣称网格通过。

    必须先在独立副本运行完整 checkMesh，使用当前 underdeterminedCells。
    后续需运行 removeFaces 并重新完整检查，不能将选择成功当成质量合格。

    Args:
        case: 含 ASCII polyMesh 与本轮坏单元集合的 case Path。

    Raises:
        ValueError: 格式不合法，或找不到保体积、凸且不跨水面的邻居。
        FileExistsError: 已有选择证据，拒绝覆盖。
    """
    from scipy.spatial import ConvexHull

    mesh = case / "constant/polyMesh"
    destination = mesh / "sets/sliverInterfaces"
    report = case / "merge-evidence.json"
    if destination.exists() or report.exists():
        raise FileExistsError("已有选择证据，请使用新的网格副本")
    count, text = foam_list(mesh / "points")
    points = np.fromstring(text.replace("(", " ").replace(")", " "), sep=" ").reshape(
        -1, 3
    )
    if len(points) != count or not np.isfinite(points).all():
        raise ValueError("点列表无效")
    count, text = foam_list(mesh / "faces")
    faces = np.array(
        [list(map(int, value.split())) for value in re.findall(r"3\((.*?)\)", text)]
    )
    if len(faces) != count or faces.min() < 0 or faces.max() >= len(points):
        raise ValueError("仅支持合法三角面列表")
    labels = {}
    for name in ("owner", "neighbour", "sets/underdeterminedCells"):
        count, text = foam_list(mesh / name)
        values = np.fromstring(text, sep=" ", dtype=int)
        if len(values) != count or np.any(values < 0):
            raise ValueError(f"索引列表无效：{name}")
        labels[name] = values
    owner = labels["owner"]
    neighbor = labels["neighbour"]
    bad = labels["sets/underdeterminedCells"]
    if len(owner) != len(faces) or len(neighbor) > len(owner):
        raise ValueError("单元面关联长度错误")
    selected, records = [], []
    used = set()
    for cell in bad:
        vertices = cell_vertices(cell, faces, owner, neighbor)
        volume = ConvexHull(points[vertices]).volume
        candidates = []
        adjacent = np.r_[
            np.flatnonzero(owner[: len(neighbor)] == cell),
            np.flatnonzero(neighbor == cell),
        ]
        for face in adjacent:
            other = neighbor[face] if owner[face] == cell else owner[face]
            if other in used or other in bad:
                continue
            other_vertices = cell_vertices(other, faces, owner, neighbor)
            union = points[np.union1d(vertices, other_vertices)]
            if union[:, 2].min() < -1e-10 and union[:, 2].max() > 1e-10:
                continue
            other_volume = ConvexHull(points[other_vertices]).volume
            combined = ConvexHull(union).volume
            error = abs(combined - volume - other_volume)
            if error <= max(1e-12, 1e-8 * (volume + other_volume)):
                candidates.append(
                    (other_volume, int(face), int(other), combined, error)
                )
        if not candidates:
            raise ValueError(f"单元 {cell} 没有满足同相凸体积条件的邻居")
        _, face, other, combined, error = max(candidates)
        selected.append(face)
        used.update((cell, other))
        records.append(
            {
                "bad_cell": int(cell),
                "neighbor": other,
                "removed_face": face,
                "combined_volume_m3": combined,
                "convex_volume_error_m3": error,
            }
        )
    body = "\n".join(map(str, selected))
    destination.write_text(
        "FoamFile {version 2.0; format ascii; class faceSet; object sliverInterfaces;}\n"
        + f"{len(selected)}\n(\n{body}\n)\n"
    )
    report.write_text(json.dumps(records, indent=2) + "\n")


def main():
    """解析输入路径及尺寸比例；只生成独立 CFD 网格副本。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hull", type=Path, help="静态吃水船壳 STL，m")
    parser.add_argument("--output", type=Path, help="新的证据目录")
    parser.add_argument("--scale", type=float, default=1, help="网格尺寸比例，默认 1")
    parser.add_argument(
        "--waterline", action="store_true", help="按静水面分区并加密水线交点"
    )
    parser.add_argument(
        "--select-slivers", type=Path, help="为已检查的 case 选择同相凸单元对"
    )
    args = parser.parse_args()
    if args.select_slivers:
        if args.hull or args.output or args.waterline:
            parser.error("选择合并面不能同时生成网格")
        select_sliver_faces(args.select_slivers)
        return
    if args.hull is None or args.output is None:
        parser.error("生成网格需要 --hull 和 --output")
    generate_mesh(args.hull, args.output, args.scale, args.waterline)


if __name__ == "__main__":
    main()
