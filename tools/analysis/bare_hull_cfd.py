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

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
import trimesh


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


def generate_mesh(source, output, scale):
    """在新目录生成网格及来源清单，失败时保留诊断目录但不写完成清单。

    Args:
        source: 输入 STL 的 Path；已经施加静态吃水变换。
        output: 不得存在的输出目录 Path。
        scale: 全部目标网格尺寸的正比例因子；1 为本轮粗网格。

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
    output.mkdir(parents=True, exist_ok=False)
    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.NumThreads", 4)
        gmsh.model.add("bare-hull")
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
        gmsh.model.addPhysicalGroup(3, [volume], name="fluid")
        gmsh.model.addPhysicalGroup(2, hull, name="hull")
        for name, surface in zip(faces, outer):
            gmsh.model.addPhysicalGroup(2, [surface], name=name)
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
        gmsh.model.mesh.field.setAsBackgroundMesh(field)
        gmsh.option.setNumber("Mesh.MshFileVersion", 2.2)
        gmsh.option.setNumber("Mesh.OptimizeNetgen", 1)
        gmsh.option.setNumber("Mesh.MeshSizeMax", 0.75 * scale)
        gmsh.model.mesh.generate(3)
        gmsh.write(str(output / "mesh.msh"))
        manifest = {
            "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
            "tool_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "gmsh_version": gmsh.__version__,
            "scale": scale,
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


def main():
    """解析输入路径及尺寸比例；只生成独立 CFD 网格副本。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hull", type=Path, required=True, help="静态吃水船壳 STL，m")
    parser.add_argument("--output", type=Path, required=True, help="新的证据目录")
    parser.add_argument("--scale", type=float, default=1, help="网格尺寸比例，默认 1")
    args = parser.parse_args()
    generate_mesh(args.hull, args.output, args.scale)


if __name__ == "__main__":
    main()
