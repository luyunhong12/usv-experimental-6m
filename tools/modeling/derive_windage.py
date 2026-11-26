"""从静态载荷下的可视几何估算水上风压投影，遮挡重叠只计一次。

像素并集用于几何面积与形心，不是CFD压力求解。法向平板Cd=1.28为
可追溯的工程假设，不能视为本船风洞标定。默认不修改输入模型。
"""

import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw
from scipy.spatial.transform import Rotation
import trimesh


def pose_matrix(text):
    """把SDF的xyz/rpy位姿转换为齐次矩阵。

    Args:
        text: 六个空格分隔的米/弧度数值，缺省为单位位姿。
    """
    pose = np.fromstring(text or "0 0 0 0 0 0", sep=" ")
    if pose.size != 6 or not np.isfinite(pose).all():
        raise ValueError("Invalid geometry pose")
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler("xyz", pose[3:]).as_matrix()
    matrix[:3, 3] = pose[:3]
    return matrix


def visual_triangles(model):
    """读取模型各连杆的可视三角形及来源哈希，保留当前遮挡几何。

    Args:
        model: 已解析的SDF model元素；网格URI必须为实际文件路径。
    """
    pieces, sources = [], {}
    for link in model.findall("link"):
        for visual in link.findall("visual"):
            geometry = visual.find("geometry")
            mesh_element = geometry.find("mesh")
            if mesh_element is not None:
                path = Path(mesh_element.findtext("uri"))
                mesh = trimesh.load(path, force="mesh", process=False)
                scale = np.fromstring(mesh_element.findtext("scale", "1 1 1"), sep=" ")
                if (
                    scale.size != 3
                    or not np.isfinite(scale).all()
                    or np.any(scale <= 0)
                ):
                    raise ValueError("Invalid visual mesh scale")
                vertices = mesh.vertices * scale
                sources[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
            elif geometry.find("box") is not None:
                mesh = trimesh.creation.box(
                    np.fromstring(geometry.findtext("box/size"), sep=" ")
                )
                vertices = mesh.vertices
            else:
                raise ValueError("Unsupported visual geometry; do not silently omit it")
            transform = pose_matrix(link.findtext("pose")) @ pose_matrix(
                visual.findtext("pose")
            )
            vertices = vertices @ transform[:3, :3].T + transform[:3, 3]
            pieces.append(vertices[mesh.faces])
    if not pieces:
        raise ValueError("No visual geometry")
    return np.concatenate(pieces), sources


def clip_air_polygon(triangle, height):
    """裁切单个跨水面三角形，返回按原顺序的水上多边形。

    Args:
        triangle: 三个船体坐标顶点，m。
        height: 各顶点高于水面的有符号高度，m。
    """
    polygon = []
    for index in range(3):
        following = (index + 1) % 3
        if height[index] >= 0:
            polygon.append(triangle[index])
        if (height[index] >= 0) != (height[following] >= 0):
            fraction = height[index] / (height[index] - height[following])
            polygon.append(
                triangle[index] + fraction * (triangle[following] - triangle[index])
            )
    return np.asarray(polygon)


def exposed_polygons(triangles, equilibrium):
    """按给定静态高度与倾角裁切水上几何，坐标仍保留在船体轴。

    Args:
        triangles: 船体坐标下N×3×3三角网格，m。
        equilibrium: 静态平衡的高度z(m)、横倾roll(rad)、纵倾pitch(rad)。
    """
    z, roll, pitch = equilibrium
    rotation = Rotation.from_euler("xyz", [roll, pitch, 0]).as_matrix()
    heights = triangles @ rotation[2] + z
    above = np.all(heights >= 0, axis=1)
    crossing = ~above & np.any(heights > 0, axis=1)
    polygons = list(triangles[above])
    polygons.extend(
        clip_air_polygon(triangle, height)
        for triangle, height in zip(triangles[crossing], heights[crossing])
    )
    if not polygons:
        raise ValueError("No geometry above water")
    return polygons


def projected_union(polygons, axis, spacing):
    """计算轴向投影的像素并集面积与形心，重叠不重复计数。

    Args:
        polygons: 水上船体坐标多边形，m。
        axis: 来流轴索引0或1；该轴上的形心分量取0，不影响力矩。
        spacing: 正的投影网格间距，m；需通过多分辨率收敛检查。
    """
    if axis not in (0, 1) or not np.isfinite(spacing) or spacing <= 0:
        raise ValueError("Invalid projection axis or spacing")
    dimensions = [index for index in range(3) if index != axis]
    points = np.concatenate(polygons)[:, dimensions]
    origin = np.floor(points.min(axis=0) / spacing) * spacing
    shape = np.ceil((points.max(axis=0) - origin) / spacing).astype(int) + 2
    if np.prod(shape) > 100_000_000:
        raise ValueError("Projection grid exceeds memory bound")
    mask = Image.new("1", tuple(shape), 0)
    draw = ImageDraw.Draw(mask)
    for polygon in polygons:
        projected = (polygon[:, dimensions] - origin) / spacing
        draw.polygon([tuple(point) for point in np.rint(projected).astype(int)], fill=1)
    row, column = np.nonzero(np.asarray(mask))
    if len(row) == 0:
        raise ValueError("Empty wind projection")
    center = np.zeros(3)
    center[dimensions] = origin + spacing * np.array([column.mean(), row.mean()])
    return {"area_m2": len(row) * spacing**2, "center_m": center.tolist()}


def derive(model_path, equilibrium_path, spacing):
    """返回输入模型的两个水上投影及固定法向平板系数假设。

    Args:
        model_path: 候选模型SDF实际路径。
        equilibrium_path: 含equilibrium_z_roll_pitch的静平衡JSON路径。
        spacing: 投影网格间距，m。
    """
    triangles, sources = visual_triangles(ET.parse(model_path).find("model"))
    equilibrium = json.loads(Path(equilibrium_path).read_text())[
        "equilibrium_z_roll_pitch"
    ]
    polygons = exposed_polygons(triangles, equilibrium)
    projections = [projected_union(polygons, axis, spacing) for axis in (0, 1)]
    return {
        "status": "GEOMETRY_UNION_FLAT_PLATE_APPROXIMATION_NOT_WIND_TUNNEL_CALIBRATED",
        "spacing_m": spacing,
        "normal_plate_cd": 1.28,
        "source": "https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/shape-effects-on-drag-2/",
        "model_sha256": hashlib.sha256(Path(model_path).read_bytes()).hexdigest(),
        "mesh_sha256": sources,
        "equilibrium_z_roll_pitch": equilibrium,
        "projections": projections,
        "air_cd_area_m2": [1.28 * item["area_m2"] for item in projections] + [0],
    }


def main():
    """输出独立风压几何数据，不覆盖输入模型或已有结果。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--equilibrium", type=Path, required=True)
    parser.add_argument("--spacing", type=float, default=0.005)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = derive(args.model, args.equilibrium, args.spacing)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write("\n")


if __name__ == "__main__":
    main()
