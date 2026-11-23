"""在现有船体上添加工程设备包络与安装支架。

Engineering device envelopes and brackets attached to the existing hull.
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
import xml.etree.ElementTree as E
import math


def add_mounts(hull, cfg):
    """在现有船体上添加设备包络与支架的视觉和碰撞几何。

    Args:
        hull: 目标船体 link 的 XML 元素，将原地添加子节点。
        cfg: 包含传感器姿态与高级传感器开关的载荷配置。
    """

    def part(name, pose, size, color):
        """添加同形的显示与碰撞长方体，仅显示部分设置材质。

        Args:
            name: 部件名称前缀。
            pose: 位置 m 和欧拉角 rad 组成的六元姿态。
            size: 长宽高，单位 m。
            color: RGBA 颜色字符串。
        """
        for kind in ["visual", "collision"]:
            e = E.SubElement(hull, kind, name=name + "_" + kind)
            E.SubElement(e, "pose").text = " ".join(map(str, pose))
            b = E.SubElement(E.SubElement(e, "geometry"), "box")
            E.SubElement(b, "size").text = " ".join(map(str, size))
            if kind == "visual":
                m = E.SubElement(e, "material")
                E.SubElement(m, "diffuse").text = color
                p = E.SubElement(E.SubElement(m, "pbr"), "metal")
                E.SubElement(p, "metalness").text = ".2"
                E.SubElement(p, "roughness").text = ".55"

    cameras = (
        ["camera", "camera_right", "camera_side"]
        if cfg["advanced_sensors"]
        else ["camera"]
    )
    for name in cameras:
        x, y, z, r, p, yaw = cfg["poses"][name]
        part(
            name + "_housing",
            [x - 0.065 * math.cos(yaw), y - 0.065 * math.sin(yaw), z, r, p, yaw],
            [0.10, 0.075, 0.065],
            ".12 .14 .16 1",
        )
        part(
            name + "_bracket",
            [
                x - 0.065 * math.cos(yaw),
                y - 0.065 * math.sin(yaw),
                (z + 0.99) / 2,
                0,
                0,
                yaw,
            ],
            [0.035, 0.035, z - 0.99],
            ".5 .52 .55 1",
        )
    x, y, z, *_ = cfg["poses"]["lidar"]
    part(
        "lidar_mast",
        [x, y, (z - 0.08 + 0.99) / 2, 0, 0, 0],
        [0.045, 0.045, z - 0.08 - 0.99],
        ".5 .52 .55 1",
    )
    part("lidar_housing", [x, y, z, 0, 0, 0], [0.10, 0.10, 0.09], ".1 .12 .14 1")


def add_ledger_solids(hull, parts):
    """从质量台账生成额外固定实体的外形与碰撞，不重复增加惯性。

    Args:
        hull: 已通过台账合并惯性的船体 SDF link。
        parts: 当前方案的台账条目，仅处理带显式 geometry 的 hull 部件。
    """
    for part in parts:
        geometry = part.get("geometry")
        if part["link"] != "hull" or geometry is None:
            continue
        if geometry["type"] == "elliptic_fin":
            continue  # 同源网格与水动力由 passive_stabilizer 生成。
        if geometry["type"] != "box":
            raise ValueError("Unsupported ledger solid")
        for kind in ["visual", "collision"]:
            name = part["id"] + "_" + kind
            previous = hull.find(f"{kind}[@name='{name}']")
            if previous is not None:
                hull.remove(previous)
            element = E.SubElement(hull, kind, name=name)
            E.SubElement(element, "pose").text = " ".join(
                map(str, [*part["cg_m"], 0, 0, 0])
            )
            box = E.SubElement(E.SubElement(element, "geometry"), "box")
            E.SubElement(box, "size").text = " ".join(map(str, geometry["size_m"]))
            if kind == "visual":
                material = E.SubElement(element, "material")
                E.SubElement(material, "diffuse").text = ".28 .30 .32 1"
