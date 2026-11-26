"""从旧船生成无稳定鳍、固定载荷的候选船，保留原模型供逐项对照。

只写指定的新输出目录；运行时不需要能源与油箱配置。暂不替换默认入口。
"""

import argparse
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools/lib"))
from equipment_ledger import rows  # noqa: E402
from mass_properties import aggregate  # noqa: E402


def subtract_part(mass, center, inertia, part):
    """用平行轴定理移除已知部件，返回剩余刚体属性。

    Args:
        mass: 原刚体质量，kg。
        center: 原刚体重心，模型FLU坐标，m。
        inertia: 原刚体重心处惯性，kg·m²。
        part: 同坐标系的台账部件，包含质量、重心及惯性。

    Returns:
        aggregate约定的质量、重心和惯性字典。

    Raises:
        ValueError: 剩余质量或惯性不满足物理条件。
    """
    removed_mass = part["mass_kg"]
    remaining = mass - removed_mass
    if remaining <= 0:
        raise ValueError("Removed part exceeds original mass")
    c = np.asarray(center)
    pc = np.asarray(part["cg_m"])
    new_center = (mass * c - removed_mass * pc) / remaining
    d = pc - c
    shift = removed_mass * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    d = new_center - c
    new_inertia = (
        np.asarray(inertia)
        - np.asarray(part["inertia_at_cg_kg_m2"])
        - shift
        - remaining * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    )
    result = {
        "mass_kg": remaining,
        "cg_m": new_center.tolist(),
        "inertia_at_cg_kg_m2": new_inertia.tolist(),
    }
    aggregate([result])
    return result


def prepare(destination):
    """生成可独立加载的候选模型、静水场景和来源清单。

    Args:
        destination: 不存在的输出目录；拒绝覆盖既有试验。

    Raises:
        FileExistsError: 输出目录已经存在。
        ValueError: 原模型惯性或稳定鳍条目无效。
    """
    destination = Path(destination).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    source = ROOT / "ros_ws/src/usv_description/models/usv_azimuth_trial.sdf"
    tree = ET.parse(source)
    model = tree.getroot().find("model")
    hull = model.find("link[@name='hull']")
    part = next(r for r in rows() if r["id"] == "passive_stabilizer")
    inertial = hull.find("inertial")
    old_mass = float(inertial.findtext("mass"))
    center = np.fromstring(inertial.findtext("pose"), sep=" ")[:3]
    tensor = inertial.find("inertia")
    matrix = np.zeros((3, 3))
    indices = [
        ("ixx", 0, 0),
        ("iyy", 1, 1),
        ("izz", 2, 2),
        ("ixy", 0, 1),
        ("ixz", 0, 2),
        ("iyz", 1, 2),
    ]
    for key, i, j in indices:
        matrix[i, j] = matrix[j, i] = float(tensor.findtext(key))
    fixed = subtract_part(old_mass, center, matrix, part)
    inertial.find("mass").text = str(fixed["mass_kg"])
    inertial.find("pose").text = " ".join(map(str, fixed["cg_m"] + [0, 0, 0]))
    for key, i, j in indices:
        tensor.find(key).text = str(fixed["inertia_at_cg_kg_m2"][i][j])
    for element in list(hull):
        if element.get("name", "").startswith("passive_stabilizer"):
            hull.remove(element)
    model.remove(model.find("plugin[@name='usv::PassiveStabilizer']"))
    propulsion = model.find("plugin[@name='usv::AzimuthPropulsion']")
    for key in ("energy", "fuel_mass"):
        propulsion.remove(propulsion.find(key))
    propulsion.append(ET.Comment("关闭能源和动态燃油惯性；使用上方固定载荷。"))
    ET.SubElement(propulsion, "energy_enabled").text = "false"
    ET.indent(tree, space="  ")
    tree.write(destination / "model.sdf", encoding="utf-8", xml_declaration=True)
    world_source = ROOT / "ros_ws/src/usv_simulation/worlds/usv_hull_trial.sdf"
    world = ET.parse(world_source)
    world.getroot().find("world").set("name", "usv_simple_trial")
    world.getroot().find("world/include/uri").text = str(destination / "model.sdf")
    world.write(destination / "world.sdf", encoding="utf-8", xml_declaration=True)
    result = {
        "status": "CANDIDATE_NOT_NAVIGATION_VALIDATED",
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "removed_mass_kg": part["mass_kg"],
        "fixed_hull": fixed,
        "load_basis": "Freeze original initial load; remove only added fin. No fuel consumption, battery depletion or liquid slosh.",
        "propulsion_basis": "Retain existing motor shaft power, torque, RPM lag and finite steering; energy supply is ideal, not propulsion capability.",
    }
    (destination / "manifest.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="新的候选模型输出目录")
    prepare(parser.parse_args().output)
