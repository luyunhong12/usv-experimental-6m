"""从权威台账汇总干质量和辅助功率预算。

Authoritative dry-component aggregation and auxiliary electrical load budget.
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
import json
import math
import numpy as np
from mass_properties import aggregate

ROOT = WORKSPACE
CFG = ROOT / "src/usv_description/config"


def load():
    """读取当前权威设备质量与功耗台账。"""
    return json.loads((CFG / "component_mass_ledger.json").read_text())


def rows(profile="baseline", mass_case="nominal"):
    """选择方案和质量工况，返回已应用方案缩放的独立条目。

    Args:
        profile: baseline 或 speed15_concept 方案名称。
        mass_case: nominal、light 或 heavy 质量敏感性工况。
    """
    if profile not in ["baseline", "speed15_concept"] or mass_case not in [
        "nominal",
        "light",
        "heavy",
    ]:
        raise ValueError("Unknown ledger scenario")
    data = load()
    items = []
    ids = set()
    scale = json.loads((CFG / "profiles/speed15_concept.json").read_text())[
        "propeller_geometry_scale"
    ]
    origins = json.loads((ROOT / "../datasets/reference/boat_export.json").read_text())[
        "origins"
    ]
    for row in data["parts"]:
        if row["id"] in ids:
            raise ValueError("Duplicate component id")
        ids.add(row["id"])
        if (
            row["link"] not in ["hull", "azimuth", "propeller"]
            or not row["profiles"]
            or not set(row["profiles"]) <= set(["baseline", "speed15_concept"])
        ):
            raise ValueError("Unknown link/profile in ledger")
        if row["power"]["class"] not in [
            "passive",
            "auxiliary",
            "energy_storage",
            "propulsion_dynamic",
            "steering_dynamic",
            "generator_output_dynamic",
            "starter_storage_dynamic",
            "energy_conversion_dynamic",
        ]:
            raise ValueError("Unknown power accounting class")
        if profile not in row["profiles"]:
            continue
        r = dict(row)
        nominal = r["mass_kg"]
        lo, hi = r["mass_range_kg"]
        if not 0 < lo <= nominal <= hi:
            raise ValueError("Invalid uncertainty range")
        r["mass_kg"] = {"nominal": nominal, "light": lo, "heavy": hi}[mass_case]
        inertia_data = np.array(r["inertia_at_cg_kg_m2"]) * r["mass_kg"] / nominal
        if profile == "speed15_concept" and r["link"] == "propeller":
            r["mass_kg"] *= scale**3
            inertia_data *= scale**5
            r["cg_m"] = (
                np.array(origins["propeller"])
                + scale * (np.array(r["cg_m"]) - origins["propeller"])
            ).tolist()
        r["inertia_at_cg_kg_m2"] = inertia_data.tolist()
        items.append(r)
    return items


def dry_links(profile="baseline", mass_case="nominal"):
    """按所属刚体汇总不含燃油的设备质量属性。

    Args:
        profile: 设备台账适用的方案名称。
        mass_case: 名义、轻或重质量工况。
    """
    items = rows(profile, mass_case)
    out = {}
    for link in ["hull", "azimuth", "propeller"]:
        p = aggregate([r for r in items if r["link"] == link])
        out[link] = {
            "mass_kg": p["mass_kg"],
            "cg_m": p["cg_m"],
            "inertia_kg_m2": p["inertia_at_cg_kg_m2"],
        }
    return out


def auxiliary_w(mode="normal", profile="baseline"):
    """按工作模式累加辅助负载预算，避免重复计入动态功率。

    Args:
        mode: normal、survey 或 peak 负载模式。
        profile: 设备台账适用的方案名称。
    """
    total = 0.0
    for r in rows(profile):
        p = r["power"]
        if p["class"] != "auxiliary":
            continue
        idle, rated, duty = p["idle_w"], p["rated_w"], p["duty"][mode]
        if not 0 <= idle <= rated or not 0 <= duty <= 1:
            raise ValueError("Invalid electrical duty")
        total += idle + (rated - idle) * duty
    if not math.isfinite(total):
        raise ValueError("Nonfinite auxiliary budget")
    return total
