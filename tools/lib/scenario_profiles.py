"""应用显式方案、载荷与供电限制并更新场景属性。

Explicit hypothetical hardware changes, including their mass and supply limits.
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
import numpy as np
from scipy.optimize import root as solve_root
from scipy.spatial.transform import Rotation
from fuel_mass import combine, configuration, tank_properties
from fuel_equilibrium import displaced

ROOT = WORKSPACE


def read_inertia(link):
    """从 SDF 刚体读取质量、质心和对称惯性张量。

    Args:
        link: 含 inertial 节点的 SDF link 元素。
    """
    it = link.find("inertial")
    c = list(map(float, it.findtext("pose").split()))[:3]
    inertia_data = it.find("inertia")
    vals = {e.tag: float(e.text) for e in inertia_data}
    return dict(
        mass_kg=float(it.findtext("mass")),
        cg_m=c,
        inertia_kg_m2=[
            [vals["ixx"], vals["ixy"], vals["ixz"]],
            [vals["ixy"], vals["iyy"], vals["iyz"]],
            [vals["ixz"], vals["iyz"], vals["izz"]],
        ],
    )


def write_inertia(link, p):
    """将质量、质心和惯性写回指定 SDF 刚体。

    Args:
        link: 需要原地更新的 SDF link 元素。
        p: 含 mass_kg、cg_m 和 inertia_kg_m2 的质量属性映射。
    """
    it = link.find("inertial")
    it.find("mass").text = str(p["mass_kg"])
    it.find("pose").text = " ".join(map(str, [*p["cg_m"], 0, 0, 0]))
    inertia_data = np.array(p["inertia_kg_m2"])
    for key, i, j in [
        ("ixx", 0, 0),
        ("iyy", 1, 1),
        ("izz", 2, 2),
        ("ixy", 0, 1),
        ("ixz", 0, 2),
        ("iyz", 1, 2),
    ]:
        it.find("inertia/" + key).text = str(inertia_data[i, j])


def apply_speed15(
    model,
    payload_kg=0.0,
    payload_center=(0.0, 0.0, 0.6),
    mass_case="nominal",
    power_mode="normal",
    profile="speed15_concept",
):
    """应用显式方案与载荷配置，并重新求解静水平衡。

    Args:
        model: 需要原地更新的 SDF model 元素。
        payload_kg: 附加载荷质量，单位 kg；当前允许 0 到 300。
        payload_center: 载荷重心三维位置，单位 m，模型坐标系。
        mass_case: 设备质量敏感性工况名称。
        power_mode: 辅助设备功率模式。
        profile: 基准或概念升级方案名称。
    """
    import xml.etree.ElementTree as E

    cfg = json.loads(
        (ROOT / "src/usv_description/config/profiles/speed15_concept.json").read_text()
    )
    if (
        np.asarray(payload_center).shape != (3,)
        or not np.isfinite(payload_center).all()
    ):
        raise ValueError("Invalid payload centre")
    if not np.isfinite(payload_kg) or not 0 <= payload_kg <= 300:
        raise ValueError("Payload must be 0..300 kg")
    p = model.find("plugin[@name='usv::AzimuthPropulsion']")
    en = p.find("energy")
    if profile == "speed15_concept":
        from equipment_ledger import rows

        selected = rows(profile)
        base_count = sum("E163" in r["name"] for r in rows("baseline"))
        assert (
            sum("E163" in r["name"] for r in selected)
            == base_count * cfg["battery_pack_multiplier"]
        ), "Battery capacity/count mismatch"
        assert (
            abs(
                sum(
                    r["mass_kg"]
                    for r in selected
                    if r["id"].startswith("extra_battery_")
                )
                - cfg["extra_battery_mass_kg"]
            )
            < 1e-9
        ), "Extra battery mass mismatch"
        assert (
            abs(
                next(r["mass_kg"] for r in selected if r["id"] == "upgrade_motor")
                - cfg["extra_motor_mass_kg"]
            )
            < 1e-9
        ), "Motor increment mismatch"
        for k, v in [
            ("static_thrust", cfg["static_thrust_n"]),
            ("shaft_power", cfg["motor_electrical_w"] * cfg["motor_efficiency"]),
            ("advance_speed", cfg["advance_zero_thrust_m_s"]),
            ("reference_rpm", cfg["reference_rpm"]),
            ("stall_torque", cfg["stall_torque_nm"]),
        ]:
            p.find(k).text = str(v)
        en.find("motor_limit_w").text = str(cfg["motor_electrical_w"])
        factor = cfg["battery_pack_multiplier"]
        for k in [
            "capacity_ah",
            "max_discharge_a",
            "thermal_capacity_j_k",
            "thermal_conductance_w_k",
        ]:
            en.find(k).text = str(float(en.findtext(k)) * factor)
        en.find("resistance_ohm").text = str(
            float(en.findtext("resistance_ohm")) / factor
        )
    from equipment_ledger import dry_links, auxiliary_w, rows
    from payload_geometry import add_ledger_solids

    add_ledger_solids(model.find("link[@name='hull']"), rows(profile, mass_case))

    links = dry_links(profile, mass_case)
    tank = configuration()
    dry = links["hull"]
    en.find("aux_output_w").text = str(auxiliary_w(power_mode, profile))
    if payload_kg:
        size = np.array([1.0, 0.8, 0.5])
        dry = combine(
            dry,
            dict(
                mass_kg=payload_kg,
                cg_m=list(payload_center),
                inertia_kg_m2=np.diag(
                    payload_kg / 12 * (sum(size**2) - size**2)
                ).tolist(),
            ),
        )
    tank["dry_hull"] = dry
    fc = p.find("fuel_mass")
    fc.find("dry_mass").text = str(dry["mass_kg"])
    fc.find("dry_cg").text = " ".join(map(str, dry["cg_m"]))
    fc.find("dry_inertia").text = " ".join(
        map(str, np.array(dry["inertia_kg_m2"]).flat)
    )
    az = model.find("link[@name='azimuth']")
    prop = model.find("link[@name='propeller']")
    scale = cfg["propeller_geometry_scale"] if profile == "speed15_concept" else 1.0
    for link in [az, prop]:
        lp = dict(links[link.get("name")])
        lp["cg_m"] = (
            np.array(lp["cg_m"])
            - np.array(list(map(float, link.findtext("pose").split()))[:3])
        ).tolist()
        write_inertia(link, lp)
    for mesh in prop.findall(".//geometry/mesh"):
        E.SubElement(mesh, "scale").text = f"{scale} {scale} {scale}"
    p.find("radius").text = str(float(p.findtext("radius")) * scale)
    if profile == "speed15_concept":
        model.find("joint[@name='propeller_spin']/axis/limit/effort").text = str(
            cfg["stall_torque_nm"]
        )
    others = []
    for link in [az, prop]:
        x = read_inertia(link)
        x["cg_m"] = (
            np.array(x["cg_m"])
            + np.array(list(map(float, link.findtext("pose").split()))[:3])
        ).tolist()
        others.append(x)
    litres = float(en.findtext("initial_fuel_l"))
    fluid_density = float(
        model.find("plugin[@name='usv::HullHydrostatics']/density").text
    )

    def props(R):
        """在指定姿态下合并燃油、设备及载荷的质量属性。

        Args:
            R: 模型到世界的旋转矩阵。
        """
        hull = combine(
            dry, tank_properties(tank, litres, R.T @ np.array([0.0, 0.0, 1.0]))
        )
        ship = hull
        for other in others:
            ship = combine(ship, other)
        return hull, ship

    def residual(q):
        """计算当前方案静水平衡的体积与重心残差。

        Args:
            q: 待求的高度 m、滚转 rad 和俯仰 rad。
        """
        volume, cb, R = displaced(q)
        h, ship = props(R)
        cg = R @ np.array(ship["cg_m"]) + [0, 0, q[0]]
        return [volume - ship["mass_kg"] / fluid_density, cb[0] - cg[0], cb[1] - cg[1]]

    sol = solve_root(residual, [-0.28, 0.02, 0])
    assert sol.success and np.linalg.norm(residual(sol.x)) < 1e-8
    hull, ship = props(Rotation.from_euler("xyz", [*sol.x[1:], 0]).as_matrix())
    write_inertia(model.find("link[@name='hull']"), hull)
    return {
        "payload_kg": payload_kg,
        "payload_center_m": list(payload_center),
        "mass_case": mass_case,
        "power_mode": power_mode,
        "status": (
            cfg["status"]
            if profile == "speed15_concept"
            else "ENGINEERING_BASELINE_NOT_MEASURED"
        ),
        "pose_z_roll_pitch": sol.x.tolist(),
        "ship_mass_kg": ship["mass_kg"],
        "ship_cg_m": ship["cg_m"],
        "hardware": (
            cfg
            if profile == "speed15_concept"
            else {
                "profile": "baseline",
                "mass_layout_source": "component_mass_ledger.json",
            }
        ),
    }
