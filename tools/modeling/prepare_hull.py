"""从船体站位与质量配置生成封闭几何、静水平衡解和仿真场景。

Generate sealed station envelope, analytical static solution, SDF and VRX world.
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
from fuel_mass import configuration as fuel_configuration, tank_properties, combine
import ast
import json
import math
import re
import numpy as np
from scipy.optimize import root
from scipy.spatial.transform import Rotation
import trimesh
import xml.etree.ElementTree as E
from contact_physics import configure_contact_physics

W = WORKSPACE
R = W.parent
D = W / "src/usv_description"
cfg = json.loads((D / "config/trial_load.json").read_text())
export = json.loads((W / "../datasets/reference/boat_export.json").read_text())
collision_manifest = json.loads(
    (W / "../datasets/reference/collision_export.json").read_text()
)
assert (
    collision_manifest["source_sha256"] == export["source_sha256"]
), "Visual/collision exports refer to different Blender revisions"
energy = json.loads((W / "src/usv_energy/config/energy.json").read_text())
energy_keys = set(
    re.findall(
        r"P\((\w+),",
        (W / "src/usv_energy/include/usv_energy/Parameters.def").read_text(),
    )
)
assert (
    set(energy["parameters"]) == energy_keys
), "Energy configuration keys differ from the model schema"
assert all(
    math.isfinite(x) for x in energy["parameters"].values()
), "Nonfinite energy parameter"
propulsion = json.loads((D / "config/propulsion.json").read_text())
integration = json.loads((D / "config/integration.json").read_text())
payloads = json.loads((D / "config/payloads.json").read_text())


def _load_runtime_dependencies_1():
    """在当前路径和运行环境配置完成后加载依赖，保持原导入时机。"""
    from equipment_ledger import dry_links, auxiliary_w

    return (
        dry_links,
        auxiliary_w,
    )


(
    dry_links,
    auxiliary_w,
) = _load_runtime_dependencies_1()

energy["parameters"]["aux_output_w"] = auxiliary_w()
ledger_links = dry_links()
for link in ["azimuth", "propeller"]:
    lp = ledger_links[link]
    cfg[link + "_mass_kg"] = lp["mass_kg"]
    cfg[link + "_cg_m"] = (
        np.array(lp["cg_m"]) - np.array(export["origins"][link])
    ).tolist()
    cfg[link + "_inertia_matrix_kg_m2"] = lp["inertia_kg_m2"]
for key in [
    "rated_electrical_power_W",
    "motor_efficiency_assumed",
    "forward_static_thrust_N",
    "reverse_static_ratio",
    "reference_rpm",
    "reference_diameter_m",
    "advance_zero_thrust_m_s",
    "motor_stall_torque_Nm",
    "speed_control_gain_Nm_per_rad_s",
    "speed_reference_tau_s",
    "command_timeout_s",
]:
    assert math.isfinite(propulsion[key]) and propulsion[key] > 0, key
assert (
    energy["parameters"]["motor_efficiency"] == propulsion["motor_efficiency_assumed"]
), "Motor efficiency differs between propulsion and energy configs"
assert (
    energy["parameters"]["motor_limit_w"] == propulsion["rated_electrical_power_W"]
), "Motor rating differs between propulsion and energy configs"
assert (
    0 < propulsion["motor_efficiency_assumed"] <= 1
    and 0 < propulsion["reverse_static_ratio"] <= 1
)
assert 0 <= propulsion["throttle_deadband"] < 1


def _load_additional_dependencies():
    """在当前路径和运行环境配置完成后加载依赖，保持原导入时机。"""
    from resistance_budget import allocate_resistance

    return (allocate_resistance,)


(allocate_resistance,) = _load_additional_dependencies()

resistance_cfg = json.loads((D / "config/resistance.json").read_text())
appendage_cfg = json.loads((D / "config/appendage_hydrodynamics.json").read_text())
resistance_budget = allocate_resistance(resistance_cfg, appendage_cfg)
assert (
    appendage_cfg["density_kg_m3"] == cfg["fluid_density_kg_m3"]
), "Hull and appendage water densities disagree"
# Reject invalid trial parameters before producing a physically invalid SDF.
for key in [
    "fluid_density_kg_m3",
    "hull_mass_kg",
    "azimuth_mass_kg",
    "propeller_mass_kg",
    "propeller_radius_m",
    "steering_velocity_limit_rad_s",
    "steering_effort_limit_Nm",
    "steering_kp",
]:
    assert math.isfinite(cfg[key]) and cfg[key] > 0, key
for key in ["hull_inertia_box_size_m", "azimuth_inertia_box_size_m"]:
    assert len(cfg[key]) == 3 and all(math.isfinite(x) and x > 0 for x in cfg[key]), key
for key in ["linear_damping_N_s_m", "angular_damping_Nm_s_rad"]:
    assert len(cfg[key]) == 3 and all(
        math.isfinite(x) and x >= 0 for x in cfg[key]
    ), key
assert len(cfg["hull_cg_m"]) == 3 and all(math.isfinite(x) for x in cfg["hull_cg_m"])
assert 0 < cfg["steering_limit_deg"] <= 35
assert math.isfinite(cfg["steering_kd"]) and cfg["steering_kd"] >= 0
# Read the authoritative station literals; do not execute the Blender build script.
source_stations = R / "design/twin_thruster/06_维护脚本/build_hull_full.py"
if source_stations.is_file():
    tree = ast.parse(source_stations.read_text())
    stations = next(
        ast.literal_eval(n.value)
        for n in tree.body
        if isinstance(n, ast.Assign)
        and any(isinstance(x, ast.Name) and x.id == "stations_mm" for x in n.targets)
    )
else:
    stations = json.loads((D / "config/hull_sealed.json").read_text())["stations_mm"]
verts = []
rings = []
for x, k, b, c, w, h in np.array(stations) / 1000:
    pts = (
        [(x, w, h), (x, b, c), (x, 0, k), (x, -b, c), (x, -w, h), (x, 0, h + 0.02 * w)]
        if w
        else [(x, 0, k)]
    )
    ring = []
    for p in pts:
        ring.append(len(verts))
        verts.append(p)
    rings.append(ring)
faces = []


def add(poly):
    """把多边形按首顶点扇形划分并追加三角面片。

    Args:
        poly: 有序多边形顶点索引
    """
    for i in range(1, len(poly) - 1):
        faces.append([poly[0], poly[i], poly[i + 1]])


# Rings go clockwise viewed from +X; stern cap points -X.
add(rings[0])
for a, b in zip(rings, rings[1:]):
    for j in range(6):
        k = (j + 1) % 6
        poly = [a[j], b[0], a[k]] if len(b) == 1 else [a[j], b[j], b[k], a[k]]
        # Mirror the diagonal across y=0. The station quads are nonplanar:
        # using the same fan corner on both sides changes submerged volume.
        if len(poly) == 4 and j in (2, 3, 4):
            poly = poly[1:] + poly[:1]
        add(poly)
v = np.array(verts)
f = np.array(faces)
mesh = trimesh.Trimesh(v, f, process=True)
assert mesh.is_watertight and mesh.is_winding_consistent
if mesh.volume < 0:
    mesh.invert()
assert mesh.volume > 0
original = json.loads((D / "config/hull_geometry.json").read_text())
_, distance, _ = trimesh.proximity.closest_point_naive(
    mesh, np.array(original["vertices"])
)
assert (
    float(distance.max()) < 1e-5
), "Station envelope no longer matches original hull vertices"

v = mesh.vertices
f = mesh.faces
# Dry sealed envelope deliberately excludes deck penetrations; no flooding model.
mesh.export(D / "meshes/hull_collision.stl")
mesh.export(D / "meshes/hull_collision.obj", include_normals=True)
np.savetxt(D / "config/hull_triangles.txt", v[f].reshape(-1, 9), fmt="%.12g")
(D / "config/hull_sealed.json").write_text(
    json.dumps(
        {
            "vertices": v.tolist(),
            "faces": f.tolist(),
            "stations_mm": stations,
            "volume_m3": mesh.volume,
        },
        indent=2,
    )
)


def submerged(transformed, level=0.0):
    """计算变换后的封闭几何在水平水面下的体积与一阶矩。

    Args:
        transformed: 已变换的几何顶点，m
        level: 水平水面高度，m
    """
    vol = 0.0
    moment = np.zeros(3)
    for tri in transformed[f]:
        poly = []
        for a, b in zip(tri, np.roll(tri, -1, axis=0)):
            ia = a[2] <= level
            ib = b[2] <= level
            if ia:
                poly.append(a)
            if ia != ib:
                poly.append(a + (b - a) * (level - a[2]) / (b[2] - a[2]))
        ref = np.array([0.0, 0.0, level])
        for j in range(1, len(poly) - 1):
            a, b, c = poly[0], poly[j], poly[j + 1]
            dv = np.dot(a - ref, np.cross(b - ref, c - ref)) / 6
            vol += dv
            moment += dv * (ref + a + b + c) / 4
    return vol, moment / vol if vol > 1e-10 else np.zeros(3)


orig = export["origins"]
az = np.array(orig["azimuth"])
prop = np.array(orig["propeller"])
# Use envelope centre of rotating visual group as its TRIAL mass centre.
azcg = np.array(
    cfg.get(
        "azimuth_cg_m", np.mean(export["groups"]["azimuth"]["bounds_local"], axis=0)
    )
)
pcg = np.array(cfg.get("propeller_cg_m", [0, 0, 0]))
fuel_cfg = fuel_configuration()
assert (
    abs(fuel_cfg["density_kg_m3"] - energy["parameters"]["diesel_kg_l"] * 1000) < 1e-9
)
assert (
    abs(
        np.prod(fuel_cfg["inner_size_m"]) * 1000
        - energy["parameters"]["tank_capacity_l"]
    )
    < 1e-8
)
initial_fuel = energy["parameters"]["initial_fuel_l"]


def hull_properties(rot):
    """按姿态裁切燃油并合并到干船体属性。

    Args:
        rot: 模型到世界的旋转矩阵
    """
    return combine(
        fuel_cfg["dry_hull"],
        tank_properties(fuel_cfg, initial_fuel, rot.T @ np.array([0.0, 0.0, 1.0])),
    )


initial_hull = hull_properties(np.eye(3))
cfg["hull_mass_kg"] = initial_hull["mass_kg"]
cfg["hull_cg_m"] = initial_hull["cg_m"]
cfg["hull_inertia_matrix_kg_m2"] = initial_hull["inertia_kg_m2"]
(W / "../datasets/reference/fuel_mass_reference.json").write_text(
    json.dumps(fuel_cfg, indent=2)
)
masses = [cfg["hull_mass_kg"], cfg["azimuth_mass_kg"], cfg["propeller_mass_kg"]]
centers = [np.array(cfg["hull_cg_m"]), az + azcg, prop + pcg]
total = sum(masses)
cg = sum(m * c for m, c in zip(masses, centers)) / total


def ship_cg(rot):
    """按当前姿态计算整船组合重心。

    Args:
        rot: 模型到世界的旋转矩阵
    """
    hp = hull_properties(rot)
    return (
        hp["mass_kg"] * np.array(hp["cg_m"])
        + masses[1] * centers[1]
        + masses[2] * centers[2]
    ) / total


rho = cfg["fluid_density_kg_m3"]


def residual(q):
    """计算浮力与重心对齐的静水平衡残差。

    Args:
        q: 高度 m、滚转 rad、俯仰 rad 的姿态向量
    """
    z, roll, pitch = q
    rot = Rotation.from_euler("xyz", [roll, pitch, 0]).as_matrix()
    vol, cb = submerged(v @ rot.T + [0, 0, z])
    cw = rot @ ship_cg(rot) + [0, 0, z]
    return [vol - total / rho, cb[0] - cw[0], cb[1] - cw[1]]


sol = root(residual, [-0.22, 0, 0.015])
assert sol.success and np.linalg.norm(residual(sol.x)) < 1e-8
z, roll, pitch = sol.x
initial_hull = hull_properties(Rotation.from_euler("xyz", [roll, pitch, 0]).as_matrix())
cfg["hull_cg_m"] = initial_hull["cg_m"]
cfg["hull_inertia_matrix_kg_m2"] = initial_hull["inertia_kg_m2"]
centers[0] = np.array(initial_hull["cg_m"])
cg = ship_cg(Rotation.from_euler("xyz", [roll, pitch, 0]).as_matrix())


# Hydrostatic restoring stiffness wrt roll / pitch after force equilibrium.
def torque(q):
    """计算给定姿态下的静水恢复力矩。

    Args:
        q: 高度 m、滚转 rad、俯仰 rad 的姿态向量
    """
    zz, rr, pp = q
    rot = Rotation.from_euler("xyz", [rr, pp, 0]).as_matrix()
    vol, cb = submerged(v @ rot.T + [0, 0, zz])
    cw = rot @ ship_cg(rot) + [0, 0, zz]
    return np.cross(cb - cw, [0, 0, rho * 9.81 * vol])


stiff = []
for i in [1, 2]:
    a = sol.x.copy()
    b = sol.x.copy()
    a[i] += 1e-5
    b[i] -= 1e-5
    stiff.append(float(-(torque(a)[i - 1] - torque(b)[i - 1]) / 2e-5))
assert all(s > 0 for s in stiff)


# XML helpers.
def child(parent, tag, text=None, **attrs):
    """创建 XML 子节点并设置文本与属性。

    Args:
        parent: 目标父元素
        tag: 子元素标签
        text: 可选文本值
        attrs: 子元素命名属性
    """
    e = E.SubElement(parent, tag, attrs)
    if text is not None:
        e.text = str(text)
    return e


def nums(a):
    """把数值序列格式化为空格分隔的 SDF 文本。

    Args:
        a: 待序列化的数值序列
    """
    return " ".join(f"{x:.12g}" for x in a)


def inertial(link, m, c, diag):
    """校验并写入刚体的质量、重心和惯性元素。

    Args:
        link: 目标 SDF 刚体元素
        m: 质量，kg
        c: 重心坐标，m
        diag: 配置未提供完整矩阵时采用的对角惯性，kg·m²
    """
    matrix = np.array(
        cfg.get(link.get("name") + "_inertia_matrix_kg_m2", np.diag(diag)), dtype=float
    )
    assert (
        matrix.shape == (3, 3)
        and np.isfinite(matrix).all()
        and np.allclose(matrix, matrix.T)
    )
    ev = np.linalg.eigvalsh(matrix)
    assert ev.min() > 0 and 2 * ev.max() <= ev.sum() + 1e-8
    n = child(link, "inertial")
    child(n, "pose", nums([*c, 0, 0, 0]))
    child(n, "mass", m)
    i = child(n, "inertia")
    for k, x in zip(["ixx", "iyy", "izz"], np.diag(matrix)):
        child(i, k, x)
    for k, x in zip(["ixy", "ixz", "iyz"], [matrix[0, 1], matrix[0, 2], matrix[1, 2]]):
        child(i, k, x)


def box_inertia(m, size):
    """按均匀长方体假设计算重心处的对角惯性。

    Args:
        m: 质量，kg
        size: 长宽高，m
    """
    x, y, z = size
    return [
        m * (y * y + z * z) / 12,
        m * (x * x + z * z) / 12,
        m * (x * x + y * y) / 12,
    ]


sdf = E.Element("sdf", version="1.9")
model = child(sdf, "model", name="usv_azimuth_trial")
child(model, "self_collide", "true")
for group, name, pos, m, c, di in [
    (
        "hull",
        "hull",
        [0, 0, 0],
        masses[0],
        centers[0],
        box_inertia(masses[0], cfg["hull_inertia_box_size_m"]),
    ),
    (
        "azimuth",
        "azimuth",
        az,
        masses[1],
        azcg,
        box_inertia(masses[1], cfg["azimuth_inertia_box_size_m"]),
    ),
    (
        "propeller",
        "propeller",
        prop,
        masses[2],
        pcg,
        [
            0.5 * masses[2] * cfg["propeller_radius_m"] ** 2,
            0.25 * masses[2] * cfg["propeller_radius_m"] ** 2,
            0.25 * masses[2] * cfg["propeller_radius_m"] ** 2,
        ],
    ),
]:
    link_element = child(model, "link", name=name)
    child(link_element, "pose", nums([*pos, 0, 0, 0]))
    inertial(link_element, m, c, di)
    pbr_file = D / "config/pbr_visuals.json"
    if pbr_file.exists():
        for finish in json.loads(pbr_file.read_text())["groups"][group]:
            visual = child(link_element, "visual", name=finish["name"])
            g = child(child(visual, "geometry"), "mesh")
            child(g, "uri", finish["mesh"])
            material = child(visual, "material")
            child(material, "diffuse", "1 1 1 1")
            metal = child(child(material, "pbr"), "metal")
            for tag, value in [
                ("albedo_map", finish["albedo"]),
                ("roughness_map", finish["roughness_map"]),
                ("normal_map", finish["normal_map"]),
                ("metalness", finish["metalness"]),
                ("roughness", 1),
            ]:
                child(metal, tag, value)
    else:
        visual = child(link_element, "visual", name=group + "_visual")
        g = child(child(visual, "geometry"), "mesh")
        child(g, "uri", str(D / "meshes" / f"{group}.dae"))
    if name == "hull":
        co = child(link_element, "collision", name="sealed_hull")
        g = child(child(co, "geometry"), "mesh")
        child(g, "uri", str(D / "meshes/hull_collision.obj"))
    # Additional external geometry and exact moving-part meshes, in each link frame.
    co = child(link_element, "collision", name=name + "_detailed_external")
    g = child(child(co, "geometry"), "mesh")
    child(g, "uri", str(D / "meshes" / f"collision_{group}.dae"))
    # Separate topics make contacts attributable to each collision surface.
    for collision in link_element.findall("collision"):
        se = child(
            link_element,
            "sensor",
            name=collision.get("name") + "_contact",
            type="contact",
        )
        child(se, "always_on", "true")
        child(se, "update_rate", 50)
        child(se, "topic", "/usv/contacts/" + collision.get("name"))
        ct = child(se, "contact")
        child(ct, "collision", collision.get("name"))
        child(ct, "topic", "/usv/contacts/" + collision.get("name"))
    # Material contact coefficients remain uncalibrated; no compliance model.
for name, parent, cl, axis, limit in [
    ("steering", "hull", "azimuth", "0 0 1", math.radians(cfg["steering_limit_deg"])),
    ("propeller_spin", "azimuth", "propeller", "1 0 0", 1e16),
]:
    j = child(model, "joint", name=name, type="revolute")
    child(j, "parent", parent)
    child(j, "child", cl)
    child(j, "pose", "0 0 0 0 0 0", relative_to=cl)
    ax = child(j, "axis")
    child(ax, "xyz", axis)
    lim = child(ax, "limit")
    child(lim, "lower", -limit)
    child(lim, "upper", limit)
    child(
        lim,
        "effort",
        (
            cfg["steering_effort_limit_Nm"]
            if name == "steering"
            else propulsion["motor_stall_torque_Nm"]
        ),
    )
    child(
        lim,
        "velocity",
        (
            cfg["steering_velocity_limit_rad_s"]
            if name == "steering"
            else propulsion["reference_rpm"] * 2 * math.pi / 60 * 1.2
        ),
    )
    dyn = child(ax, "dynamics")
    child(dyn, "damping", 5 if name == "steering" else 0)
p = child(
    model, "plugin", filename="libUsvHullHydrostatics.so", name="usv::HullHydrostatics"
)
for k, x in [
    ("link_name", "hull"),
    ("density", rho),
    ("water_level", 0),
    ("linear_damping", nums(cfg["linear_damping_N_s_m"])),
    ("angular_damping", nums(cfg["angular_damping_Nm_s_rad"])),
    ("quadratic_damping", nums(resistance_budget["hull_quadratic_drag_N_s2_m2"])),
    (
        "angular_quadratic_damping",
        nums(resistance_budget["hull_quadratic_rotational_drag_Nm_s2_rad2"]),
    ),
    ("triangles_file", str(D / "config/hull_triangles.txt")),
]:
    child(p, k, x)
resistance_cfg = json.loads((D / "config/resistance.json").read_text())
child(p, "geometry_resistance", str(resistance_cfg["enabled"]).lower())
child(
    p,
    "distributed_lateral_drag",
    str(resistance_cfg["distributed_lateral_drag"]).lower(),
)
child(p, "reference_wet_area", resistance_cfg["reference_wet_area_m2"])
child(
    p, "reference_projected_area", nums(resistance_cfg["reference_projected_area_m2"])
)
hydro_cfg = json.loads((D / "config/hydrodynamics.json").read_text())
child(p, "added_linear", nums(hydro_cfg["added_linear_kg"]))
child(p, "added_angular", nums(hydro_cfg["added_angular_kg_m2"]))
child(p, "pressure_max_edge", integration.get("wave_pressure_max_edge_m", 0.35))
child(p, "drag_center", nums(integration["drag_center_m"]))
child(p, "air_center", nums(integration["air_center_m"]))
child(p, "air_cd_area", nums(integration["air_cd_area_m2"]))
p = child(
    model,
    "plugin",
    filename="libUsvAzimuthPropulsion.so",
    name="usv::AzimuthPropulsion",
)
for k, x in [
    ("reference_rpm", propulsion["reference_rpm"]),
    ("static_thrust", propulsion["forward_static_thrust_N"]),
    ("reverse_ratio", propulsion["reverse_static_ratio"]),
    (
        "shaft_power",
        propulsion["rated_electrical_power_W"] * propulsion["motor_efficiency_assumed"],
    ),
    ("stall_torque", propulsion["motor_stall_torque_Nm"]),
    ("speed_gain", propulsion["speed_control_gain_Nm_per_rad_s"]),
    ("reference_tau", propulsion["speed_reference_tau_s"]),
    ("deadband", propulsion["throttle_deadband"]),
    ("advance_speed", propulsion["advance_zero_thrust_m_s"]),
    ("command_timeout", propulsion["command_timeout_s"]),
    ("water_level", propulsion["water_level_m"]),
    ("radius", propulsion["reference_diameter_m"] / 2),
]:
    child(p, k, x)
fc = child(p, "fuel_mass")
for key, value in [
    ("dry_mass", fuel_cfg["dry_hull"]["mass_kg"]),
    ("dry_cg", nums(fuel_cfg["dry_hull"]["cg_m"])),
    ("dry_inertia", nums(np.array(fuel_cfg["dry_hull"]["inertia_kg_m2"]).flatten())),
    ("tank_size", nums(fuel_cfg["inner_size_m"])),
    ("tank_center", nums(fuel_cfg["center_m"])),
    ("density", fuel_cfg["density_kg_m3"]),
]:
    child(fc, key, value)
ec = child(p, "energy")
for k, energy_value in energy["parameters"].items():
    child(ec, k, energy_value)
sc = child(p, "steering")
for k, x in dict(
    reference_velocity_feedforward="true",
    kp=cfg["steering_kp"],
    ki=cfg.get("steering_ki", 0),
    kd=cfg["steering_kd"],
    limit=math.radians(cfg["steering_limit_deg"]),
    rate=cfg.get("steering_reference_rate_rad_s", 0.15),
    acceleration=cfg.get("steering_reference_acceleration_rad_s2", 0.3),
    effort=cfg["steering_effort_limit_Nm"],
    **integration["steering"],
).items():
    child(sc, k, x)
appendage_cfg = json.loads((D / "config/appendage_hydrodynamics.json").read_text())
ap = child(
    model,
    "plugin",
    filename="libUsvAppendageHydrodynamics.so",
    name="usv::AppendageHydrodynamics",
)
child(ap, "link_name", appendage_cfg["link"])
child(ap, "density", appendage_cfg["density_kg_m3"])
child(ap, "water_level", 0)
for surface in appendage_cfg["surfaces"]:
    sf = child(ap, "surface", name=surface["name"])
    child(sf, "cd0", surface["profile_drag_coefficient"])
    child(sf, "efficiency", surface["oswald_efficiency"])
    for outline_x, outline_z in surface["outline_xz_m"]:
        child(sf, "point", nums([outline_x, 0, outline_z]))
for plug in model.findall("plugin"):
    if plug.get("filename") in [
        "libUsvHullHydrostatics.so",
        "libUsvAzimuthPropulsion.so",
        "libUsvAppendageHydrodynamics.so",
    ]:
        child(plug, "topic_namespace", "usv")
        field = child(plug, "wavefield")
        child(field, "topic", "/vrx/wavefield/parameters")
        wave = child(field, "wave")
        child(wave, "gain", 0)
        child(wave, "steepness", 0)
# Sensor poses are geometric/engineering mounting references; device mass is
# already part of the aggregate equipment ledger, not added a second time.
hull = model.find("link[@name='hull']")


def _load_runtime_dependencies_2():
    """在当前路径和运行环境配置完成后加载依赖，保持原导入时机。"""
    from payload_geometry import add_mounts, add_ledger_solids
    from equipment_ledger import rows

    from passive_stabilizer import (
        add_stabilizer,
        configuration as stabilizer_configuration,
    )

    return (
        add_mounts,
        add_ledger_solids,
        rows,
        add_stabilizer,
        stabilizer_configuration,
    )


(
    add_mounts,
    add_ledger_solids,
    ledger_rows,
    add_stabilizer,
    stabilizer_configuration,
) = _load_runtime_dependencies_2()

add_mounts(hull, payloads)
add_ledger_solids(hull, ledger_rows())

add_stabilizer(model, stabilizer_configuration(), D / "meshes/passive_stabilizer")


def sensor(name, kind, pose, hz):
    """创建传感器并优先应用载荷配置中的安装位姿。

    Args:
        name: 传感器名称
        kind: SDF 传感器类型
        pose: 默认六元位姿字符串，位置 m、角度 rad
        hz: 更新频率，Hz
    """
    pose = nums(payloads["poses"].get(name, list(map(float, pose.split()))))
    se = child(hull, "sensor", name=name, type=kind)
    child(se, "pose", pose)
    child(se, "always_on", "true")
    child(se, "update_rate", hz)
    child(se, "topic", "/usv/sensors/" + name)
    child(se, "gz_frame_id", "usv/" + name)
    return se


def noise(parent, sigma):
    """向传感器参数添加零均值高斯噪声配置。

    Args:
        parent: 目标测量分量的 XML 元素
        sigma: 标准差，单位与该测量分量一致
    """
    n = child(parent, "noise", type="gaussian")
    child(n, "mean", 0)
    child(n, "stddev", sigma)


im = child(
    sensor("imu", "imu", "0 0 0.5 0 0 0", integration["sensors"]["imu_hz"]), "imu"
)
child(im, "enable_orientation", "true")
child(child(im, "orientation_reference_frame"), "localization", "ENU")
for group, sigma in [("angular_velocity", 0.0002), ("linear_acceleration", 0.01)]:
    for axis in ["x", "y", "z"]:
        noise(
            child(child(im, group) if im.find(group) is None else im.find(group), axis),
            sigma,
        )
nav = child(
    sensor("gps", "navsat", "0.5 0 0.9 0 0 0", integration["sensors"]["gps_hz"]),
    "navsat",
)
for group, hs, vs in [
    ("position_sensing", 0.15, 0.3),
    ("velocity_sensing", 0.02, 0.03),
]:
    n = child(nav, group)
    noise(child(n, "horizontal"), hs)
    noise(child(n, "vertical"), vs)
mag = child(
    sensor(
        "magnetometer",
        "magnetometer",
        "0 0 1 0 0 0",
        integration["sensors"]["magnetometer_hz"],
    ),
    "magnetometer",
)
for axis in ["x", "y", "z"]:
    noise(child(mag, axis), 1e-7)
cam = child(
    sensor("camera", "camera", "1.5 0 1 0 0.10 0", payloads["camera"]["hz"]), "camera"
)
child(cam, "camera_info_topic", "/usv/sensors/camera_info")
child(cam, "horizontal_fov", payloads["camera"]["horizontal_fov_rad"])
im = child(cam, "image")
child(im, "width", payloads["camera"]["width"])
child(im, "height", payloads["camera"]["height"])
child(im, "format", "R8G8B8")
cl = child(cam, "clip")
child(cl, "near", 0.1)
child(cl, "far", 500)
li = child(
    sensor("lidar", "gpu_lidar", "0 0 1 0 0 0", payloads["lidar"]["hz"]), "lidar"
)
scan = child(li, "scan")
h = child(scan, "horizontal")
child(h, "samples", payloads["lidar"]["horizontal_samples"])
child(h, "resolution", 1)
child(h, "min_angle", -math.pi)
child(h, "max_angle", math.pi)
ran = child(li, "range")
child(ran, "min", 0.2)
child(ran, "max", 100)
child(ran, "resolution", 0.02)
ln = child(li, "noise")
child(ln, "type", "gaussian")
child(ln, "mean", 0)
child(ln, "stddev", 0.02)
# Optional VRX-class payload profile; keep the original planar scan interface.
if payloads["advanced_sensors"]:
    import copy

    for camera_name in ["camera_right", "camera_side"]:
        se = copy.deepcopy(hull.find("sensor[@name='camera']"))
        se.set("name", camera_name)
        se.find("pose").text = nums(payloads["poses"][camera_name])
        se.find("topic").text = "/usv/sensors/" + camera_name
        se.find("gz_frame_id").text = "usv/" + camera_name
        se.find("camera/camera_info_topic").text = (
            "/usv/sensors/" + camera_name + "_info"
        )
        hull.append(se)
    se = copy.deepcopy(hull.find("sensor[@name='lidar']"))
    se.set("name", "lidar3d")
    se.find("topic").text = "/usv/sensors/lidar3d"
    se.find("gz_frame_id").text = "usv/lidar"
    scan = se.find("lidar/scan")
    vertical_scan = child(scan, "vertical")
    lc = payloads["lidar"]
    child(vertical_scan, "samples", lc["vertical_samples"])
    child(vertical_scan, "resolution", 1)
    child(vertical_scan, "min_angle", math.radians(lc["vertical_min_deg"]))
    child(vertical_scan, "max_angle", math.radians(lc["vertical_max_deg"]))
    hull.append(se)
p = child(
    model,
    "plugin",
    filename="gz-sim-pose-publisher-system",
    name="gz::sim::systems::PosePublisher",
)
child(p, "publish_link_pose", "true")
child(p, "use_pose_vector_msg", "true")
child(p, "update_frequency", 10)
p = child(
    model,
    "plugin",
    filename="gz-sim-joint-state-publisher-system",
    name="gz::sim::systems::JointStatePublisher",
)
child(p, "topic", "/usv/joint_state")
p = child(
    model,
    "plugin",
    filename="gz-sim-odometry-publisher-system",
    name="gz::sim::systems::OdometryPublisher",
)
for k, x in [
    ("odom_frame", "world"),
    ("robot_base_frame", "usv/base_link"),
    ("odom_topic", "/usv/ground_truth/odometry"),
    ("tf_topic", "/usv/ground_truth/tf"),
    ("dimensions", 3),
    ("odom_publish_frequency", 20),
]:
    child(p, k, x)
E.indent(sdf)
E.ElementTree(sdf).write(
    D / "models/usv_azimuth_trial.sdf", encoding="utf-8", xml_declaration=True
)
world = E.parse(W / "src/usv_simulation/worlds/usv_baseline_offline.sdf")
ww = world.getroot().find("world")
ww.set("name", "usv_hull_trial")
child(ww, "gravity", "0 0 -9.81")
configure_contact_physics(ww)
# Pure still-water test: no wind, wave forcing or decorative projectile. Flat
# local water visual prevents a moving rendered surface disagreeing with physics.
for inc in list(ww.findall("include")):
    ww.remove(inc)
for p in list(ww.findall("plugin")):
    if "Wind" in p.get("name", "") or "Publisher" in p.get("name", ""):
        ww.remove(p)
water = child(ww, "model", name="flat_water")
child(water, "static", "true")
link_element = child(water, "link", name="water")
vis = child(link_element, "visual", name="surface")
child(vis, "cast_shadows", "false")
plane = child(child(vis, "geometry"), "plane")
child(plane, "normal", "0 0 1")
child(plane, "size", "2000 2000")
mat = child(vis, "material")
child(mat, "ambient", "0.02 0.15 0.2 1")
child(mat, "diffuse", "0.02 0.25 0.35 1")
child(vis, "transparency", 0.3)
inc = child(ww, "include")
child(inc, "uri", str(D / "models/usv_azimuth_trial.sdf"))
child(inc, "pose", nums([0, 0, *sol.x, 0]))
cam = ww.find("gui/plugin[@name='3D View']/camera_pose")
if cam is not None:
    cam.text = "-9 -9 6 0 0.4 0.8"
E.indent(world)
world.write(
    W / "src/usv_simulation/worlds/usv_hull_trial.sdf",
    encoding="utf-8",
    xml_declaration=True,
)
report = {
    "profile": cfg["status"],
    "sealed_envelope_watertight": True,
    "max_source_vertex_distance_m": float(distance.max()),
    "sealed_envelope_volume_m3": float(mesh.volume),
    "triangles": len(f),
    "total_trial_mass_kg": total,
    "trial_total_cg_m": cg.tolist(),
    "equilibrium_pose_z_roll_pitch": sol.x.tolist(),
    "displaced_volume_m3": total / rho,
    "residual": residual(sol.x),
    "roll_pitch_restoring_stiffness_Nm_rad": stiff,
    "limitations": [
        "station envelope sealed deck; no flooding",
        "appendage displacement omitted",
        "estimated aggregate/mesh inertia; no measured mass distribution",
        "linear/quadratic damping uncalibrated; constant diagonal added mass trial; VRX distributed hull pressure and deep-water orbital flow; relative current/wind enabled",
        "propulsion and energy are engineering proxies; fuel inertia uses a quasi-static unbaffled box tank",
    ],
}
(W / "../datasets/reference/hydrostatic_trial.json").write_text(
    json.dumps(report, indent=2)
)
print(json.dumps(report, indent=2))
