"""准备固定裸船体两相流短时预检；不把启动瞬态当作操纵系数。

OpenCFD v2506 / interFoam，单位采用 SI，船体 FLU 轴与世界轴平行，
静水面 z=0。只生成新 case；求解、网格门禁和长时间收敛需要另行执行。
近壁暂用 SST 壁面函数，必须检查 y+；没有棱柱层，不能直接用于标定。
"""

import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil

import numpy as np

from bare_hull_cfd import foam_list

TEMPLATES = Path(__file__).with_name("cfd_templates")
NORMALS = {
    "xmin": (-1, 0, 0),
    "xmax": (1, 0, 0),
    "ymin": (0, -1, 0),
    "ymax": (0, 1, 0),
}


def flow_state(u, beta):
    """将船体速度转换为固定船壳的来流，并计算初始湍流量。

    Args:
        u: 船体有符号纵速，m/s；倒车为负。
        beta: 斜流定义 v=|u|tan(beta)，单位度，绝对值小于45。

    Returns:
        世界来流向量、k（m²/s²）、omega（1/s）；强度1%、长度0.42 m。

    Raises:
        ValueError: 输入非有限，或静水与斜流角矛盾。
    """
    if not np.isfinite([u, beta]).all() or abs(beta) >= 45 or (u == 0 and beta != 0):
        raise ValueError("速度/斜流角无效；静水必须使用零斜流角")
    flow = np.array([-u, -abs(u) * np.tan(np.deg2rad(beta)), 0.0])
    k = max(1.5 * (np.linalg.norm(flow) * 0.01) ** 2, 1e-8)
    omega = max(np.sqrt(k) / (0.09**0.25 * 0.42), 1e-4)
    return flow, k, omega


def initial_water_phase(vertices):
    """判断共形多边形面属于水还是空气，拒绝跨水面的面。

    Args:
        vertices: 面顶点的有限世界坐标数组，至少三点，单位 m。

    Returns:
        静水面下返回1，否则返回0；不是非共形面的面积分数估计。
    """
    z = vertices[:, 2]
    if z.min() < -1e-9 and z.max() > 1e-9:
        raise ValueError("存在跨水面的面，不能用二值初始化")
    return float(z.mean() < 0)


def boundary_water(mesh):
    """读取多边形边界面的水相初值，要求静水面与体网格共形。

    Args:
        mesh: polyMesh 目录 Path，要求 points/faces 为 ASCII。

    Returns:
        每个外边界 patch 对应的0/1水相数组。

    Raises:
        ValueError: 点/面数量或索引不符、patch 缺失或边界面跨水面。
    """
    count, text = foam_list(mesh / "points")
    points = np.fromstring(text.replace("(", " ").replace(")", " "), sep=" ")
    if points.size != count * 3:
        raise ValueError("点列表数量不符")
    points = points.reshape(-1, 3)
    count, text = foam_list(mesh / "faces")
    faces = re.findall(r"(\d+)\(([^()]*)\)", text)
    if len(faces) != count or not np.isfinite(points).all():
        raise ValueError("点坐标或面列表数量无效")
    boundary = (mesh / "boundary").read_text()
    patches = re.findall(
        r"(\w+)\s*\{[^}]*?nFaces\s+(\d+);\s*startFace\s+(\d+);", boundary
    )
    result = {}
    for name, count, start in patches:
        values = []
        subset = faces[int(start) : int(start) + int(count)]
        if len(subset) != int(count):
            raise ValueError(f"边界 {name} 的面索引越界")
        for size, text in subset:
            indices = np.fromstring(text, sep=" ", dtype=int)
            if (
                len(indices) != int(size)
                or len(indices) < 3
                or indices.min() < 0
                or indices.max() >= len(points)
            ):
                raise ValueError("边界面顶点索引无效")
            values.append(initial_water_phase(points[indices]))
        result[name] = np.asarray(values)
    if set(result) != set(NORMALS) | {"hull", "top", "bottom"}:
        raise ValueError("边界标记不是约定的七个 patch")
    return result


def patch_fields(name, water, flow, k, omega):
    """构造一个边界的六个场条件，斜流侧入口由法向通量判断。

    Args:
        name: 七个约定 patch 中的名称。
        water: 本 patch 面上的0/1水相初值数组。
        flow: 世界来流，m/s，长度3。
        k: 入口湍动能，m²/s²。
        omega: 入口比耗散率，1/s。

    Returns:
        U、p_rgh、alpha.water、k、omega、nut 的字典内容。
    """
    vector = "(" + " ".join(f"{v:.12g}" for v in flow) + ")"
    alpha = f"nonuniform List<scalar>\n{len(water)}\n(\n"
    alpha += "\n".join(f"{value:g}" for value in water) + "\n)"
    inlet = name in NORMALS and np.dot(flow, NORMALS[name]) < -1e-10
    result = {
        "U": f"type inletOutlet; inletValue uniform {vector}; value uniform {vector};",
        "p_rgh": "type fixedValue; value uniform 0;",
        "alpha.water": f"type inletOutlet; inletValue {alpha}; value {alpha};",
        "k": f"type inletOutlet; inletValue uniform {k}; value uniform {k};",
        "omega": f"type inletOutlet; inletValue uniform {omega}; value uniform {omega};",
        "nut": "type calculated; value uniform 0;",
    }
    if inlet:
        result.update(
            {
                "U": f"type fixedValue; value uniform {vector};",
                "p_rgh": "type fixedFluxPressure; value uniform 0;",
                "alpha.water": f"type fixedValue; value {alpha};",
                "k": f"type fixedValue; value uniform {k};",
                "omega": f"type fixedValue; value uniform {omega};",
            }
        )
    elif name == "top":
        result["U"] = f"type pressureInletOutletVelocity; value uniform {vector};"
        result["alpha.water"] = (
            "type inletOutlet; inletValue uniform 0; value uniform 0;"
        )
    elif name in ("hull", "bottom"):
        result.update(
            {
                "U": "type noSlip;" if name == "hull" else "type slip;",
                "p_rgh": "type fixedFluxPressure; value uniform 0;",
                "alpha.water": "type zeroGradient;",
                "k": "type zeroGradient;",
                "omega": "type zeroGradient;",
            }
        )
        if name == "hull":
            result.update(
                {
                    "k": f"type kqRWallFunction; value uniform {k};",
                    "omega": f"type omegaWallFunction; value uniform {omega};",
                    "nut": "type nutUSpaldingWallFunction; value uniform 0;",
                }
            )
    return result


def write_foam(path, class_name, body):
    """写入单个新 case 的字典或场文件。

    Args:
        path: 输出文件 Path；其父目录必须已存在。
        class_name: OpenFOAM 对象类型名。
        body: 头部之后的完整字典文本。
    """
    path.write_text(
        f"FoamFile {{ version 2.0; format ascii; class {class_name}; object {path.name}; }}\n"
        + body
        + "\n"
    )


def prepare_case(mesh_case, output, equilibrium, u, beta, duration):
    """复制网格并生成预检算例，来源/产物哈希写入末尾清单。

    Args:
        mesh_case: 含 constant/polyMesh 的已检查网格目录 Path。
        output: 不得存在的新算例目录 Path；失败可留下部分文件。
        equilibrium: 静态工况 JSON 路径，含 cg_m 与平衡 z/roll/pitch。
        u: 有符号船体纵速，m/s，倒车为负。
        beta: 船体斜流角定义，度。
        duration: 有限正的预检时长，s；不代表稳态统计时长。

    Raises:
        ValueError: 参数、水平姿态、网格或边界条件不满足本轮约定。
        FileExistsError: 证据目录已存在，不覆盖。
    """
    flow, k, omega = flow_state(u, beta)
    if not np.isfinite(duration) or duration <= 0:
        raise ValueError("预检时长必须为有限正数")
    state = json.loads(equilibrium.read_text())
    pose = np.asarray(state["equilibrium_z_roll_pitch"], dtype=float)
    cg = np.asarray(state["cg_m"], dtype=float)
    if pose.shape != (3,) or cg.shape != (3,) or not np.isfinite([pose, cg]).all():
        raise ValueError("工况坐标必须是有限三维数组")
    if np.max(np.abs(pose[1:])) > 1e-8:
        raise ValueError("本轮只接受世界轴与船体轴平行的固定姿态")
    cg = cg + [0, 0, pose[0]]
    source_mesh = mesh_case / "constant/polyMesh"
    water = boundary_water(source_mesh)
    output.mkdir(parents=True, exist_ok=False)
    shutil.copytree(
        source_mesh, output / "constant/polyMesh", ignore=shutil.ignore_patterns("sets")
    )
    (output / "system").mkdir()
    (output / "0").mkdir()
    boundary = output / "constant/polyMesh/boundary"
    boundary.write_text(
        re.sub(r"(hull\s*\{\s*type\s+)patch", r"\1wall", boundary.read_text())
    )
    bcs = {
        name: patch_fields(name, alpha, flow, k, omega) for name, alpha in water.items()
    }
    vector = "(" + " ".join(f"{v:.12g}" for v in flow) + ")"
    fields = {
        "U": ("Vector", "0 1 -1 0 0 0 0", vector),
        "p_rgh": ("Scalar", "1 -1 -2 0 0 0 0", "0"),
        "alpha.water": ("Scalar", "0 0 0 0 0 0 0", "0"),
        "k": ("Scalar", "0 2 -2 0 0 0 0", str(k)),
        "omega": ("Scalar", "0 0 -1 0 0 0 0", str(omega)),
        "nut": ("Scalar", "0 2 -1 0 0 0 0", "0"),
    }
    for name, (kind, dimensions, value) in fields.items():
        body = f"dimensions [{dimensions}];\ninternalField uniform {value};\nboundaryField\n{{\n"
        body += (
            "\n".join(patch + " { " + bc[name] + " }" for patch, bc in bcs.items())
            + "\n}"
        )
        write_foam(output / "0" / name, "vol" + kind + "Field", body)
    for name in (
        "fvSchemes",
        "fvSolution",
        "transportProperties",
        "turbulenceProperties",
    ):
        directory = "system" if name.startswith("fv") else "constant"
        shutil.copy(TEMPLATES / name, output / directory / name)
    write_foam(
        output / "constant/g",
        "uniformDimensionedVectorField",
        "dimensions [0 1 -2 0 0 0 0]; value (0 0 -9.81);",
    )
    control = (TEMPLATES / "controlDict").read_text()
    control = control.replace("@END@", str(duration)).replace(
        "@CG@", " ".join(map(str, cg))
    )
    write_foam(output / "system/controlDict", "dictionary", control)
    write_foam(
        output / "system/setFieldsDict",
        "dictionary",
        "defaultFieldValues (volScalarFieldValue alpha.water 0);\n"
        "regions (boxToCell { box (-25 -13 -13) (25 13 0); "
        "fieldValues (volScalarFieldValue alpha.water 1); });",
    )
    write_foam(
        output / "system/decomposeParDict",
        "dictionary",
        "numberOfSubdomains 4; method scotch;",
    )
    manifest = {
        "scope": "UNQUALIFIED_STARTUP_PILOT_NOT_CALIBRATION",
        "u_m_s": u,
        "beta_deg": beta,
        "flow_m_s": flow.tolist(),
        "duration_s": duration,
        "cg_world_m": cg.tolist(),
        "k": k,
        "omega": omega,
        "tool_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "equilibrium_sha256": hashlib.sha256(equilibrium.read_bytes()).hexdigest(),
        "case_sha256": {
            str(path.relative_to(output)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(output.rglob("*"))
            if path.is_file()
        },
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")


def main():
    """解析 case 准备参数；不隐式启动或重启任何仿真进程。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mesh-case", type=Path, required=True, help="已检查的网格目录"
    )
    parser.add_argument("--output", type=Path, required=True, help="新的试验目录")
    parser.add_argument(
        "--equilibrium", type=Path, required=True, help="静态载荷工况 JSON"
    )
    parser.add_argument("--u", type=float, required=True, help="船体纵速 m/s，倒车为负")
    parser.add_argument("--beta", type=float, default=0, help="斜流角，度")
    parser.add_argument("--duration", type=float, default=0.1, help="启动预检时长，s")
    args = parser.parse_args()
    prepare_case(
        args.mesh_case, args.output, args.equilibrium, args.u, args.beta, args.duration
    )


if __name__ == "__main__":
    main()
