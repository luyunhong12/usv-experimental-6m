"""固定船体、均匀来流的Gazebo拖曳台架；只检验现有力模型，不是CFD。

船体FLU坐标，纵速u固定，横速v=|u|tan(beta)。约束保持零角速度、
静态吃水和姿态；关闭推进、空气力和波浪，保留船体/附体水动力。
输出完整遥测与分项力矩，绝不拟合或写回生产参数。
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

TOPICS = {
    "hull": "audit/hull",
    "appendages": "audit/appendages",
    "separation": "hydrodynamics/separation",
    "pose": "hydrostatics",
    "environment": "environment/state",
}


def tow_matrix(speeds, angles):
    """生成前后航和正负斜流矩阵，不把beta误作常规倒车航向角。

    Args:
        speeds: 正的纵速绝对值列表，m/s。
        angles: 斜流角列表，度；绝对值须小于45。
    """
    values = np.asarray([*speeds, *angles], dtype=float)
    if (
        not np.isfinite(values).all()
        or min(speeds) <= 0
        or max(abs(np.array(angles))) >= 45
    ):
        raise ValueError("Invalid towing matrix")
    return [
        {
            "u_m_s": direction * speed,
            "beta_deg": angle,
            "v_m_s": speed * np.tan(np.radians(angle)),
        }
        for direction in (1, -1)
        for speed in speeds
        for angle in angles
    ]


def digest(path):
    """返回实际文件的SHA256，缺失文件直接报错。

    Args:
        path: 需要绑定的本地输入文件路径。
    """
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def prepare_rig(model_path, equilibrium_path, output, plugins, step):
    """生成独立静态台架，保持源模型质量/几何，绑定实际加载的插件。

    Args:
        model_path: 无鳍候选SDF路径，必须包含所需两个水动力插件。
        equilibrium_path: 静水平衡JSON路径，含z/roll/pitch，单位m/rad。
        output: 新的证据目录；调用者负责拒绝覆盖。
        plugins: 插件类名到实际共享库路径的映射。
        step: 正物理步长，s，不得超过附加惯性的4 ms限制。
    """
    if not 0 < step <= 0.004:
        raise ValueError("Step must be in (0, .004] s")
    tree = ET.parse(model_path)
    model = tree.find("model")
    selected = {
        p.get("name"): p for p in model.findall("plugin") if p.get("name") in plugins
    }
    if set(selected) != set(plugins) or len(model.findall(".//model")):
        raise ValueError("Rig requires a flat hull/appendage candidate")
    inputs = {
        str(p): digest(p) for p in [model_path, equilibrium_path, *plugins.values()]
    }
    for p in model.findall(".//*"):
        if p.tag.endswith("_file"):
            inputs[p.text] = digest(p.text)
    static = model.find("static")
    if static is None:
        static = ET.SubElement(model, "static")
    static.text = "true"
    for p in list(model.findall("plugin")):
        if p.get("name") not in plugins:
            model.remove(p)
        else:
            p.set("filename", str(Path(plugins[p.get("name")]).resolve()))
    for link in model.findall("link"):
        for sensor in list(link.findall("sensor")):
            link.remove(sensor)
    for wave in model.findall(".//wavefield/wave"):
        wave.find("gain").text = "0"
    selected["usv::HullHydrostatics"].find("air_cd_area").text = "0 0 0"
    eq = json.loads(Path(equilibrium_path).read_text())["equilibrium_z_roll_pitch"]
    if len(eq) != 3 or not np.isfinite(eq).all():
        raise ValueError("Invalid static equilibrium")
    tree.write(output / "rig.sdf")
    world = ET.Element("sdf", version="1.9")
    w = ET.SubElement(world, "world", name="captive_tow")
    ET.SubElement(w, "gravity").text = "0 0 -9.81"
    physics = ET.SubElement(w, "physics", name="physics", type="ignored")
    ET.SubElement(physics, "max_step_size").text = str(step)
    ET.SubElement(physics, "real_time_factor").text = "0"
    ET.SubElement(
        w, "plugin", filename="gz-sim-physics-system", name="gz::sim::systems::Physics"
    )
    inc = ET.SubElement(w, "include")
    ET.SubElement(inc, "uri").text = str(output / "rig.sdf")
    ET.SubElement(inc, "pose").text = " ".join(map(str, [0, 0, *eq, 0]))
    ET.ElementTree(world).write(output / "world.sdf")
    return {"input_sha256": inputs, "equilibrium": eq, "step_s": step}


def tail_samples(rows, begin, end, columns):
    """选取稳定尾段并拒绝缺样、非有限值、时间跳变和错误消息宽度。

    Args:
        rows: 某个遥测主题的全部数值行。
        begin: 尾段起始仿真时间，s。
        end: 尾段终止仿真时间，s。
        columns: 消息应有的列数。
    """
    a = np.asarray(rows, dtype=float)
    if a.ndim != 2 or a.shape[1] != columns:
        raise ValueError("Missing or incompatible telemetry")
    a = a[(a[:, 0] >= begin) & (a[:, 0] <= end)]
    if len(a) < 5 or not np.isfinite(a).all():
        raise ValueError("Insufficient finite samples")
    dt = np.diff(a[:, 0])
    if np.any(dt <= 0) or np.max(dt) > 0.25 or a[-1, 0] - a[0, 0] < 0.7:
        raise ValueError("Incomplete telemetry window")
    return a


def reduce_stage(data, case, begin, end, equilibrium, drag_x):
    """从真实插件审计分离力分项，全部力矩统一到整船重心。

    Args:
        data: 各主题原始遥测字典。
        case: 当前u、v和beta定义。
        begin: 工况开始时间，s。
        end: 工况结束时间，s；仅统计末1秒。
        equilibrium: 静态z/roll/pitch，m/rad。
        drag_x: 船体阻力参考点的纵坐标，m。
    """
    widths = {
        "hull": 28,
        "appendages": 10,
        "separation": 6,
        "pose": 12,
        "environment": 16,
    }
    selected = {
        k: tail_samples(data[k], max(begin + 1, end - 1), end, n)
        for k, n in widths.items()
    }
    mean = {k: a.mean(axis=0) for k, a in selected.items()}
    h, s = mean["hull"], mean["separation"]
    sep = np.array([0, s[4], 0, 0, 0, s[5] + (drag_x - h[1]) * s[4]])
    # 侧向分离力只用于横向/偏航分解，横倾力矩保留在原完整drag项中。
    components = {
        "hull_drag_with_separation": h[10:16].tolist(),
        "added_mass": h[16:22].tolist(),
        "appendages": mean["appendages"][4:10].tolist(),
    }
    total = sum(np.asarray(v) for v in components.values())
    local = np.array([case["u_m_s"], case["v_m_s"], 0])
    rotation = Rotation.from_euler("xyz", [*equilibrium[1:], 0]).as_matrix()
    expected_pose = np.array([0, 0, *equilibrium, 0])
    pose_error = np.max(abs(selected["pose"][:, 6:12] - expected_pose))
    water_error = np.max(abs(selected["environment"][:, 4:7] + rotation @ local))
    velocity_error = np.max(abs(selected["separation"][:, 1:3] - local[:2]))
    spread = max(
        np.ptp(selected[k][:, 4:], axis=0).max() for k in ("hull", "appendages")
    )
    checks = {
        "pose_fixed": bool(pose_error < 1e-8),
        "current_delivered": bool(water_error < 1e-8),
        "relative_velocity": bool(velocity_error < 1e-8),
        "force_steady": bool(spread < 1e-5),
        "drag_passive": bool(np.dot(h[10:13] + mean["appendages"][4:7], local) <= 1e-6),
    }
    return {
        **case,
        "begin_s": begin,
        "end_s": end,
        "components_N_Nm": components,
        "separation_Y_N": float(sep[1]),
        "separation_N_Nm": float(sep[5]),
        "hull_drag_without_separation_Y_N": float(h[11] - sep[1]),
        "hull_drag_without_separation_N_Nm": float(h[15] - sep[5]),
        "total_N_Nm": total.tolist(),
        "checks": checks,
        "valid": all(checks.values()),
    }


def wait_ready(process, publishers, data):
    """等待真实通信连接及首个物理遥测，失败不伪造试验结果。

    Args:
        process: 当前Gazebo子进程。
        publishers: 必须建立连接的发布器列表。
        data: 订阅器追加的遥测字典。
    """
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError("Gazebo exited before rig became ready")
        if all(p.has_connections() for p in publishers) and all(data.values()):
            return
        time.sleep(0.05)
    raise TimeoutError("Rig transport/telemetry unavailable")


def run_sweep(output, cases, metadata):
    """启动独立Gazebo，顺序执行工况，异常也保存原始数据并清理进程。

    Args:
        output: 已准备台架world.sdf的证据目录。
        cases: 固定姿态来流工况列表。
        metadata: 输入哈希、静平衡和步长绑定信息。
    """
    from gz.transport13 import Node
    from gz.msgs10.double_v_pb2 import Double_V
    from gz.msgs10.vector3d_pb2 import Vector3d

    node, data, callbacks = Node(), {k: [] for k in TOPICS}, []
    for key, topic in TOPICS.items():

        def capture(message, key=key):
            """保存订阅行。

            Args:
                message: 当前Double_V消息。
                key: 当前主题的固定字典键。
            """
            data[key].append(list(message.data))

        callbacks.append(capture)
        node.subscribe(Double_V, "/usv/" + topic, capture)
    current = node.advertise("/vrx/current/velocity", Vector3d)
    log = (output / "runtime.log").open("w")
    process = subprocess.Popen(
        ["gz", "sim", "-s", "-r", str(output / "world.sdf")],
        stdout=log,
        stderr=log,
        start_new_session=True,
    )
    results = []
    try:
        wait_ready(process, [current], data)
        rotation = Rotation.from_euler(
            "xyz", [*metadata["equilibrium"][1:], 0]
        ).as_matrix()
        model = ET.parse(output / "rig.sdf").find("model")
        drag_x = float(
            model.findtext("plugin[@name='usv::HullHydrostatics']/drag_center").split()[
                0
            ]
        )
        for case in cases:
            begin = data["pose"][-1][0]
            deadline = time.monotonic() + 45
            water = -rotation @ np.array([case["u_m_s"], case["v_m_s"], 0])
            while data["pose"][-1][0] - begin < 3:
                if process.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError("Rig exited or stalled during a case")
                current.publish(Vector3d(x=water[0], y=water[1], z=water[2]))
                time.sleep(0.01)
            end = data["pose"][-1][0]
            result = reduce_stage(
                data, case, begin, end, metadata["equilibrium"], drag_x
            )
            results.append(result)
            if not result["valid"]:
                raise RuntimeError("Invalid imposed-motion measurement")
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        log.close()
        for key, rows in data.items():
            np.savetxt(output / (key + ".csv"), rows, delimiter=",")
        (output / "results.json").write_text(json.dumps(results, indent=2) + "\n")
    unchanged = all(digest(p) == sha for p, sha in metadata["input_sha256"].items())
    report = {
        **metadata,
        "scope": "MODEL_INTERNAL_CAPTIVE_TEST_NOT_CFD",
        "source_unchanged": unchanged,
        "completed": len(results) == len(cases),
        "cases": results,
    }
    (output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    if not unchanged:
        raise RuntimeError("Input changed during captive tests")


def main():
    """解析显式输入并运行；复用结果目录会失败，不修改源船模。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--equilibrium", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--hull-plugin", type=Path, required=True)
    parser.add_argument("--appendage-plugin", type=Path, required=True)
    parser.add_argument("--step", type=float, default=0.001)
    parser.add_argument(
        "--speeds", type=float, nargs="+", default=[1.884, 2.27, 2.66, 3.05, 3.43]
    )
    parser.add_argument(
        "--angles",
        type=float,
        nargs="+",
        default=[-20, -10, -5, -2, -1, -0.5, 0, 0.5, 1, 2, 5, 10, 20],
    )
    args = parser.parse_args()
    cases = tow_matrix(args.speeds, args.angles)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    plugins = {
        "usv::HullHydrostatics": args.hull_plugin,
        "usv::AppendageHydrodynamics": args.appendage_plugin,
    }
    metadata = prepare_rig(args.model, args.equilibrium, output, plugins, args.step)
    metadata["input_sha256"][str(Path(__file__).resolve())] = digest(__file__)
    (output / "binding.json").write_text(json.dumps(metadata, indent=2) + "\n")
    (output / "cases.json").write_text(json.dumps(cases, indent=2) + "\n")
    run_sweep(output, cases, metadata)


if __name__ == "__main__":
    main()
