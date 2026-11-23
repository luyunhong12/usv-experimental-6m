"""验证碰撞接触的方向与几何位置。

Rigid-geometry contact checks: fixed hull orientations and free falling probes.
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
from trial_evidence import execution_snapshot, hashes
import os
import time
import signal
import subprocess
import json
import math
import copy
import argparse
import uuid
import re
from pathlib import Path
import xml.etree.ElementTree as E

W = WORKSPACE
parser = argparse.ArgumentParser()
parser.add_argument("--output", type=Path, default=W / "log/contact_directions")
parser.add_argument(
    "--profile", choices=["baseline", "speed15_concept"], default="baseline"
)
parser.add_argument(
    "--step-size",
    type=float,
    default=None,
    help="诊断用物理步长，单位 s；默认沿用生产场景",
)
parser.add_argument(
    "--collision-detector",
    choices=["fcl", "ode", "bullet", "dart"],
    default=None,
    help="诊断碰撞检测后端；默认沿用生产场景",
)
parser.add_argument(
    "--free-probes", action="store_true", help="仅诊断：取消所有探针法向导轨，观察滚落"
)
parser.add_argument(
    "--drop-height", type=float, default=0.6, help="探针球心距表面初始高度，单位 m"
)
parser.add_argument("--probe-mass", type=float, default=1.0, help="均质球质量，单位 kg")
parser.add_argument(
    "--contact-erp",
    type=float,
    default=None,
    help="诊断用接触数值回正比例，默认沿用生产配置",
)
parser.add_argument(
    "--correction-velocity",
    type=float,
    default=None,
    help="诊断用最大回正速度，单位 m/s",
)
parser.add_argument(
    "--case", default=None, help="只运行指定接触位置；逐位置运行可隔离多发布器发现问题"
)
args = parser.parse_args()
if args.step_size is not None and not 0 < args.step_size <= 0.01:
    parser.error("step-size must be in (0, 0.01]")
if not 0.03 <= args.drop_height <= 3 or not 0.1 <= args.probe_mass <= 10:
    parser.error("drop-height must be 0.03..3 m and probe-mass 0.1..10 kg")
if (args.contact_erp is None) != (args.correction_velocity is None):
    parser.error("contact-erp and correction-velocity must be supplied together")
if args.contact_erp is not None and not (
    0 < args.contact_erp <= 1 and 0 < args.correction_velocity <= 1
):
    parser.error("contact correction parameters must be in (0,1]")
out = args.output.resolve()
out.mkdir(parents=True, exist_ok=False)
os.environ["GZ_PARTITION"] = "usv_contact_" + uuid.uuid4().hex[:10]
plugin_path = W / "install/usv_physics/lib"
if not (plugin_path / "libUsvPhysics.so").is_file():
    raise RuntimeError("Build the production physics plugin before contact tests")
os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"] = (
    str(plugin_path) + os.pathsep + os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
)


def _load_runtime_dependencies_1():
    """在当前路径和运行环境配置完成后加载依赖，保持原导入时机。"""
    from gz.transport13 import Node
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.world_control_pb2 import WorldControl
    from gz.msgs10.boolean_pb2 import Boolean

    return (
        Node,
        Pose_V,
        WorldControl,
        Boolean,
    )


(
    Node,
    Pose_V,
    WorldControl,
    Boolean,
) = _load_runtime_dependencies_1()

snapshot = execution_snapshot()
validation_sources = [Path(__file__), W.parent / "tools/modeling/contact_checks.py"]
source_hashes = hashes(validation_sources)
(out / "source_versions.json").write_text(
    json.dumps({"execution": snapshot, "validation_sources": source_hashes}, indent=2)
)
world = E.parse(W / "src/usv_simulation/worlds/usv_hull_trial.sdf")
ww = world.getroot().find("world")
ww.set("name", "contact_directions")
physics_plugin = ww.find("plugin[@name='gz::sim::systems::Physics']")
if args.contact_erp is not None:
    physics_plugin.find("rigid_contact_erp").text = str(args.contact_erp)
    physics_plugin.find("rigid_contact_max_correction_velocity").text = str(
        args.correction_velocity
    )
expected_erp = float(physics_plugin.find("rigid_contact_erp").text)
expected_correction = float(
    physics_plugin.find("rigid_contact_max_correction_velocity").text
)
if args.step_size is not None:
    ww.find("physics/max_step_size").text = str(args.step_size)
if args.collision_detector is not None:
    ww.find("physics/dart/collision_detector").text = args.collision_detector
for x in list(ww):
    if x.tag in ["model", "include", "gui"] or (
        x.tag == "plugin" and "Physics" not in x.get("name", "")
    ):
        ww.remove(x)
base = (
    E.parse(W / "src/usv_description/models/usv_azimuth_trial.sdf")
    .getroot()
    .find("model")
)
if args.profile == "speed15_concept":
    from scenario_profiles import apply_speed15

    apply_speed15(base)
for x in list(base.findall("plugin")):
    base.remove(x)
for link_element in base.findall("link"):
    for x in list(link_element.findall("visual")) + list(
        link_element.findall("sensor")
    ):
        link_element.remove(x)
E.SubElement(base, "static").text = "true"
cases = [
    # 船体原点上方是雷达支架；在无遮挡甲板区域测试原高度门槛。
    ("deck", [0, 0, 0], [1.0, 0.5], 0.85, 1.05),
    ("bottom", [math.pi, 0, 0], [0, 0], 0, 0.12),
    ("port", [math.pi / 2, 0, 0], [0, -0.5], 0.9, 1.2),
    ("starboard", [-math.pi / 2, 0, 0], [0, 0.5], 0.9, 1.2),
    ("stern", [0, math.pi / 2, 0], [0.5, 0], 3.0, 3.8),
    ("propeller_hub", [0, math.pi / 2, 0], [-0.143, 0], 3.7, 3.95),
    # 新鳍位于船底外：翻转后探针应落在鳍端，而非穿透后落到船壳。
    ("stabilizer_tip", [math.pi, 0, 0], [0.5, 0], 0.393, 0.393),
    ("stabilizer_side", [math.pi / 2, 0, 0], [0.5, 0.2], 0.009, 0.009),
]
if args.case is not None:
    cases = [case for case in cases if case[0] == args.case]
    if not cases:
        parser.error("Unknown contact case")
node = Node()
rows = {}
times = {}
step_size = float(ww.find("physics/max_step_size").text)
callbacks = []
for idx, (name, rot, xy, low, high) in enumerate(cases):
    offset = idx * 20.0
    m = copy.deepcopy(base)
    m.set("name", "fixed_" + name)
    E.SubElement(m, "pose").text = " ".join(map(str, [offset, 0, 0, *rot]))
    ww.append(m)
    b = E.SubElement(ww, "model", name="probe_" + name)
    E.SubElement(b, "pose").text = (
        f"{offset+xy[0]} {xy[1]} {high+args.drop_height} 0 0 0"
    )
    link_element = E.SubElement(b, "link", name="ball")
    i = E.SubElement(link_element, "inertial")
    E.SubElement(i, "mass").text = str(args.probe_mass)
    ii = E.SubElement(i, "inertia")
    for k in ["ixx", "iyy", "izz"]:
        E.SubElement(ii, k).text = str(0.4 * args.probe_mass * 0.025**2)
    g = E.SubElement(E.SubElement(link_element, "collision", name="ball"), "geometry")
    E.SubElement(E.SubElement(g, "sphere"), "radius").text = ".025"
    if not args.free_probes:
        joint = E.SubElement(b, "joint", name="normal_guide", type="prismatic")
        E.SubElement(joint, "parent").text = "world"
        E.SubElement(joint, "child").text = "ball"
        axis = E.SubElement(joint, "axis")
        E.SubElement(axis, "xyz").text = "0 0 1"
        limit = E.SubElement(axis, "limit")
        E.SubElement(limit, "lower").text = "-1000"
        E.SubElement(limit, "upper").text = "1000"
    p = E.SubElement(
        b,
        "plugin",
        filename="gz-sim-pose-publisher-system",
        name="gz::sim::systems::PosePublisher",
    )
    E.SubElement(p, "publish_model_pose").text = "true"
    E.SubElement(p, "use_pose_vector_msg").text = "true"
    E.SubElement(p, "update_frequency").text = str(1 / step_size)
    rows[name] = []
    times[name] = []

    def callback(msg, key=name):
        """记录指定接触探针的三维位置。

        Args:
            msg: 包含多物体位姿的消息
            key: 当前探针对应的工况名称
        """
        for pose in msg.pose:
            if pose.name == "probe_" + key:
                rows[key].append([pose.position.x, pose.position.y, pose.position.z])
                times[key].append(pose.header.stamp.sec + pose.header.stamp.nsec * 1e-9)

    callbacks.append(callback)
    if not node.subscribe(Pose_V, "/model/probe_" + name + "/pose", callback):
        raise RuntimeError("Unable to subscribe to probe " + name)
f = out / "contact_directions.sdf"
world.write(f, encoding="utf-8", xml_declaration=True)
with (out / "launch.log").open("w") as log:
    proc = subprocess.Popen(
        ["gz", "sim", "-v", "4", "-s", str(f)],
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    try:
        start = time.monotonic()
        # 暂停启动，微步完成发现握手；不可错过首次落球接触再补订阅。
        for attempt in range(40):
            if proc.poll() is not None:
                break
            if all(times.values()):
                break
            if attempt and attempt % 10 == 0:
                for index, case in enumerate(cases):
                    name = case[0]
                    if not times[name]:
                        topic = "/model/probe_" + name + "/pose"
                        node.unsubscribe(topic)
                        if not node.subscribe(Pose_V, topic, callbacks[index]):
                            raise RuntimeError("Unable to retry probe subscription")
            node.request(
                "/world/contact_directions/control",
                WorldControl(pause=True, multi_step=1),
                WorldControl,
                Boolean,
                500,
            )
            time.sleep(0.1)
        if not all(times.values()):
            (out / "startup_failure.json").write_text(
                json.dumps(
                    {
                        "samples": {k: len(v) for k, v in times.items()},
                        "last_sim_s": {
                            k: v[-1] if v else None for k, v in times.items()
                        },
                    }
                )
            )
            raise RuntimeError("Probe discovery failed before impact")
        release_times = {name: values[-1] for name, values in times.items()}
        released = False
        for attempt in range(8):
            node.request(
                "/world/contact_directions/control",
                WorldControl(pause=False),
                WorldControl,
                Boolean,
                500,
            )
            time.sleep(0.2)
            # RPC 确认可能超时；推进量必须超过握手可能排队的全部微步。
            released = all(
                values[-1] > release_times[name] + max(0.2, 41 * step_size)
                for name, values in times.items()
            )
            if released:
                break
        if not released:
            raise RuntimeError("Unable to observe release of paused contact trial")
        while time.monotonic() - start < 120:
            if proc.poll() is not None:
                break
            if all(x and x[-1] - x[0] >= 8 for x in times.values()):
                break
            time.sleep(0.2)
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()


def _load_runtime_dependencies_2():
    """在当前路径和运行环境配置完成后加载依赖，保持原导入时机。"""
    from contact_checks import evaluate

    return (evaluate,)


(evaluate,) = _load_runtime_dependencies_2()

result = evaluate(
    rows,
    cases,
    json.loads((W / "src/usv_description/config/hull_sealed.json").read_text()),
)
if not args.free_probes:
    from contact_checks import evaluate_fin_precision
    from passive_stabilizer import configuration, geometry

    vertices, faces, _ = geometry(configuration())
    for idx, (name, rot, xy, low, high) in enumerate(cases):
        if name.startswith("stabilizer_"):
            result[name] = evaluate_fin_precision(
                rows[name],
                times[name],
                idx,
                rot,
                vertices,
                faces,
                step_size,
                args.drop_height,
            )
launch_log = re.sub(
    r"\x1b\[[0-9;]*m", "", (out / "launch.log").read_text(errors="replace")
)
result["physics_backend"] = {
    "passed": "Failed to load system plugin" not in launch_log
    and "Loaded system [gz::sim::systems::Physics]" in launch_log
    and f"DART rigid contact ERP={expected_erp:g} correction_limit_m_s={expected_correction:g}"
    in launch_log,
    "requirement": "Production UsvPhysics loaded and explicit DART settings read back",
}
result["version_binding"] = {
    "passed": execution_snapshot() == snapshot
    and hashes(validation_sources) == source_hashes,
    "requirement": "Runtime inputs, installed binaries and validation sources unchanged",
}
(out / "sample_times.json").write_text(json.dumps(times))
(out / "settings.json").write_text(json.dumps(vars(args), default=str, indent=2))
(out / "result.json").write_text(json.dumps(result, indent=2))
(out / "trajectories.json").write_text(json.dumps(rows))
print(json.dumps(result, indent=2))
raise SystemExit(0 if all(x["passed"] for x in result.values()) else 1)
