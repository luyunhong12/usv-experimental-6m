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
import os
import time
import signal
import subprocess
import json
import math
import copy
import argparse
from pathlib import Path
import xml.etree.ElementTree as E

W = WORKSPACE
parser = argparse.ArgumentParser()
parser.add_argument("--output", type=Path, default=W / "log/contact_directions")
parser.add_argument(
    "--profile", choices=["baseline", "speed15_concept"], default="baseline"
)
args = parser.parse_args()
out = args.output.resolve()
out.mkdir(parents=True, exist_ok=True)
os.environ["GZ_PARTITION"] = "usv_contact_directions"


def _load_runtime_dependencies_1():
    """在当前路径和运行环境配置完成后加载依赖，保持原导入时机。"""
    from gz.transport13 import Node
    from gz.msgs10.pose_v_pb2 import Pose_V

    return (
        Node,
        Pose_V,
    )


(
    Node,
    Pose_V,
) = _load_runtime_dependencies_1()

world = E.parse(W / "src/usv_simulation/worlds/usv_hull_trial.sdf")
ww = world.getroot().find("world")
ww.set("name", "contact_directions")
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
    for x in list(link_element.findall("visual")):
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
node = Node()
rows = {}
callbacks = []
for idx, (name, rot, xy, low, high) in enumerate(cases):
    offset = idx * 20.0
    m = copy.deepcopy(base)
    m.set("name", "fixed_" + name)
    E.SubElement(m, "pose").text = " ".join(map(str, [offset, 0, 0, *rot]))
    ww.append(m)
    b = E.SubElement(ww, "model", name="probe_" + name)
    E.SubElement(b, "pose").text = f"{offset+xy[0]} {xy[1]} {high+.6} 0 0 0"
    link_element = E.SubElement(b, "link", name="ball")
    i = E.SubElement(link_element, "inertial")
    E.SubElement(i, "mass").text = "1"
    ii = E.SubElement(i, "inertia")
    for k in ["ixx", "iyy", "izz"]:
        E.SubElement(ii, k).text = ".00025"
    g = E.SubElement(E.SubElement(link_element, "collision", name="ball"), "geometry")
    E.SubElement(E.SubElement(g, "sphere"), "radius").text = ".025"
    p = E.SubElement(
        b,
        "plugin",
        filename="gz-sim-pose-publisher-system",
        name="gz::sim::systems::PosePublisher",
    )
    E.SubElement(p, "publish_model_pose").text = "true"
    E.SubElement(p, "use_pose_vector_msg").text = "true"
    E.SubElement(p, "update_frequency").text = "50"
    rows[name] = []

    def callback(msg, key=name):
        """记录指定接触探针的三维位置。

        Args:
            msg: 包含多物体位姿的消息
            key: 当前探针对应的工况名称
        """
        for pose in msg.pose:
            if pose.name == "probe_" + key:
                rows[key].append([pose.position.x, pose.position.y, pose.position.z])

    callbacks.append(callback)
    node.subscribe(Pose_V, "/model/probe_" + name + "/pose", callback)
f = out / "contact_directions.sdf"
world.write(f, encoding="utf-8", xml_declaration=True)
with (out / "launch.log").open("w") as log:
    proc = subprocess.Popen(
        ["gz", "sim", "-s", "-r", str(f)],
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    try:
        start = time.monotonic()
        while time.monotonic() - start < 45:
            if proc.poll() is not None:
                break
            if all(len(x) > 350 for x in rows.values()):
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
(out / "result.json").write_text(json.dumps(result, indent=2))
(out / "trajectories.json").write_text(json.dumps(rows))
print(json.dumps(result, indent=2))
raise SystemExit(0 if all(x["passed"] for x in result.values()) else 1)
