#!/usr/bin/env python3
"""执行指定航行工况并保存性能遥测与判据。

Readiness-gated native Gazebo/ROS curves, manoeuvres, and endurance tests.
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
from project_paths import PROJECT_ROOT, WORKSPACE, tool_path
import argparse
import json
import math
import os
import signal
import subprocess
import time
import hashlib
import uuid
import atexit
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
from readiness import ReadinessWindow
from test_isolation import reserve_ros_domain

p = argparse.ArgumentParser()
p.add_argument("--case", required=True)
p.add_argument("--spec", type=Path, required=True)
p.add_argument("--output", type=Path, required=True)
a = p.parse_args()
all_specs = json.loads(a.spec.read_text())
spec = all_specs[a.case]
w = WORKSPACE
out = a.output.resolve()
out.mkdir(parents=True, exist_ok=True)
ros_domain, ros_domain_lease = reserve_ros_domain()
os.environ.update(
    GZ_PARTITION="usv_perf_" + a.case + "_" + uuid.uuid4().hex[:8],
    GZ_IP="127.0.0.1",
    ROS_DOMAIN_ID=str(ros_domain),
)
(out / "source_versions.json").write_text(
    json.dumps(
        {
            "partition": os.environ["GZ_PARTITION"],
            "ros_domain": os.environ["ROS_DOMAIN_ID"],
            "sha256": {
                str(f.resolve().relative_to(PROJECT_ROOT)): hashlib.sha256(
                    f.read_bytes()
                ).hexdigest()
                for f in [
                    w / "src/usv_control/scripts/boat_control.py",
                    w / "src/usv_simulation/src/HullHydrostatics.cc",
                    w / "src/usv_simulation/src/AzimuthPropulsion.cc",
                    w / "src/usv_description/config/resistance.json",
                    w / "src/usv_description/config/trial_load.json",
                    w / "src/usv_simulation/src/AppendageHydrodynamics.cc",
                    w / "src/usv_simulation/include/AppendageHydrodynamics.hh",
                    w / "src/usv_simulation/include/WrenchAudit.hh",
                    w / "src/usv_description/config/appendage_hydrodynamics.json",
                ]
            },
        },
        indent=2,
    )
)


def _load_runtime_dependencies_1():
    """在当前路径和运行环境配置完成后加载依赖，保持原导入时机。"""
    from gz.transport13 import Node
    from gz.msgs10.double_v_pb2 import Double_V
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.vector3d_pb2 import Vector3d
    import rclpy
    from std_msgs.msg import Float64, String
    from geometry_msgs.msg import Twist

    return (
        Node,
        Double_V,
        Boolean,
        Vector3d,
        rclpy,
        Float64,
        String,
        Twist,
    )


(
    Node,
    Double_V,
    Boolean,
    Vector3d,
    rclpy,
    Float64,
    String,
    Twist,
) = _load_runtime_dependencies_1()

world = out / "world.sdf"
subprocess.run(
    [
        "python3",
        str(tool_path("prepare_sydney.py")),
        "--profile",
        "speed15_concept",
        "--payload-kg",
        str(spec.get("payload_kg", 0)),
        "--fuel-l",
        str(spec.get("fuel_l", 60)),
        "--wave-gain",
        str(spec.get("wave_gain", 0)),
        "--power-mode",
        spec.get("power_mode", "normal"),
        "--mass-case",
        spec.get("mass_case", "nominal"),
        "--payload-center",
        *map(str, spec.get("payload_center", [0, 0, 0.6])),
        "--output",
        str(world),
    ],
    check=True,
    stdout=subprocess.DEVNULL,
)
tree = E.parse(world)
inc = tree.find("world/include[name='usv']")
bt = E.parse(inc.findtext("uri"))
bm = bt.find("model")
if spec.get("isolated_water"):
    for el in list(tree.find("world")):
        if (
            el.tag == "include"
            and el is not inc
            and el.findtext("name") != "Coast Waves"
        ):
            tree.find("world").remove(el)
        elif el.tag == "model":
            tree.find("world").remove(el)
if spec.get("disable_appendages"):
    for el in list(bm.findall("plugin[@name='usv::AppendageHydrodynamics']")):
        bm.remove(el)
if "appendage_coefficient_scale" in spec:
    ap = bm.find("plugin[@name='usv::AppendageHydrodynamics']")
    E.SubElement(ap, "coefficient_scale").text = str(
        spec["appendage_coefficient_scale"]
    )
if "stabilizer_coefficient_scale" in spec:
    fin = bm.find("plugin[@name='usv::PassiveStabilizer']")
    if fin is None:
        raise ValueError("Stabilizer sensitivity requested without stabilizer")
    E.SubElement(fin, "coefficient_scale").text = str(
        spec["stabilizer_coefficient_scale"]
    )
if "initial_yaw" in spec:
    pose = inc.findtext("pose").split()
    pose[5] = str(spec["initial_yaw"])
    inc.find("pose").text = " ".join(pose)

for link in bm.findall("link"):
    for sensor in list(link.findall("sensor")):
        if sensor.get("type") in ["camera", "gpu_lidar"]:
            link.remove(sensor)
hydro = bm.find("plugin[@name='usv::HullHydrostatics']")
for tag in [
    "linear_damping",
    "quadratic_damping",
    "angular_damping",
    "angular_quadratic_damping",
]:
    el = hydro.find(tag)
    el.text = " ".join(
        str(float(v) * spec.get("drag_scale", 1)) for v in el.text.split()
    )
for key, value in spec.get("hydro_overrides", {}).items():
    el = hydro.find(key)
    if el is None:
        el = E.SubElement(hydro, key)
    el.text = str(value)
if "step_size" in spec:
    tree.find("world/physics/max_step_size").text = str(spec["step_size"])
# Diagnostic overrides affect only this generated trial, never production config.
propulsion = bm.find("plugin[@name='usv::AzimuthPropulsion']")
for key, value in spec.get("propulsion_overrides", {}).items():
    el = propulsion.find(key)
    if el is None:
        el = E.SubElement(propulsion, key)
    el.text = str(value)
# Symmetric-mass counterfactual: average each rigid mass distribution with
# its y-reflection. Mass, x/z CG and diagonal inertias are retained.
if spec.get("symmetric_mass_diagnostic"):
    for link in bm.findall("link"):
        inertial = link.find("inertial")
        pose = inertial.findtext("pose").split()
        pose[1] = "0"
        inertial.find("pose").text = " ".join(pose)
        for key in ["ixy", "iyz"]:
            inertial.find("inertia/" + key).text = "0"
    fm = propulsion.find("fuel_mass")
    cg = fm.findtext("dry_cg").split()
    cg[1] = "0"
    fm.find("dry_cg").text = " ".join(cg)
    inertia = fm.findtext("dry_inertia").split()
    for j in [1, 3, 5, 7]:
        inertia[j] = "0"
    fm.find("dry_inertia").text = " ".join(inertia)
energy = bm.find("plugin[@name='usv::AzimuthPropulsion']/energy")
for key, value in spec.get("energy_overrides", {}).items():
    energy.find(key).text = str(value)
boat = out / "boat.sdf"
bt.write(boat, encoding="utf-8", xml_declaration=True)
inc.find("uri").text = str(boat)
tree.write(world, encoding="utf-8", xml_declaration=True)
node = Node()
rows = {
    k: []
    for k in [
        "propulsion/state",
        "energy/truth",
        "actuators/state",
        "environment/state",
        "fuel_mass/state",
        "hydrodynamics/resistance",
        "wave_response/state",
        "hydrostatics",
        "audit/hull",
        "audit/propulsion",
    ]
}
if bm.find("plugin[@name='usv::AppendageHydrodynamics']") is not None:
    rows["hydrodynamics/appendages"] = []
    rows["audit/appendages"] = []
if bm.find("plugin[@name='usv::PassiveStabilizer']") is not None:
    rows["stabilizer/hydrodynamics/appendages"] = []
    rows["stabilizer/audit/appendages"] = []
for topic in rows:

    def record_message_1(msg, k=topic):
        """记录收到的消息，沿用当前订阅所绑定的状态容器。

        Args:
            msg: 当前订阅接收到的消息，类型由订阅接口确定。
            k: 回调创建时绑定的主题或状态类别键。
        """
        return rows[k].append(list(msg.data))

    node.subscribe(Double_V, "/usv/" + topic, record_message_1)
gen = node.advertise("/usv/energy/generator_enable", Boolean)
wind = node.advertise("/vrx/wind/velocity", Vector3d)
current = node.advertise("/vrx/current/velocity", Vector3d)
rclpy.init()
ros = rclpy.create_node("vessel_performance_test")
held_throttle = ros.create_publisher(
    Float64, "/usv/actuators/heading_hold_throttle", 10
)
th = ros.create_publisher(Float64, "/usv/actuators/throttle", 10)
steer = ros.create_publisher(Float64, "/usv/actuators/steering", 10)
vel = ros.create_publisher(Twist, "/usv/cmd_vel", 10)
health = []
gate = ReadinessWindow()
ready = False


def on_health(msg):
    """解析控制健康状态并更新连续就绪判断。

    Args:
        msg: 包含 JSON 健康状态的 ROS 消息
    """
    global ready
    value = json.loads(msg.data)
    if value.get("gz_partition") != os.environ["GZ_PARTITION"]:
        return
    health.append(value)
    ready = gate.update(value, time.monotonic())


ros.create_subscription(String, "/usv/control/health", on_health, 10)
procs = []
files = []
cleaned = False


def cleanup():
    """幂等清理性能试验启动的进程与资源。"""
    global cleaned
    if cleaned:
        return
    cleaned = True
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    for proc in reversed(procs):
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
    subprocess.run(
        [
            "python3",
            str(tool_path("stop_test_partition.py")),
            os.environ["GZ_PARTITION"],
            "--exclude-pid",
            str(os.getpid()),
        ],
        stdout=subprocess.DEVNULL,
    )
    for f in files:
        f.close()


atexit.register(cleanup)


def interrupted(signum, frame):
    """将系统信号转为试验异常以触发清理。

    Args:
        signum: 信号编号
        frame: 信号发生时的 Python 栈帧
    """
    raise RuntimeError("Interrupted signal " + str(signum))


signal.signal(signal.SIGTERM, interrupted)
signal.signal(signal.SIGINT, interrupted)
for name, args in [
    (
        "gazebo",
        ["bash", str(tool_path("run_hull_trial.sh")), "world:=" + str(world)],
    ),
    (
        "interfaces",
        [
            "bash",
            str(tool_path("run_interfaces.sh")),
            "speed_scale:=4.3",
            *spec.get("interfaces_args", []),
        ],
    ),
]:
    f = (out / (name + ".log")).open("w")
    files.append(f)
    procs.append(
        subprocess.Popen(
            args, stdout=f, stderr=subprocess.STDOUT, start_new_session=True
        )
    )
stages = spec["stages"]
stage = 0
stage_start = None
start_sim = None
stage_log = []
zigzag_reference = None
stage_origin = None
wall = time.monotonic()
failure = None
last_progress = 0
bridge_pid = None
resumed = False
last_retry = wall
transport_retries = []
try:
    while time.monotonic() - wall < sum(x["duration"] for x in stages) * 3 + 90:
        if any(x.poll() is not None for x in procs):
            raise RuntimeError("Launch process exited")
        rclpy.spin_once(ros, timeout_sec=0.01)
        now = time.monotonic()
        gate.fresh(now)
        if now - last_progress > 1:
            (out / "progress.json").write_text(
                json.dumps(
                    {
                        "stage": stage,
                        "sim_s": (
                            rows["propulsion/state"][-1][0]
                            if rows["propulsion/state"]
                            else 0
                        ),
                        "wall_s": now - wall,
                        "health": health[-1] if health else None,
                        "missing_streams": [k for k, v in rows.items() if not v],
                    }
                )
            )
            last_progress = now
        if start_sim is None:
            if now - last_retry > 3:
                last_retry = now
                for topic, values in rows.items():
                    if not values:
                        node.unsubscribe("/usv/" + topic)

                        def record_message_2(msg, k=topic):
                            """记录收到的消息，沿用当前订阅所绑定的状态容器。

                            Args:
                                msg: 当前订阅接收到的消息，类型由订阅接口确定。
                                k: 回调创建时绑定的主题或状态类别键。
                            """
                            return rows[k].append(list(msg.data))

                        node.subscribe(
                            Double_V,
                            "/usv/" + topic,
                            record_message_2,
                        )
                        transport_retries.append({"wall_s": now - wall, "topic": topic})
            if now - wall > spec.get("startup_timeout", 60):
                raise RuntimeError("Readiness timeout; no timed trial was started")
            if not (ready and gate.fresh(now) and all(rows.values())):
                continue
            start_sim = rows["propulsion/state"][-1][0]
            stage_start = start_sim
            (out / "startup.json").write_text(
                json.dumps(
                    {
                        "ready": True,
                        "wall_s": now - wall,
                        "sim_s": start_sim,
                        "health": health[-1],
                        "transport_retries": transport_retries,
                    },
                    indent=2,
                )
            )
        t = rows["propulsion/state"][-1][0]
        elapsed = t - start_sim
        s = stages[stage]
        if stage_origin is None:
            stage_origin = rows["propulsion/state"][-1][12:15]
        if not gate.fresh(now):
            raise RuntimeError("Controller health stream became stale")
        if not health[-1]["ready"] and spec.get("kind") != "sensor_dropout":
            raise RuntimeError("Readiness lost: " + str(health[-1]["missing"]))
        if spec.get("kind") == "sensor_dropout":
            if elapsed > 12 and bridge_pid is None:
                for child in (
                    Path(f"/proc/{procs[1].pid}/task/{procs[1].pid}/children")
                    .read_text()
                    .split()
                ):
                    if (
                        Path(f"/proc/{child}/cmdline")
                        .read_bytes()
                        .split(b"\0")[0]
                        .endswith(b"/parameter_bridge")
                    ):
                        bridge_pid = int(child)
                        os.kill(bridge_pid, signal.SIGSTOP)
                        break
                if bridge_pid is None:
                    raise RuntimeError("Owned ROS bridge not found")
            if elapsed > 25 and bridge_pid is not None and not resumed:
                os.kill(bridge_pid, signal.SIGCONT)
                resumed = True
        if spec.get("kind") == "command_dropout" and 12 < elapsed < 25:
            pass  # Deliberately stop both manual publishers; keep observing health.
        elif s.get("mode") == "fixed_heading":
            held_throttle.publish(Float64(data=float(s["throttle"])))
        elif s.get("mode", "direct") == "velocity":
            msg = Twist()
            msg.linear.x = float(s["speed"])
            msg.angular.z = float(s.get("yaw_rate", 0.0))
            vel.publish(msg)
        else:
            th.publish(Float64(data=float(s.get("throttle", 0))))
            steer.publish(Float64(data=math.radians(s.get("steering_deg", 0))))
        generator = spec.get("generator_on", False)
        if spec.get("generator_cycle"):
            generator = 60 < elapsed < 220 or 300 < elapsed < 500
        gen.publish(Boolean(data=generator))
        wind.publish(
            Vector3d(**dict(zip(["x", "y", "z"], spec.get("wind", [0, 0, 0]))))
        )
        current.publish(
            Vector3d(**dict(zip(["x", "y", "z"], spec.get("current", [0, 0, 0]))))
        )
        reached_heading = False
        if "until_heading_deg" in s:
            if zigzag_reference is None:
                zigzag_reference = rows["propulsion/state"][-1][14]
            heading_delta = math.atan2(
                math.sin(rows["propulsion/state"][-1][14] - zigzag_reference),
                math.cos(rows["propulsion/state"][-1][14] - zigzag_reference),
            )
            target_heading = math.radians(s["until_heading_deg"])
            reached_heading = heading_delta * math.copysign(1, target_heading) >= abs(
                target_heading
            )
            if t - stage_start >= s["duration"] and not reached_heading:
                raise RuntimeError("Zigzag heading execution timed out: " + s["name"])
        reached_distance = False
        if "until_along_m" in s:
            x, y, yaw = stage_origin
            position = rows["propulsion/state"][-1]
            along = (position[12] - x) * math.cos(yaw) + (position[13] - y) * math.sin(
                yaw
            )
            target = s["until_along_m"]
            reached_distance = along * math.copysign(1, target) >= abs(target)
            if t - stage_start >= s["duration"] and not reached_distance:
                raise RuntimeError("Along-track distance timed out: " + s["name"])
        if t - stage_start >= s["duration"] or reached_heading or reached_distance:
            stage_log.append(
                dict(
                    s,
                    index=stage,
                    start_sim=stage_start,
                    end_sim=t,
                    distance_triggered=reached_distance,
                    stage_origin=stage_origin,
                    heading_triggered=reached_heading,
                    heading_reference_rad=zigzag_reference,
                )
            )
            stage_origin = None
            stage += 1
            stage_start = t
            if stage == len(stages):
                break
        time.sleep(0.02)
    if stage < len(stages):
        raise RuntimeError("Timed trial did not finish")
except Exception as exc:
    failure = str(exc)
finally:
    if bridge_pid and not resumed:
        try:
            os.kill(bridge_pid, signal.SIGCONT)
        except ProcessLookupError:
            pass
    for _ in range(3):
        th.publish(Float64(data=0.0))
        steer.publish(Float64(data=0.0))
        rclpy.spin_once(ros, timeout_sec=0.01)
    cleanup()
    ros.destroy_node()
    rclpy.shutdown()
for k, v in rows.items():
    np.savetxt(out / (k.replace("/", "_") + ".csv"), v, delimiter=",")
(out / "health.json").write_text(json.dumps(health, indent=2))
(out / "stages.json").write_text(json.dumps(stage_log, indent=2))
(out / "run_status.json").write_text(
    json.dumps(
        {
            "case": a.case,
            "spec": spec,
            "finished_stages": stage,
            "start_sim": start_sim,
            "failure": failure,
            "bridge_resumed": resumed,
        },
        indent=2,
    )
)


def _load_runtime_dependencies_2():
    """在当前路径和运行环境配置完成后加载依赖，保持原导入时机。"""
    from performance_analysis import analyze

    return (analyze,)


(analyze,) = _load_runtime_dependencies_2()

r = analyze(out)
print(json.dumps(r, indent=2))
raise SystemExit(0 if r["passed"] else 1)
