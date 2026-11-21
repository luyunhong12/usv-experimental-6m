#!/usr/bin/env python3
"""检查推进器的力、转矩和功率响应。

Actual Gazebo integration evidence, independent partitions per manoeuvre.
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
from project_paths import WORKSPACE, tool_path
import argparse
import os
import time
import signal
import subprocess
import json
import math
import numpy as np
import xml.etree.ElementTree as E

p = argparse.ArgumentParser()
p.add_argument(
    "--case", choices=["forward", "reverse", "left", "right", "reversal"], required=True
)
p.add_argument("--sydney", action="store_true")
p.add_argument(
    "--output-dir", type=_ProjectPath, help="本次独立证据目录；不覆盖历史运行"
)
a = p.parse_args()
W = WORKSPACE
out = a.output_dir or (
    W / ("log/sydney_validation" if a.sydney else "log/propulsion_validation") / a.case
)
out.mkdir(parents=True, exist_ok=True)
os.environ["GZ_PARTITION"] = (
    ("usv_sydney_" if a.sydney else "usv_propulsion_") + a.case + "_" + str(os.getpid())
)
os.environ.setdefault("GZ_IP", "127.0.0.1")


def _load_runtime_dependencies():
    """在当前路径和运行环境配置完成后加载依赖，保持原导入时机。"""
    from gz.transport13 import Node
    from gz.msgs10.double_pb2 import Double
    from gz.msgs10.double_v_pb2 import Double_V
    from gz.msgs10.model_pb2 import Model
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.contacts_pb2 import Contacts
    from PIL import Image as PILImage

    return (
        Node,
        Double,
        Double_V,
        Model,
        Image,
        Contacts,
        PILImage,
    )


(
    Node,
    Double,
    Double_V,
    Model,
    Image,
    Contacts,
    PILImage,
) = _load_runtime_dependencies()

rows = []
joints = []
n = Node()


def record_message_1(m):
    """记录收到的消息，沿用当前订阅所绑定的状态容器。

    Args:
        m: 当前订阅接收到的消息，类型由订阅接口确定。
    """
    return rows.append(list(m.data))


n.subscribe(Double_V, "/usv/propulsion/state", record_message_1)
images = []
mass = []
contacts = []
contact_frames = []
if a.sydney:

    def capture_first_image_2(m):
        """满足阶段条件时保存首帧图像，保留原有筛选条件。

        Args:
            m: 当前订阅接收到的消息，类型由订阅接口确定。
        """
        return images.append(m) if not images and rows and (rows[-1][0] > 1) else None

    n.subscribe(
        Image,
        "/usv/sydney/inspection",
        capture_first_image_2,
    )

    def record_message_3(m):
        """记录收到的消息，沿用当前订阅所绑定的状态容器。

        Args:
            m: 当前订阅接收到的消息，类型由订阅接口确定。
        """
        return mass.append(list(m.data))

    n.subscribe(Double_V, "/usv/fuel_mass/state", record_message_3)

    def contact(m):
        """统计接触帧并保存非空接触详情。

        Args:
            m: Gazebo 接触消息
        """
        contact_frames.append(len(m.contact))
        if m.contact:
            contacts.append(str(m))

    n.subscribe(Contacts, "/usv/sydney/contacts", contact)


def joint(m):
    """记录转向关节的仿真时刻、角度与角速度。

    Args:
        m: 含关节状态的模型消息
    """
    for j in m.joint:
        if j.name == "steering":
            joints.append(
                [rows[-1][0] if rows else 0, j.axis1.position, j.axis1.velocity]
            )


n.subscribe(Model, "/usv/joint_state", joint)
t = s = None
launcher = (
    ["bash", str(tool_path("run_sydney.sh")), "headless:=True"]
    if a.sydney
    else ["bash", str(tool_path("run_hull_trial.sh"))]
)
if a.sydney:
    subprocess.run(["python3", str(tool_path("prepare_sydney.py"))], check=True)
    tree = E.parse(W / "src/usv_simulation/worlds/usv_sydney.sdf")
    world = tree.getroot().find("world")
    boat = E.parse(W / "src/usv_description/models/usv_azimuth_trial.sdf")
    hull = boat.getroot().find("model/link[@name='hull']")
    cs = E.SubElement(hull, "sensor", name="validation_contacts", type="contact")
    E.SubElement(cs, "always_on").text = "true"
    E.SubElement(cs, "update_rate").text = "20"
    E.SubElement(cs, "topic").text = "/usv/sydney/contacts"
    ct = E.SubElement(cs, "contact")
    E.SubElement(ct, "topic").text = "/usv/sydney/contacts"
    for collision in hull.findall("collision"):
        E.SubElement(ct, "collision").text = collision.get("name")
    boat_path = out / "boat.sdf"
    boat.write(boat_path, encoding="utf-8", xml_declaration=True)
    for inc in world.findall("include"):
        if inc.find("uri").text.endswith("usv_azimuth_trial.sdf"):
            inc.find("uri").text = str(boat_path)
    camera = E.SubElement(world, "model", name="inspection_camera")
    E.SubElement(camera, "static").text = "true"
    E.SubElement(camera, "pose").text = "-468 168 8 0 0.40 2.35619449"
    link = E.SubElement(camera, "link", name="camera")
    sensor = E.SubElement(link, "sensor", name="camera", type="camera")
    E.SubElement(sensor, "always_on").text = "true"
    E.SubElement(sensor, "update_rate").text = "0.5"
    E.SubElement(sensor, "topic").text = "/usv/sydney/inspection"
    cam = E.SubElement(sensor, "camera")
    E.SubElement(cam, "horizontal_fov").text = "1.1"
    im = E.SubElement(cam, "image")
    for key, val in [("width", "1200"), ("height", "800"), ("format", "R8G8B8")]:
        E.SubElement(im, key).text = val
    clip = E.SubElement(cam, "clip")
    E.SubElement(clip, "near").text = ".1"
    E.SubElement(clip, "far").text = "3000"
    scene = out / "usv_sydney.sdf"
    tree.write(scene, encoding="utf-8", xml_declaration=True)
    launcher.append("world:=" + str(scene))
log = (out / "launch.log").open("w")
proc = subprocess.Popen(
    launcher, cwd=W, stdout=log, stderr=subprocess.STDOUT, start_new_session=True
)
try:
    start = time.monotonic()
    while time.monotonic() - start < (240 if a.sydney else 85) and proc.poll() is None:
        sim = rows[-1][0] if rows else 0
        if sim >= 40:
            break
        if not rows:
            time.sleep(0.04)
            continue
        if t is None:
            t = n.advertise("/usv/propulsion/throttle", Double)
            s = n.advertise("/usv/steering/angle", Double)
        throttle = (
            0
            if sim < 2
            else (
                -0.8
                if a.case == "reverse" or (a.case == "reversal" and sim >= 15)
                else 0.8
            )
        )
        angle = {"left": -20, "right": 20}.get(a.case, 0)
        # Deliberately stop heartbeat at 28 s: watchdog must disengage motor.
        if sim < 28:
            t.publish(Double(data=throttle))
            s.publish(Double(data=math.radians(angle)))
        time.sleep(0.04)
finally:
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=6)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
    log.close()
d = np.array(rows)
np.savetxt(
    out / "telemetry.csv",
    d,
    delimiter=",",
    header="time,command,rpm,reference_rpm,thrust_N,motor_torque_Nm,water_torque_Nm,shaft_power_W,wet_fraction,axial_velocity,body_u,body_v,x,y,yaw,roll,pitch",
    comments="",
)
np.savetxt(
    out / "joints.csv",
    np.array(joints),
    delimiter=",",
    header="time,angle_rad,rate_rad_s",
    comments="",
)
r = {"case": a.case, "passed": False, "samples": len(d)}
if a.sydney:
    np.savetxt(out / "fuel_mass.csv", np.array(mass), delimiter=",")
    if images:
        im = images[0]
        rgb = (
            np.frombuffer(im.data, dtype=np.uint8)
            .reshape(im.height, im.step)[:, : im.width * 3]
            .reshape(im.height, im.width, 3)
        )
        PILImage.fromarray(rgb).save(out / "sydney_view.png")
    r.update(
        scene="Sydney Regatta",
        wave_gain=0,
        wind_m_s=0,
        render_captured=bool(images),
        native_mass_samples=len(mass),
    )
    r.update(
        contact_message_count=len(contact_frames), hull_contact_events=len(contacts)
    )
    (out / "contacts.txt").write_text("\n".join(contacts))
if len(d) and d[-1, 0] >= 39:
    d[:, 14] = np.unwrap(d[:, 14])
    driven = d[(d[:, 0] > 24) & (d[:, 0] < 27)]
    tail = d[d[:, 0] > 30]
    u = float(np.mean(driven[:, 10]))
    yaw = float(driven[-1, 14])
    early = d[(d[:, 0] > 10) & (d[:, 0] < 14)]
    checks = {
        "coast_slowdown": bool(abs(d[-1, 10]) < abs(u)),
        "finite": bool(np.isfinite(d).all()),
        "power_limited": bool(np.max(abs(d[:, 7])) <= 5040.01),
        "torque_limited": bool(np.max(abs(d[:, 5])) <= 60.01),
        "attitude_bounded": bool(np.max(abs(d[:, 15:17])) < 0.2),
        "watchdog_off": bool(len(tail) and np.max(abs(tail[:, [1, 5]])) < 1e-9),
        "motion_direction": bool(
            u < -0.2 if a.case in ["reverse", "reversal"] else u > 0.2
        ),
    }
    if a.sydney:
        checks["render_captured"] = bool(images)
        checks["native_mass_ack"] = bool(mass and np.isfinite(np.array(mass)).all())
        checks["spawn_in_sydney"] = bool(
            np.linalg.norm(d[0, 12:14] - [-480, 180]) < 0.1
        )
        checks["no_hull_contact"] = not contacts
    if a.case in ["left", "right"]:
        jt = np.array(joints)
        jd = jt[(jt[:, 0] > 24) & (jt[:, 0] < 27)] if len(jt) else []
        actual = float(np.mean(jd[:, 1])) if len(jd) else None
        checks["steering_tracks"] = (
            actual is not None
            and abs(actual - math.radians({"left": -20, "right": 20}[a.case])) < 0.04
        )
        r["driven_steering_deg"] = math.degrees(actual) if actual is not None else None
    if a.case == "left":
        checks["turn_direction"] = yaw > 0.15
    if a.case == "right":
        checks["turn_direction"] = yaw < -0.15
    if a.case == "reversal":
        checks["both_directions"] = bool(
            np.mean(early[:, 10]) > 0.2 and np.min(driven[:, 2]) < 0
        )
        rev = d[(d[:, 0] >= 15) & (d[:, 0] < 28)]
        negative_rpm = rev[rev[:, 2] < -10]
        negative_speed = rev[rev[:, 10] < -0.05]
        r["reverse_rpm_delay_s"] = (
            float(negative_rpm[0, 0] - 15) if len(negative_rpm) else None
        )
        r["reverse_motion_delay_s"] = (
            float(negative_speed[0, 0] - 15) if len(negative_speed) else None
        )
        checks["finite_reversal_delay"] = bool(
            len(negative_rpm)
            and len(negative_speed)
            and negative_rpm[0, 0] > 15
            and negative_speed[0, 0] > negative_rpm[0, 0]
        )
    r.update(
        passed=all(checks.values()),
        checks=checks,
        driven_speed_m_s=u,
        driven_yaw_deg=math.degrees(yaw),
        max_abs_power_W=float(np.max(abs(d[:, 7]))),
        final_speed_m_s=float(d[-1, 10]),
        max_abs_roll_pitch_deg=np.degrees(np.max(abs(d[:, 15:17]), axis=0)).tolist(),
        final_sim_s=float(d[-1, 0]),
    )
(out / "result.json").write_text(json.dumps(r, indent=2))
print(json.dumps(r, indent=2))
raise SystemExit(0 if r["passed"] else 1)
