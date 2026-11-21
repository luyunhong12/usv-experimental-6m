#!/usr/bin/env python3
"""检查实际舵角、显示姿态与操控方向的一致性。

Actual ArduRover PWM -> physical steering -> moving-mesh GUI acceptance.

Run against the isolated GUI world recorded in the report; no real hardware.
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
import os
import sys
import time
import json
import math
import signal
import subprocess
import shutil
import argparse
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from PIL import Image

w = WORKSPACE
sys.path.insert(0, str(w / "../dependencies/vendor/python"))


def _load_runtime_dependencies_1():
    """在当前路径和运行环境配置完成后加载依赖，保持原导入时机。"""
    from pymavlink import mavutil

    return (mavutil,)


(mavutil,) = _load_runtime_dependencies_1()

os.environ.update(GZ_PARTITION="usv_steering_visual_20261120", GZ_IP="127.0.0.1")


def _load_runtime_dependencies_2():
    """在当前路径和运行环境配置完成后加载依赖，保持原导入时机。"""
    from gz.transport13 import Node
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.double_v_pb2 import Double_V
    from gz.msgs10.model_pb2 import Model

    return (
        Node,
        Pose_V,
        Double_V,
        Model,
    )


(
    Node,
    Pose_V,
    Double_V,
    Model,
) = _load_runtime_dependencies_2()

parser = argparse.ArgumentParser()
parser.add_argument(
    "--output", type=Path, default=w / "../artifacts/runs/转向关节与画面验证_2026-11-20"
)
args = parser.parse_args()
out = args.output.resolve()
out.mkdir(parents=True, exist_ok=True)
if (out / "result.json").exists():
    raise SystemExit(
        "Existing results: archive them before repeating this fixed-instance test"
    )
shutil.copy2(w / "src/usv_ardupilot/config/platform15.parm", out / "platform15.parm")
node = Node()
latest = {}
rows = []
poses = {}
joint = None
status = []
servos = []


def state(m):
    """缓存最新执行器状态数组。

    Args:
        m: 执行器 Double_V 消息
    """
    latest["state"] = list(m.data)


def pose(m):
    """缓存船体与推进组件位姿。

    Args:
        m: 多物体位姿消息
    """
    for v in m.pose:
        if v.name in ["hull", "azimuth", "propeller", "usv"]:
            poses[v.name] = v


node.subscribe(Double_V, "/usv/actuators/state", state)
node.subscribe(Pose_V, "/world/usv_sydney/dynamic_pose/info", pose)


def cache_telemetry_1(m):
    """更新最近的遥测状态，字段顺序沿用对应消息定义。

    Args:
        m: 当前订阅接收到的消息，类型由订阅接口确定。
    """
    return latest.update(joints=m)


node.subscribe(Model, "/usv/joint_state", cache_telemetry_1)
procs = []
logs = []


def start(args, name):
    """启动试验子进程并登记输出日志。

    Args:
        args: 子进程命令参数列表
        name: 输出日志文件名
    """
    f = (out / name).open("w")
    logs.append(f)
    p = subprocess.Popen(
        args, stdout=f, stderr=subprocess.STDOUT, start_new_session=True
    )
    procs.append(p)


start(
    [
        "bash",
        str(tool_path("run_ardupilot.sh")),
        "--port",
        "9442",
        "--log",
        str(out / "json_frames.jsonl"),
    ],
    "bridge.log",
)
start(
    [
        "python3",
        str(tool_path("run_sitl.py")),
        "--instance",
        "44",
        "--profile",
        "platform15",
        "--wipe",
    ],
    "sitl.log",
)
M = mavutil.mavlink
conn = None
armed = False
mode = None
last_hb = last_rc = last_arm = 0.0
begin = time.monotonic()
phase = -1
phase_t = 0
captures = []
failure = None
navigation_ready = False
stages = [
    ("center_before", 1500, 0, 8),
    ("left", 1000, -35, 12),
    ("center_between", 1500, 0, 12),
    ("right", 2000, 35, 12),
    ("center_after", 1500, 0, 12),
]


def heading(p):
    """由位姿四元数计算朝向。

    Args:
        p: 包含 orientation 的位姿消息
    """
    q = p.orientation
    r = Rotation.from_quat([q.x, q.y, q.z, q.w])
    axis = r.apply([1, 0, 0])
    return math.degrees(math.atan2(axis[1], axis[0]))


def capture(name):
    # Restore the inspection camera before every shot; desktop interaction may
    # otherwise leave a valid screenshot with the tiny vessel out of view.
    """调整观察相机并采集指定名称的图像证据。

    Args:
        name: 本次截图的阶段名称
    """
    camera = "pose: {position: {x: -485 y: 178.7 z: -0.2} orientation: {w: 0.9203 x: -0.0234 y: 0.0553 z: 0.3869}}"
    move = subprocess.run(
        [
            "gz",
            "service",
            "-s",
            "/gui/move_to/pose",
            "--reqtype",
            "gz.msgs.GUICamera",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "3000",
            "--req",
            camera,
        ],
        capture_output=True,
        text=True,
        timeout=5,
    )
    if move.returncode or "data: true" not in move.stdout:
        raise RuntimeError("Inspection camera request failed")
    time.sleep(1)
    folder = out / "screenshots" / name
    folder.mkdir(parents=True, exist_ok=True)
    # Separate CLI process avoids blocking transport callbacks while awaiting a GUI reply.
    command = [
        "gz",
        "service",
        "-s",
        "/gui/screenshot",
        "--reqtype",
        "gz.msgs.StringMsg",
        "--reptype",
        "gz.msgs.Boolean",
        "--timeout",
        "5000",
        "--req",
        "data: " + json.dumps(str(folder)),
    ]
    reply = subprocess.run(command, capture_output=True, text=True, timeout=8)
    if reply.returncode or "data: true" not in reply.stdout:
        raise RuntimeError(
            "GUI screenshot request failed: " + reply.stdout + reply.stderr
        )
    deadline = time.monotonic() + 5
    selected = None
    while time.monotonic() < deadline:
        files = list(folder.glob("*.png"))
        if files:

            def modification_time_2(p):
                """返回文件修改时间，供最新记录排序。

                Args:
                    p: 待排序的 pathlib 路径对象。
                """
                return p.stat().st_mtime

            candidate = max(files, key=modification_time_2)
            try:
                with Image.open(candidate) as im:
                    im.verify()
                selected = candidate
                break
            except (OSError, SyntaxError):
                pass
        time.sleep(0.1)
    if selected is None:
        raise RuntimeError("GUI did not finish a valid PNG screenshot")
    shutil.copy2(selected, out / (name + ".png"))
    return name + ".png"


try:
    while time.monotonic() - begin < 180:
        if any(p.poll() is not None for p in procs):
            raise RuntimeError("Owned flight-control child exited")
        now = time.monotonic()
        if conn is None:
            try:
                conn = mavutil.mavlink_connection(
                    "tcp:127.0.0.1:6200", source_system=255, source_component=190
                )
            except (ConnectionError, OSError):
                time.sleep(0.2)
                continue
        for _ in range(50):
            m = conn.recv_match(blocking=False)
            if m is None:
                break
            if m.get_type() == "HEARTBEAT":
                armed = bool(m.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED)
                mode = m.custom_mode
            elif m.get_type() == "STATUSTEXT":
                status.append(m.text)
            elif m.get_type() == "SERVO_OUTPUT_RAW":
                servos.append(m.to_dict())
        if now - last_hb > 1:
            conn.mav.heartbeat_send(M.MAV_TYPE_GCS, M.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            last_hb = now
        if phase < 0 and now - last_arm > 4:
            if mode == 15:
                navigation_ready = True
            conn.mav.set_mode_send(
                1, M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0 if navigation_ready else 15
            )
            conn.mav.request_data_stream_send(1, 1, M.MAV_DATA_STREAM_ALL, 10, 1)
            if navigation_ready and mode == 0:
                conn.mav.command_long_send(
                    1, 1, M.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
                )
            last_arm = now
        if now - last_rc > 0.05:
            pwm = 1500 if phase < 0 else stages[phase][1]
            conn.mav.rc_channels_override_send(
                1, 1, pwm, 1500, 1500, 1500, 65535, 65535, 65535, 65535
            )
            last_rc = now
        if "state" in latest and all(
            k in poses for k in ["hull", "azimuth", "propeller"]
        ):
            st = latest["state"]
            sim = st[0]
            if phase < 0 and armed and mode == 0:
                phase = 0
                phase_t = sim
                print("normal arming; starting", stages[phase][0], flush=True)
            if phase >= 0:
                if not armed:
                    raise RuntimeError("Unexpected disarm")
                joint_angles = (
                    {j.name: j.axis1.position for j in latest["joints"].joint}
                    if "joints" in latest
                    else {}
                )
                values = [
                    sim,
                    phase,
                    stages[phase][2],
                    math.degrees(st[1]),
                    heading(poses["azimuth"]),
                    heading(poses["propeller"]),
                    math.degrees(joint_angles.get("steering", float("nan"))),
                    st[4],
                    st[5],
                ]
                rows.append(values)
                if sim - phase_t >= stages[phase][3]:
                    name, pwm, target, duration = stages[phase]
                    tail = np.array(
                        [r for r in rows if r[1] == phase and r[0] > sim - 2]
                    )
                    shot = capture(name)
                    entry = dict(
                        stage=name,
                        sim_time=sim,
                        rc_pwm=pwm,
                        target_deg=target,
                        actual_deg=float(np.mean(tail[:, 3])),
                        azimuth_mesh_heading_deg=float(np.mean(tail[:, 4])),
                        propeller_axis_heading_deg=float(np.mean(tail[:, 5])),
                        max_tracking_error_deg=float(np.max(abs(tail[:, 3] - target))),
                        max_mesh_joint_error_deg=float(
                            np.max(abs(tail[:, 4:7] - tail[:, 3, None]))
                        ),
                        screenshot=shot,
                    )
                    captures.append(entry)
                    print(json.dumps(entry), flush=True)
                    phase += 1
                    phase_t = sim
                    if phase == len(stages):
                        break
        time.sleep(0.01)
except (Exception, KeyboardInterrupt) as exc:
    failure = str(exc) or type(exc).__name__
finally:
    if conn:
        conn.mav.rc_channels_override_send(
            1, 1, 1500, 1500, 1500, 1500, 65535, 65535, 65535, 65535
        )
        conn.mav.command_long_send(
            1, 1, M.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
        )
        time.sleep(0.2)
    for p in reversed(procs):
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGTERM)
            try:
                p.wait(timeout=6)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait()
    for f in logs:
        f.close()
np.savetxt(
    out / "steering.csv",
    rows,
    delimiter=",",
    header="sim_s,stage,target_deg,actual_deg,azimuth_mesh_heading_deg,propeller_axis_heading_deg,joint_state_deg,servo_torque_nm,servo_power_w",
    comments="",
)
(out / "mavlink.json").write_text(
    json.dumps({"status": status, "servo_output": servos}, indent=2)
)
checks = {
    "completed": len(captures) == 5,
    "normal_arming": phase >= 0,
    "finite": bool(rows and np.isfinite(rows).all()),
    "tracking_within_1deg": bool(
        captures and all(x["max_tracking_error_deg"] < 1 for x in captures)
    ),
    "moving_mesh_matches_joint": bool(
        captures and all(x["max_mesh_joint_error_deg"] < 0.1 for x in captures)
    ),
    "actual_servo_pwm_extremes": bool(
        servos
        and min(x["servo1_raw"] for x in servos) <= 1005
        and max(x["servo1_raw"] for x in servos) >= 1995
    ),
    "no_thrust_command": bool(
        servos and all(x["servo3_raw"] in [0, 1500] for x in servos)
    ),
    "no_runtime_failure": failure is None,
}
r = {
    "passed": all(checks.values()),
    "checks": checks,
    "captures": captures,
    "failure": failure,
    "scope": "Actual ArduRover MANUAL RC override -> SERVO1 PWM -> JSON bridge -> physical joint -> GUI screenshots; zero propulsion isolates steering. Visual review recorded separately.",
}
(out / "result.json").write_text(json.dumps(r, indent=2))
print(json.dumps(r, indent=2))
raise SystemExit(0 if r["passed"] else 1)
