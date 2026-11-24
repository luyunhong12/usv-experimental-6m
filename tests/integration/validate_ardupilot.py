#!/usr/bin/env python3
"""验证本机飞控与仿真通信的指定接入路径。

Local SITL handshake, normal arming and manual forward/reverse/turn check.
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
import signal
import subprocess
import argparse
import shutil
import xml.etree.ElementTree as E
import numpy as np

root = WORKSPACE
sys.path.insert(0, str(root / "../dependencies/vendor/python"))


def _load_runtime_dependencies_1():
    """在当前路径和运行环境配置完成后加载依赖，保持原导入时机。"""
    from pymavlink import mavutil

    return (mavutil,)


(mavutil,) = _load_runtime_dependencies_1()

p = argparse.ArgumentParser()
p.add_argument("--profile", choices=["baseline", "platform15"], default="baseline")
p.add_argument("--label")
p.add_argument(
    "--isolated-water",
    action="store_true",
    help="从试验世界移除其他模型，隔离码头碰撞对接口验收的影响；保留船体物理。",
)
p.add_argument(
    "--gcs-dropout",
    action="store_true",
    help="正常操船后再次推进，再停止地面站心跳和遥控消息，验证失联停推。",
)
p.add_argument(
    "--external-sitl",
    action="store_true",
    help="由调用者管理实例42的 SITL 生命周期；本测试只启动仿真与桥。",
)
p.add_argument(
    "--no-rendering",
    action="store_true",
    help="仅从试验快照移除相机、GPU 雷达与渲染传感器插件，保留物理和 IMU。",
)
a = p.parse_args()
if a.label and not a.label.isidentifier():
    p.error("Invalid label")
os.environ["GZ_PARTITION"] = "usv_ardupilot_validation"
os.environ["GZ_IP"] = "127.0.0.1"


def _load_runtime_dependencies_2():
    """在当前路径和运行环境配置完成后加载依赖，保持原导入时机。"""
    from gz.transport13 import Node
    from gz.msgs10.double_v_pb2 import Double_V

    return (
        Node,
        Double_V,
    )


(
    Node,
    Double_V,
) = _load_runtime_dependencies_2()

out = root.parent / "artifacts/sitl" / (a.label or "manual_validation")
if out.exists():
    p.error("Output directory already exists; choose a new --label")
out.mkdir(parents=True, exist_ok=True)
node = Node()
rows = []


def record_message_1(m):
    """记录收到的消息，沿用当前订阅所绑定的状态容器。

    Args:
        m: 当前订阅接收到的消息，类型由订阅接口确定。
    """
    return rows.append(list(m.data))


node.subscribe(Double_V, "/usv/propulsion/state", record_message_1)
world = out / "world.sdf"
subprocess.run(
    ["python3", str(tool_path("prepare_sydney.py")), "--output", str(world)]
    + (["--profile", "speed15_concept"] if a.profile == "platform15" else []),
    check=True,
    stdout=subprocess.DEVNULL,
)
tree = E.parse(world)
for inc in tree.findall("world/include"):
    if a.isolated_water and inc.findtext("name") != "usv":
        tree.find("world").remove(inc)
        continue
    if inc.findtext("name") == "usv":
        snapshot = out / "boat.sdf"
        shutil.copy2(inc.findtext("uri"), snapshot)
        inc.find("uri").text = str(snapshot)
        if a.no_rendering:
            boat = E.parse(snapshot)
            for parent in boat.iter():
                for sensor in list(parent.findall("sensor")):
                    if sensor.get("type") in {"camera", "gpu_lidar", "gpu_ray"}:
                        parent.remove(sensor)
            boat.write(snapshot, encoding="utf-8", xml_declaration=True)
if a.no_rendering:
    for plugin in list(tree.findall("world/plugin")):
        if plugin.get("name") == "gz::sim::systems::Sensors":
            tree.find("world").remove(plugin)
tree.write(world, encoding="utf-8", xml_declaration=True)
for filename in ["boat.parm", "platform15.parm"]:
    shutil.copy2(root / "src/usv_ardupilot/config" / filename, out / filename)
procs = []
files = []


def start(args, name):
    """启动子进程并保留其输出文件供退出时清理。

    Args:
        args: 子进程命令参数列表
        name: 输出日志文件名
    """
    f = (out / name).open("w")
    files.append(f)
    p = subprocess.Popen(
        args, cwd=out, stdout=f, stderr=subprocess.STDOUT, start_new_session=True
    )
    procs.append(p)
    return p


start(
    ["bash", str(tool_path("run_hull_trial.sh")), "world:=" + str(world)], "gazebo.log"
)
start(
    [
        "bash",
        str(tool_path("run_ardupilot.sh")),
        "--port",
        "9422",
        "--log",
        str(out / "json_frames.jsonl"),
    ],
    "bridge.log",
)
# Instance 42 has independent TCP/UDP ports and EEPROM/log directory.
if not a.external_sitl:
    start(
        [
            "python3",
            str(tool_path("run_sitl.py")),
            "--instance",
            "42",
            "--profile",
            a.profile,
            "--wipe",
        ],
        "sitl.log",
    )
status = []
attitudes = []
heartbeats = []
servo = []
estimator = []
estimator_faults = []
navigation_ready = False
navigation_ready_at_arming = False
last_estimator_wall = 0.0
estimator_ready_since = None
phase_start = None
armed = False
conn = None
begin = time.monotonic()
last_rc = last_hb = last_arm = 0.0
result = {}
try:
    while time.monotonic() - begin < 240:
        if any(p.poll() is not None for p in procs):
            break
        if conn is None:
            try:
                conn = mavutil.mavlink_connection(
                    "tcp:127.0.0.1:6180", source_system=255, source_component=190
                )
            except (ConnectionError, OSError):
                time.sleep(0.2)
                continue
        m = conn.recv_match(blocking=False)
        if m:
            kind = m.get_type()
            if kind == "STATUSTEXT":
                status.append(m.text)
                if phase_start is not None and "EKF variance" in m.text:
                    estimator_faults.append(m.text)
            elif kind == "HEARTBEAT":
                heartbeats.append(m.to_dict())
                armed = bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif kind == "ATTITUDE":
                attitudes.append(m.to_dict())
            elif kind == "SERVO_OUTPUT_RAW":
                servo.append(m.to_dict())
            elif kind == "EKF_STATUS_REPORT":
                estimator.append(dict(sim_s=rows[-1][0] if rows else 0, **m.to_dict()))
                last_estimator_wall = time.monotonic()
                required = (
                    mavutil.mavlink.EKF_ATTITUDE
                    | mavutil.mavlink.EKF_VELOCITY_HORIZ
                    | mavutil.mavlink.EKF_POS_HORIZ_ABS
                )
                valid = (
                    (m.flags & required) == required
                    and m.velocity_variance < 1
                    and m.pos_horiz_variance < 1
                )
                if not valid:
                    estimator_ready_since = None
                elif estimator_ready_since is None:
                    estimator_ready_since = last_estimator_wall
                navigation_ready = (
                    estimator_ready_since is not None
                    and last_estimator_wall - estimator_ready_since >= 2
                )
        now = time.monotonic()
        sim = rows[-1][0] if rows else 0
        elapsed = sim - phase_start if phase_start is not None else -1
        link_active = not (a.gcs_dropout and elapsed >= 84)
        if link_active and now - last_hb > 1:
            conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                0,
            )
            last_hb = now
        if heartbeats and now - last_arm > 4 and phase_start is None:
            conn.mav.set_mode_send(
                1, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0
            )
            conn.mav.request_data_stream_send(
                1, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
            )
            conn.mav.command_long_send(
                1,
                1,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT,
                200000,
                0,
                0,
                0,
                0,
                0,
            )
            # MANUAL 可在导航估计尚未就绪时解锁；验收主动等待稳定定位，保留原解锁检查。
            if navigation_ready and now - last_estimator_wall < 1:
                conn.mav.command_long_send(
                    1,
                    1,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    1,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
            last_arm = now
        if armed and rows and phase_start is None:
            navigation_ready_at_arming = navigation_ready
            phase_start = sim
        elapsed = sim - phase_start if phase_start is not None else -1
        if link_active and now - last_rc > 0.1:
            thr = (
                1800
                if 0 <= elapsed < 12 or 28 <= elapsed < 68
                else 1200 if 12 <= elapsed < 28 else 1500
            )
            steer = 1800 if 28 <= elapsed < 48 else 1200 if 48 <= elapsed < 68 else 1500
            if a.gcs_dropout and 76 <= elapsed < 84:
                thr = 1800
            conn.mav.rc_channels_override_send(
                1, 1, steer, 1500, thr, 1500, 65535, 65535, 65535, 65535
            )
            last_rc = now
        if elapsed >= (100 if a.gcs_dropout else 76):
            break
        time.sleep(0.003)
finally:
    if conn:
        conn.mav.rc_channels_override_send(
            1, 1, 1500, 1500, 1500, 1500, 65535, 65535, 65535, 65535
        )
        conn.mav.command_long_send(
            1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
        )
    for p in reversed(procs):
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGINT)
            try:
                p.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait()
    for f in files:
        f.close()
np.savetxt(out / "propulsion.csv", np.array(rows), delimiter=",")
(out / "mavlink.json").write_text(
    json.dumps(
        dict(
            status=status,
            attitude=attitudes,
            heartbeat=heartbeats,
            servo=servo,
            estimator=estimator,
        ),
        indent=2,
    )
)
checks = {
    "heartbeat": bool(heartbeats),
    "normal_arming": phase_start is not None,
    "navigation_ready_before_arming": navigation_ready_at_arming,
    "no_ekf_variance_after_arming": not estimator_faults,
    "attitude_telemetry": bool(attitudes),
    "servo_telemetry": bool(servo),
    "completed": False,
    "forward": False,
    "reverse": False,
    "right_turn": False,
    "left_turn": False,
    "neutral_throttle": False,
}
if a.gcs_dropout:
    checks.update(driving_before_dropout=False, stopped_after_dropout=False)
if phase_start is not None and rows:
    d = np.array(rows)
    t = d[:, 0] - phase_start
    fwd = d[(t > 8) & (t < 11)]
    rev = d[(t > 24) & (t < 27)]
    turn = d[(t > 44) & (t < 47)]
    left = d[(t > 64) & (t < 67)]
    neutral = d[(t > 72) & (t < 75)]
    checks["completed"] = bool(t[-1] >= 75)
    checks["forward"] = bool(len(fwd) and np.mean(fwd[:, 10]) > 0.2)
    checks["reverse"] = bool(len(rev) and np.mean(rev[:, 10]) < -0.2)
    # 从各自转向阶段的后半段测增量，避免初始航向或上一阶段偏转造成假通过。
    yaw = np.unwrap(d[:, 14])
    right_window = yaw[(t > 38) & (t < 47)]
    left_window = yaw[(t > 58) & (t < 67)]
    checks["right_turn"] = bool(
        len(turn)
        and len(right_window) > 1
        and right_window[-1] - right_window[0] < -0.2
    )
    checks["left_turn"] = bool(
        len(left) and len(left_window) > 1 and left_window[-1] - left_window[0] > 0.2
    )
    checks["neutral_throttle"] = bool(
        len(neutral) and np.max(np.abs(neutral[:, 1])) < 0.01
    )
    result.update(
        phase_start_sim_s=phase_start,
        forward_speed_m_s=float(np.mean(fwd[:, 10])) if len(fwd) else None,
        reverse_speed_m_s=float(np.mean(rev[:, 10])) if len(rev) else None,
    )
    if a.gcs_dropout:
        before_loss = d[(t > 80) & (t < 83)]
        after_loss = d[(t > 95) & (t < 99)]
        checks["driving_before_dropout"] = bool(
            len(before_loss) and np.mean(before_loss[:, 1]) > 0.1
        )
        checks["stopped_after_dropout"] = bool(
            t[-1] >= 99 and len(after_loss) and np.max(np.abs(after_loss[:, 1])) < 0.01
        )
result.update(
    passed=all(checks.values()),
    checks=checks,
    status_tail=status[-15:],
    external_sitl=a.external_sitl,
    rendering_enabled=not a.no_rendering,
    gcs_dropout=a.gcs_dropout,
    isolated_water=a.isolated_water,
)
(out / "result.json").write_text(json.dumps(result, indent=2))
print(json.dumps(result, indent=2))
raise SystemExit(0 if result["passed"] else 1)
