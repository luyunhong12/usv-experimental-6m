#!/usr/bin/env python3
"""验证船舶在指定波场中的响应与能源状态。

Actual Gazebo free-floating response, independent reproducible wave cases.
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
import time
import json
import subprocess
import signal
import argparse
import math
import re
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np

p = argparse.ArgumentParser()
p.add_argument(
    "--case",
    choices=["calm", "longitudinal", "beam", "quarter", "strong"],
    required=True,
)
p.add_argument("--output", type=Path)
p.add_argument("--period", type=float, default=5)
p.add_argument("--step", type=float, default=0.004)
p.add_argument("--added-scale", type=float, default=1)
a = p.parse_args()
w = WORKSPACE
out = a.output or w / "../artifacts/runs/船体三项优化_2026-11-20/waves" / a.case
out.mkdir(parents=True, exist_ok=True)
gain = 0 if a.case == "calm" else (0.6 if a.case == "strong" else 0.3)
direction = {
    "calm": 0,
    "longitudinal": 0,
    "beam": math.pi / 2,
    "quarter": math.pi / 4,
    "strong": math.pi / 4,
}[a.case]
os.environ.update(GZ_PARTITION="usv_wave_" + a.case, GZ_IP="127.0.0.1")


def _load_runtime_dependencies():
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
) = _load_runtime_dependencies()

node = Node()
rows = []


def record_message_1(m):
    """记录收到的消息，沿用当前订阅所绑定的状态容器。

    Args:
        m: 当前订阅接收到的消息，类型由订阅接口确定。
    """
    return rows.append(list(m.data))


node.subscribe(Double_V, "/usv/wave_response/state", record_message_1)
world = out / "world.sdf"
subprocess.run(
    [
        "python3",
        str(tool_path("prepare_sydney.py")),
        "--profile",
        "speed15_concept",
        "--wave-gain",
        str(gain),
        "--output",
        str(world),
    ],
    check=True,
    stdout=subprocess.DEVNULL,
)
t = E.parse(world)
inc = t.find("world/include[name='usv']")
bt = E.parse(inc.findtext("uri"))
for wave in bt.findall(".//wavefield/wave"):
    el = wave.find("direction")
    if el is None:
        el = E.SubElement(wave, "direction")
    el.text = str(direction)
    el = wave.find("period")
    if el is None:
        el = E.SubElement(wave, "period")
    el.text = str(a.period)
for tag in ["added_linear", "added_angular"]:
    el = bt.find("model/plugin[@name='usv::HullHydrostatics']/" + tag)
    el.text = " ".join(str(float(x) * a.added_scale) for x in el.text.split())
t.find("world/physics/max_step_size").text = str(a.step)
bt.write(out / "boat.sdf", encoding="utf-8", xml_declaration=True)
inc.find("uri").text = str(out / "boat.sdf")
pub = t.find("world/plugin[@name='vrx::PublisherPlugin']/message")


def replace_wave_value_2(m):
    """保留正则匹配前缀并写入当前工况要求的波参数文本。

    Args:
        m: 正则匹配对象，第 1 组为保留的配置前缀。
    """
    return m[1] + str(direction)


pub.text = re.sub(
    r'(key:\s*"direction".*?double_value:\s*)[-+\d.eE]+',
    replace_wave_value_2,
    pub.text,
    flags=re.S,
)


def replace_wave_value_3(m):
    """保留正则匹配前缀并写入当前工况要求的波参数文本。

    Args:
        m: 正则匹配对象，第 1 组为保留的配置前缀。
    """
    return m[1] + str(a.period)


pub.text = re.sub(
    r'(key:\s*"period".*?double_value:\s*)[-+\d.eE]+',
    replace_wave_value_3,
    pub.text,
    flags=re.S,
)
if a.case == "calm":
    pose = list(map(float, inc.findtext("pose").split()))
    pose[2] += 0.05
    pose[3] += math.radians(5)
    inc.find("pose").text = " ".join(map(str, pose))
t.write(world, encoding="utf-8", xml_declaration=True)
f = (out / "gazebo.log").open("w")
proc = subprocess.Popen(
    ["bash", str(tool_path("run_hull_trial.sh")), "world:=" + str(world)],
    stdout=f,
    stderr=subprocess.STDOUT,
    start_new_session=True,
)
start = time.monotonic()
failure = None
try:
    while time.monotonic() - start < 240:
        if proc.poll() is not None:
            raise RuntimeError("Gazebo exited")
        if rows and rows[-1][0] >= 65:
            break
        time.sleep(0.1)
except Exception as e:
    failure = str(e)
finally:
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGTERM)
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
    f.close()
d = np.array(rows)
checks = {
    "completed": bool(len(d) and d[-1, 0] >= 65),
    "finite": bool(len(d) and np.isfinite(d).all()),
    "no_failure": failure is None,
}
metrics = {}
if len(d) and np.any(d[:, 0] > 25):
    tail = d[d[:, 0] > 25]
    metrics = {
        "heave_peak_to_peak_m": float(np.ptp(tail[:, 6])),
        "roll_peak_to_peak_deg": float(np.rad2deg(np.ptp(tail[:, 7]))),
        "pitch_peak_to_peak_deg": float(np.rad2deg(np.ptp(tail[:, 8]))),
        "surface_peak_to_peak_m": float(np.ptp(tail[:, 1])),
        "max_abs_attitude_deg": float(np.rad2deg(np.max(abs(d[:, 7:9])))),
        "mean_buoyancy_N": float(np.mean(tail[:, 11])),
        "pressure_triangles": int(d[-1, 17]),
        "final_pose_z_roll_pitch": d[-1, 6:9].tolist(),
    }
    checks["attitude_bounded"] = metrics["max_abs_attitude_deg"] < 20
    checks["positive_buoyancy"] = bool(np.min(tail[:, 11]) > 0)
    if gain == 0:
        checks["settles_after_release"] = (
            metrics["heave_peak_to_peak_m"] < 0.01
            and metrics["roll_peak_to_peak_deg"] < 0.3
        )
    else:
        checks["responds_to_waves"] = (
            metrics["heave_peak_to_peak_m"] > 0.02
            and max(metrics["roll_peak_to_peak_deg"], metrics["pitch_peak_to_peak_deg"])
            > 0.1
        )
np.savetxt(
    out / "wave_response.csv",
    d,
    delimiter=",",
    header="t,eta_center,eta_bow,eta_stern,eta_port,eta_starboard,z,roll,pitch,Fx,Fy,Fz,Mx,My,Mz,volume,wet_area,triangles",
    comments="",
)
r = {
    "passed": all(checks.values()),
    "case": a.case,
    "gain": gain,
    "direction_rad": direction,
    "period_s": a.period,
    "step_s": a.step,
    "added_mass_scale": a.added_scale,
    "checks": checks,
    "metrics": metrics,
    "failure": failure,
}
(out / "result.json").write_text(json.dumps(r, indent=2))
print(json.dumps(r, indent=2))
raise SystemExit(0 if r["passed"] else 1)
