#!/usr/bin/env python3
"""验证物理后端惯性更新的实际响应与回读。

Prove native inertial updates using F=ma and torque=I*alpha, not only readback.
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
import signal
import subprocess
import json
import argparse
import numpy as np

p = argparse.ArgumentParser()
p.add_argument("--mode", choices=["force", "torque"], required=True)
p.add_argument(
    "--output-dir", type=_ProjectPath, help="本次独立证据目录；不覆盖历史运行"
)
a = p.parse_args()
W = WORKSPACE
out = a.output_dir or W / "log/fuel_mass_validation" / ("backend_" + a.mode)
out.mkdir(parents=True, exist_ok=True)
os.environ["GZ_PARTITION"] = "usv_inertia_" + a.mode + "_" + str(os.getpid())
os.environ.setdefault("GZ_IP", "127.0.0.1")


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

rows = []
n = Node()


def record_message_1(m):
    """记录收到的消息，沿用当前订阅所绑定的状态容器。

    Args:
        m: 当前订阅接收到的消息，类型由订阅接口确定。
    """
    return rows.append(list(m.data))


n.subscribe(Double_V, "/usv/inertia_probe", record_message_1)
world = out / "world.sdf"
world.write_text(
    f"""<sdf version="1.9"><world name="inertia_probe"><gravity>0 0 0</gravity><physics name="test" type="dart"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics><plugin filename="libUsvPhysics.so" name="gz::sim::systems::Physics"/><model name="probe"><link name="body"><inertial><mass>10</mass><inertia><ixx>1</ixx><iyy>2</iyy><izz>3</izz></inertia></inertial></link><plugin filename="libUsvInertiaResponseProbe.so" name="usv::physics::InertiaResponseProbe"><mode>{a.mode}</mode></plugin></model></world></sdf>"""
)
log = (out / "launch.log").open("w")
proc = subprocess.Popen(
    ["bash", str(tool_path("run_hull_trial.sh")), "world:=" + str(world)],
    stdout=log,
    stderr=subprocess.STDOUT,
    start_new_session=True,
)
try:
    start = time.monotonic()
    while time.monotonic() - start < 30 and proc.poll() is None:
        if rows and rows[-1][0] >= 4.5:
            break
        time.sleep(0.05)
finally:
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=6)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
    log.close()
    n.unsubscribe("/usv/inertia_probe")
d = np.array(rows)
np.savetxt(
    out / "telemetry.csv",
    d,
    delimiter=",",
    header="time,vx,wz,mass,cgx,cgy,cgz,izz",
    comments="",
)
r = {"mode": a.mode, "passed": False, "samples": len(d)}
if len(d) and d[-1, 0] >= 4:
    col = 1 if a.mode == "force" else 2
    first = d[(d[:, 0] > 0.3) & (d[:, 0] < 1.8)]
    second = d[(d[:, 0] > 2.3) & (d[:, 0] < 4.3)]
    slopes = [float(np.polyfit(x[:, 0], x[:, col], 1)[0]) for x in [first, second]]
    expected = [1, 2] if a.mode == "force" else [1 / 3, 2 / 3]
    # Generalized velocity remains continuous at the mass change (no artificial impulse).
    boundary = d[(d[:, 0] > 1.95) & (d[:, 0] < 2.05), col]
    jump = float(np.max(abs(np.diff(boundary))))
    checks = {
        "force_or_torque_response": bool(
            np.allclose(slopes, expected, rtol=0.002, atol=0.001)
        ),
        "native_readback": bool(
            abs(d[-1, 3] - 5) < 1e-9
            and np.allclose(d[-1, 4:7], [0.1, 0.2, 0.3])
            and abs(d[-1, 7] - 1.5) < 1e-9
        ),
        "no_velocity_impulse": bool(jump < 0.06),
        "finite": bool(np.isfinite(d).all()),
    }
    r.update(
        passed=all(checks.values()),
        checks=checks,
        measured_accelerations=slopes,
        expected_accelerations=expected,
        boundary_velocity_step=jump,
    )
(out / "result.json").write_text(json.dumps(r, indent=2))
print(json.dumps(r, indent=2))
raise SystemExit(0 if r["passed"] else 1)
