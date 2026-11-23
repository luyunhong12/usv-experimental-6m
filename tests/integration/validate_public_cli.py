#!/usr/bin/env python3
"""通过公开命令行入口检查启动与操控。

Check the public launcher/manual CLI from a cwd outside the checkout.
"""
from pathlib import Path
import argparse
import json
import os
import signal
import subprocess
import time
import uuid

r = next(p for p in Path(__file__).resolve().parents if (p / "project.toml").is_file())
p = argparse.ArgumentParser()
p.add_argument("--output", type=Path, required=True)
p.add_argument("--drive-duration", type=float, default=4)
p.add_argument("--ready-timeout", type=float, default=70)
a = p.parse_args()
if a.drive_duration <= 0 or a.ready_timeout <= 0:
    p.error("Durations must be positive")
o = a.output.resolve()
o.mkdir(parents=True, exist_ok=True)
partition = "usv_cli_" + uuid.uuid4().hex[:8]
os.environ.update(GZ_PARTITION=partition, GZ_IP="127.0.0.1", ROS_DOMAIN_ID="89")


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
prop = []
act = []


def record_message_1(m):
    """记录收到的消息，沿用当前订阅所绑定的状态容器。

    Args:
        m: 当前订阅接收到的消息，类型由订阅接口确定。
    """
    return prop.append(list(m.data))


node.subscribe(Double_V, "/usv/propulsion/state", record_message_1)


def record_message_2(m):
    """记录收到的消息，沿用当前订阅所绑定的状态容器。

    Args:
        m: 当前订阅接收到的消息，类型由订阅接口确定。
    """
    return act.append(list(m.data))


node.subscribe(Double_V, "/usv/actuators/state", record_message_2)
log = (o / "platform.log").open("w")
proc = subprocess.Popen(
    [str(r / "usv"), "run", "--wave-gain", "0", "headless:=True"],
    cwd="/tmp",
    stdout=log,
    stderr=subprocess.STDOUT,
    start_new_session=True,
)
checks = {}
failure = None
stages = []
try:
    ready = r / "ros_ws/log" / ("platform-ready-" + partition + ".json")
    deadline = time.monotonic() + a.ready_timeout
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            raise RuntimeError("Platform exited before readiness")
        if (
            ready.exists()
            and json.loads(ready.read_text()).get("ready")
            and prop
            and act
        ):
            break
        time.sleep(0.1)
    else:
        raise RuntimeError("Timed out waiting for ready platform and telemetry")
    checks["ready"] = True
    for name, th, angle in [
        ("forward", 0.3, 0),
        ("reverse", -0.3, 0),
        ("left", 0.3, -20),
        ("right", 0.3, 20),
    ]:
        start = len(prop)
        astart = len(act)
        result = subprocess.run(
            [
                str(r / "usv"),
                "drive",
                "--throttle",
                str(th),
                "--steering-deg",
                str(angle),
                "--duration",
                str(a.drive_duration),
            ],
            cwd="/tmp",
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=max(20, a.drive_duration + 15),
        )
        (o / (name + ".log")).write_text(result.stdout)
        pr = prop[start:]
        ar = act[astart:]
        ok = result.returncode == 0 and any(abs(x[1] - th) < 1e-6 for x in pr)
        if angle:
            ok = ok and any(x[1] * angle > 0 and abs(x[1]) > 0.15 for x in ar)
        checks[name] = bool(ok)
        stages.append(
            {
                "name": name,
                "exit_code": result.returncode,
                "propulsion_samples": len(pr),
                "actuator_samples": len(ar),
            }
        )
    time.sleep(1.2)
    checks["zero_after_release"] = bool(prop and abs(prop[-1][1]) < 1e-9)
except Exception as exc:
    failure = str(exc)
finally:
    if proc.poll() is None:
        os.kill(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
    log.close()
    subprocess.run(
        [
            "python3",
            str(r / "tests/integration/stop_test_partition.py"),
            partition,
            "--exclude-pid",
            str(os.getpid()),
        ],
        check=True,
    )
(o / "telemetry.json").write_text(json.dumps({"propulsion": prop, "actuators": act}))
result = {
    "passed": not failure and len(checks) == 6 and all(checks.values()),
    "checks": checks,
    "failure": failure,
    "stages": stages,
    "partition": partition,
    "cwd": "/tmp",
    "drive_duration_wall_s": a.drive_duration,
    "ready_timeout_wall_s": a.ready_timeout,
    "scope": "Public command routing and steering response, not steady maneuver performance",
}
(o / "result.json").write_text(json.dumps(result, indent=2))
print(json.dumps(result, indent=2))
raise SystemExit(0 if result["passed"] else 1)
