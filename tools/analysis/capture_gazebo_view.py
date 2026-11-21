#!/usr/bin/env python3
"""通过 Gazebo 接口保存当前视图。

Capture an actual Gazebo GUI render, with a fixed optional camera pose.
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
import argparse
import os
import subprocess
import json
import time
import shutil
from pathlib import Path
from scipy.spatial.transform import Rotation
from PIL import Image

p = argparse.ArgumentParser()
p.add_argument("--partition", required=True)
p.add_argument("--output", type=Path, required=True)
p.add_argument("--pose", type=float, nargs=6)
a = p.parse_args()
os.environ.update(GZ_PARTITION=a.partition, GZ_IP="127.0.0.1")
out = a.output.resolve()
out.parent.mkdir(parents=True, exist_ok=True)


def request(service, kind, data):
    """调用 Gazebo 服务并验证布尔响应成功。

    Args:
        service: 服务名称
        kind: 请求消息类型
        data: 序列化请求正文
    """
    r = subprocess.run(
        [
            "gz",
            "service",
            "-s",
            service,
            "--reqtype",
            kind,
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "5000",
            "--req",
            data,
        ],
        capture_output=True,
        text=True,
        timeout=8,
    )
    if r.returncode or "data: true" not in r.stdout:
        raise RuntimeError(r.stdout + r.stderr)


if a.pose:
    x, y, z, roll, pitch, yaw = a.pose
    qx, qy, qz, qw = Rotation.from_euler("xyz", [roll, pitch, yaw]).as_quat()
    from gz.msgs10.gui_camera_pb2 import GUICamera
    from google.protobuf.text_format import MessageToString

    camera = GUICamera()
    camera.pose.position.x = x
    camera.pose.position.y = y
    camera.pose.position.z = z
    camera.pose.orientation.x = qx
    camera.pose.orientation.y = qy
    camera.pose.orientation.z = qz
    camera.pose.orientation.w = qw
    request("/gui/move_to/pose", "gz.msgs.GUICamera", MessageToString(camera))
    time.sleep(1)
folder = out.parent / (out.stem + "_raw")
folder.mkdir(parents=True, exist_ok=True)
before = set(folder.glob("*.png"))
request("/gui/screenshot", "gz.msgs.StringMsg", "data: " + json.dumps(str(folder)))
deadline = time.monotonic() + 8
while time.monotonic() < deadline:
    files = set(folder.glob("*.png")) - before
    for f in files:
        try:
            with Image.open(f) as im:
                im.verify()
            shutil.copy2(f, out)
            print(out)
            raise SystemExit(0)
        except (OSError, SyntaxError):
            pass
    time.sleep(0.1)
raise SystemExit("Gazebo did not finish a valid render screenshot")
