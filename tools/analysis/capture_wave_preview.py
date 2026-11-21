#!/usr/bin/env python3
"""采集波场与船体状态的预览证据。

Record native Gazebo frames and contemporaneous wave telemetry.
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
import argparse
from PIL import Image

p = argparse.ArgumentParser()
p.add_argument("--partition", default="usv_refined_final")
a = p.parse_args()
w = WORKSPACE
out = w / "../artifacts/validation/船体三项优化_2026-11-20/preview"
out.mkdir(parents=True, exist_ok=True)
os.environ.update(GZ_PARTITION=a.partition, GZ_IP="127.0.0.1")


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
samples = []


def record_message_1(m):
    """记录收到的消息，沿用当前订阅所绑定的状态容器。

    Args:
        m: 当前订阅接收到的消息，类型由订阅接口确定。
    """
    return samples.append(list(m.data))


node.subscribe(Double_V, "/usv/wave_response/state", record_message_1)
start = time.monotonic()
while not samples and time.monotonic() - start < 10:
    time.sleep(0.1)
if not samples:
    raise RuntimeError("No live wave telemetry")
frames = []
for i in range(24):
    f = out / f"frame_{i:02d}.png"
    before = samples[-1][0]
    subprocess.run(
        [
            "python3",
            str(tool_path("capture_gazebo_view.py")),
            "--partition",
            a.partition,
            "--output",
            str(f),
        ],
        check=True,
        stdout=subprocess.DEVNULL,
    )
    frames.append(
        {
            "file": f.name,
            "sim_before": before,
            "sim_after": samples[-1][0],
            "state_after": samples[-1],
        }
    )
    time.sleep(0.2)
(out / "frames.json").write_text(json.dumps(frames, indent=2))
(out / "telemetry.json").write_text(json.dumps(samples))
images = []
for f in frames:
    im = Image.open(out / f["file"]).convert("RGB")
    im.thumbnail((760, 760))
    images.append(im.copy())
durations = [
    max(100, min(3000, int(1000 * (b["sim_after"] - a["sim_after"]))))
    for a, b in zip(frames, frames[1:])
]
durations.append(durations[-1])
images[0].save(
    out / "waves.gif",
    save_all=True,
    append_images=images[1:],
    duration=durations,
    loop=0,
)
print(
    json.dumps(
        {
            "frames": len(frames),
            "sim_seconds": frames[-1]["sim_after"] - frames[0]["sim_after"],
            "output": str(out / "waves.gif"),
        }
    )
)
