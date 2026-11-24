#!/usr/bin/env python3
"""按工程配置启动本机或容器中的飞控仿真进程。

参数源只读挂载，持久状态写入 artifacts；容器通过 Linux 主机网络连接本机桥。
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
import argparse
import os
from pathlib import Path

root = WORKSPACE
p = argparse.ArgumentParser()
p.add_argument("--instance", type=int, default=10)
p.add_argument(
    "--binary",
    type=Path,
    default=Path(
        os.environ.get(
            "ARDUPILOT_BIN",
            str(root.parent / "dependencies/src/ardupilot/build/sitl/bin/ardurover"),
        )
    ),
)
p.add_argument(
    "--container-image",
    default=os.environ.get("ARDUPILOT_CONTAINER_IMAGE"),
    help="容器镜像名或摘要；设置后使用 Docker，不使用本机 --binary。",
)
p.add_argument("--wipe", action="store_true")
p.add_argument("--profile", choices=["baseline", "platform15"], default="baseline")
a = p.parse_args()
if not 0 <= a.instance <= 99:
    p.error("Instance must be in [0,99]")
if a.container_image and (
    a.container_image.startswith("-") or any(c.isspace() for c in a.container_image)
):
    p.error("Invalid container image name")
if not a.container_image and not a.binary.is_file():
    p.error("ArduRover binary not found; pass --binary or ARDUPILOT_BIN")
work = root.parent / "artifacts/sitl" / a.profile / f"instance_{a.instance}"
work.mkdir(parents=True, exist_ok=True)
parameter_dir = root / "src/usv_ardupilot/config"
runtime_parameters = Path("/config") if a.container_image else parameter_dir
args = [
    "/usr/local/bin/ardurover" if a.container_image else str(a.binary.resolve()),
    "--model",
    "JSON",
    "--sim-address",
    "127.0.0.1",
    "--sim-port-out",
    str(9002 + 10 * a.instance),
    "--instance",
    str(a.instance),
    "--home",
    "-33.724223,150.679736,0,90",
    "--defaults",
    str(runtime_parameters / "boat.parm")
    + (
        "," + str(runtime_parameters / "platform15.parm")
        if a.profile == "platform15"
        else ""
    ),
    "--speedup",
    "1",
    "--rate",
    "100",
]
if a.wipe:
    args.append("--wipe")
if a.container_image:
    import shutil

    docker = shutil.which("docker")
    if docker is None:
        p.error("Docker executable not found")
    args = [
        docker,
        "run",
        "--rm",
        "--init",
        "--network=host",
        "--cap-drop=ALL",
        "--security-opt=no-new-privileges",
        "--user",
        f"{os.getuid()}:{os.getgid()}",
        "--name",
        f"usv-sitl-{a.profile}-{a.instance}",
        "--mount",
        f"type=bind,source={parameter_dir},target=/config,readonly",
        "--mount",
        f"type=bind,source={work},target=/data",
        a.container_image,
        *args[1:],
    ]
print(
    f"JSON bridge port {9002+10*a.instance}; MAVLink TCP port {5760+10*a.instance}",
    flush=True,
)
os.chdir(work)
os.execv(args[0], args)
