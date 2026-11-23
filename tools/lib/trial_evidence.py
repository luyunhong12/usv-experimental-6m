"""绑定试验工况、源码、构建产物和遥测，拒绝静默复用过期证据。"""

import argparse
import hashlib
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BUILD_RECORD = ROOT / "ros_ws/log/build_binding.json"
BUILD_START = ROOT / "ros_ws/log/build_inputs_started.json"


def digest(value):
    """计算 JSON 值的稳定摘要。

    Args:
        value: 可序列化的配置或清单。
    """
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()


def file_hash(path):
    """流式计算文件摘要，避免大网格占用过多内存。

    Args:
        path: 待读取的文件路径。
    """
    result = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def hashes(paths, root=ROOT):
    """按相对路径记录输入文件的内容摘要。

    Args:
        paths: 输入文件路径集合。
        root: 路径基准目录。
    """
    return {
        str(p.relative_to(root)): file_hash(p)
        for p in sorted(set(paths))
        if p.is_file()
    }


def build_inputs():
    """收集编译与安装输入，防止 Python、参数或模型仍使用旧副本。"""
    paths = []
    for base in [ROOT / "ros_ws/src", ROOT / "dependencies/src/vrx"]:
        paths.extend(
            p
            for p in base.rglob("*")
            if (
                p.suffix
                in {
                    ".cc",
                    ".cpp",
                    ".hh",
                    ".hpp",
                    ".h",
                    ".py",
                    ".json",
                    ".xml",
                    ".sdf",
                    ".txt",
                    ".obj",
                    ".stl",
                    ".dae",
                }
                or p.name in {"CMakeLists.txt", "package.xml"}
            )
            and "instances" not in p.parts
            and p.name != "usv_sydney.sdf"
        )
    return hashes(paths)


def binaries():
    """记录本工作区安装的共享库、脚本、模型和参数副本。"""
    paths = [
        p
        for p in (ROOT / "ros_ws/install").rglob("*")
        if p.suffix
        in {".so", ".py", ".json", ".xml", ".sdf", ".txt", ".obj", ".stl", ".dae"}
        and "instances" not in p.parts
        and p.name != "usv_sydney.sdf"
    ]
    return hashes(paths)


def begin_build():
    """在编译前记录输入，防止构建途中改动源码后误绑定。"""
    BUILD_START.parent.mkdir(parents=True, exist_ok=True)
    BUILD_START.write_text(json.dumps(build_inputs(), indent=2) + "\n")


def record_build():
    """只在成功构建后保存源码与安装库的对应关系。"""
    inputs = build_inputs()
    if not BUILD_START.exists() or json.loads(BUILD_START.read_text()) != inputs:
        raise ValueError(
            "Sources changed during build; rebuild without editing native sources"
        )
    value = {"schema": 1, "inputs": inputs, "binaries": binaries()}
    if not any(name.endswith(".so") for name in value["binaries"]):
        raise ValueError(
            "No installed native libraries; build did not produce usable plugins"
        )
    BUILD_RECORD.parent.mkdir(parents=True, exist_ok=True)
    BUILD_RECORD.write_text(json.dumps(value, indent=2) + "\n")


def execution_snapshot():
    """验证构建新鲜度，并收集实际运行所需配置及脚本。"""
    if not BUILD_RECORD.exists():
        raise ValueError("Missing build binding; run ./usv build first")
    built = json.loads(BUILD_RECORD.read_text())
    if built["inputs"] != build_inputs() or built["binaries"] != binaries():
        raise ValueError(
            "Native sources or installed libraries changed; run ./usv build"
        )
    paths = []
    for folder in ["ros_ws/src", "tools/lib", "tools/modeling", "tools/control"]:
        paths.extend(
            p
            for p in (ROOT / folder).rglob("*")
            if p.is_file()
            and "__pycache__" not in p.parts
            and "instances" not in p.parts
            and p.suffix
            in {".py", ".sh", ".json", ".xml", ".txt", ".sdf", ".obj", ".stl", ".dae"}
        )
    # Generated Sydney world changes per launch; each trial seals its own world.
    paths = [p for p in paths if p.name != "usv_sydney.sdf"]
    paths.extend(
        ROOT / name
        for name in [
            "tests/integration/validate_vessel_performance.py",
            "tests/regression/run_performance_suite.py",
            "dependencies/sources.repos",
        ]
    )
    environment = {
        key: os.environ.get(key)
        for key in [
            "RMW_IMPLEMENTATION",
            "FASTRTPS_DEFAULT_PROFILES_FILE",
            "FASTDDS_DEFAULT_PROFILES_FILE",
            "FASTDDS_BUILTIN_TRANSPORTS",
            "RMW_FASTRTPS_USE_QOS_FROM_XML",
            "LIBGL_ALWAYS_SOFTWARE",
            "__NV_PRIME_RENDER_OFFLOAD",
            "__GLX_VENDOR_LIBRARY_NAME",
            "DISPLAY",
        ]
    }
    external_profiles = {}
    for key in ["FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE"]:
        if environment[key]:
            external_profiles[key] = file_hash(Path(environment[key]))
    return {
        "build": built,
        "runtime": hashes(paths),
        "environment": environment,
        "external_profiles": external_profiles,
    }


def initialize_suite(output, specs, snapshot, resume=False):
    """创建不可混用的套件清单；恢复必须显式且完全匹配。

    Args:
        output: 输出根目录。
        specs: 所有工况的实际配置。
        snapshot: 执行源码与构建产物快照。
        resume: 是否显式恢复同一试验。
    """
    output = Path(output)
    identity = {"schema": 1, "specs": specs, "execution": snapshot}
    identity["fingerprint"] = digest(identity)
    path = output / "suite_binding.json"
    if output.exists() and any(output.iterdir()):
        if not resume or not path.exists() or json.loads(path.read_text()) != identity:
            raise ValueError(
                "Output is not an explicitly resumed identical suite; use a new directory"
            )
    else:
        output.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(identity, indent=2) + "\n")
    return identity


def seal_case(output, name, spec, fingerprint):
    """完成运行后封存生成输入、结果和全部遥测摘要。

    Args:
        output: 单工况目录。
        name: 工况名称。
        spec: 实际执行配置。
        fingerprint: 套件执行指纹。
    """
    output = Path(output)
    paths = [
        p
        for p in output.rglob("*")
        if p.is_file()
        and p.suffix in {".json", ".csv", ".sdf", ".xml", ".png", ".npz"}
        and p.name not in {"case_binding.json", "progress.json"}
    ]
    record = {
        "schema": 1,
        "case": name,
        "spec": spec,
        "fingerprint": fingerprint,
        "files": hashes(paths, output),
    }
    (output / "case_binding.json").write_text(json.dumps(record, indent=2) + "\n")


def verify_case(output, name, expected, fingerprint=None):
    """核对工况、阶段及证据内容；不能给旧数据贴上新工况标签。

    Args:
        output: 单工况目录。
        name: 预期名称。
        expected: 预期实际工况配置。
        fingerprint: 恢复执行时要求的指纹；离线分析可省略。
    """
    output = Path(output)
    bound = json.loads((output / "case_binding.json").read_text())
    if bound["case"] != name or bound["spec"] != expected:
        raise ValueError("Recorded case/spec differs from requested case/spec")
    if fingerprint is not None and bound["fingerprint"] != fingerprint:
        raise ValueError("Execution fingerprint mismatch")
    required = {"result.json", "run_status.json", "stages.json", "propulsion_state.csv"}
    if not required.issubset(bound["files"]):
        raise ValueError("Missing required evidence hashes")
    for name_on_disk, expected_hash in bound["files"].items():
        if file_hash(output / name_on_disk) != expected_hash:
            raise ValueError("Evidence changed: " + name_on_disk)
    result = json.loads((output / "result.json").read_text())
    status = json.loads((output / "run_status.json").read_text())
    if result["spec"] != expected or status["spec"] != expected:
        raise ValueError("Actual runtime spec mismatch")
    stages = json.loads((output / "stages.json").read_text())
    if len(stages) != len(expected["stages"]):
        raise ValueError("Incomplete stage sequence")
    for actual, requested in zip(stages, expected["stages"]):
        if any(actual.get(key) != value for key, value in requested.items()):
            raise ValueError("Executed stage differs from requested stage")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=["begin-build", "record-build"])
    args = parser.parse_args()
    if args.action == "begin-build":
        begin_build()
    else:
        record_build()
