#!/usr/bin/env python3
"""调度性能试验工况并在中断时清理子进程。

Isolated physics trials with bounded cancellation and preserved evidence.
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
from project_paths import tool_path
from trial_evidence import execution_snapshot, initialize_suite, seal_case, verify_case
import argparse
import fcntl
import json
import signal
import subprocess
import sys
import threading
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument("--spec", type=Path, required=True)
p.add_argument("--output", type=Path, required=True)
p.add_argument("--skip-case", action="append", default=[])
p.add_argument("--workers", type=int, default=4)
p.add_argument(
    "--resume", action="store_true", help="Resume only identical version-bound results"
)
a = p.parse_args()
cases = json.loads(a.spec.read_text())
snapshot = execution_snapshot()
binding = initialize_suite(a.output, cases, snapshot, a.resume)
active = {}
guard = threading.Lock()
stop = threading.Event()


def run(case):
    """在中断与跳过策略下执行单个性能工况。

    Args:
        case: 工况名称
    """
    if case in a.skip_case or stop.is_set():
        return
    with (a.output / (case + ".lock")).open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        result = a.output / case / "result.json"
        if stop.is_set():
            return
        if result.exists():
            saved = verify_case(
                result.parent, case, cases[case], binding["fingerprint"]
            )
            if not saved["passed"]:
                raise ValueError("Failed case retained; use a new output directory")
            print("REUSE", case, flush=True)
            return 0
        if result.parent.exists() and any(result.parent.iterdir()):
            raise ValueError("Incomplete attempt retained; use a new output directory")
        if execution_snapshot() != snapshot:
            raise ValueError("Execution inputs changed before case start")
        print("START", case, flush=True)
        with (a.output / (case + "_console.log")).open("w") as log:
            with guard:
                if stop.is_set():
                    return
                proc = subprocess.Popen(
                    [
                        sys.executable,
                        str(tool_path("validate_vessel_performance.py")),
                        "--case",
                        case,
                        "--spec",
                        str(a.spec.resolve()),
                        "--output",
                        str((a.output / case).resolve()),
                    ],
                    stdout=log,
                    stderr=subprocess.STDOUT,
                )
                active[case] = proc
            code = proc.wait()
            # Also clean up after a native abort, which bypasses the child's finally/atexit.
            metadata = a.output / case / "source_versions.json"
            if metadata.exists():
                partition = json.loads(metadata.read_text())["partition"]
                subprocess.run(
                    [
                        sys.executable,
                        str(tool_path("stop_test_partition.py")),
                        partition,
                    ],
                    stdout=subprocess.DEVNULL,
                )
            with guard:
                active.pop(case, None)
        if execution_snapshot() != snapshot:
            raise ValueError(
                "Execution inputs changed during trial; evidence not reusable"
            )
        if code == 0 and result.exists():
            seal_case(result.parent, case, cases[case], binding["fingerprint"])
            verify_case(result.parent, case, cases[case], binding["fingerprint"])
        elif code != 0:
            print(f"FAIL {case}: exit {code}; evidence retained", flush=True)
        print("END", case, code, flush=True)
        return code


def interrupt(signum, frame):
    """将信号转换为中断以结束工况调度。

    Args:
        signum: 信号编号
        frame: 信号发生时的 Python 栈帧
    """
    raise KeyboardInterrupt


signal.signal(signal.SIGTERM, interrupt)
priority = [
    k
    for k in ["clock_independent", "neutral_rudder", "curve_0_100", "endurance_10min"]
    if k in cases
]
ordered = priority + [k for k in cases if k not in priority]
pool = ThreadPoolExecutor(max_workers=a.workers)
try:
    exit_codes = dict(zip(ordered, pool.map(run, ordered)))
except BaseException:
    stop.set()
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    with guard:
        for proc in active.values():
            if proc.poll() is None:
                proc.terminate()
    raise
finally:
    pool.shutdown(wait=True, cancel_futures=True)
results = {
    case: (
        json.loads((a.output / case / "result.json").read_text())
        if (a.output / case / "result.json").exists()
        else {"passed": False, "failure": "no result"}
    )
    for case in cases
}
unchanged = execution_snapshot() == snapshot
summary = {
    "passed": unchanged
    and not a.skip_case
    and all(code == 0 for code in exit_codes.values())
    and all(r["passed"] for r in results.values()),
    "exit_codes": exit_codes,
    "execution_unchanged": unchanged,
    "skipped": a.skip_case,
    "fingerprint": binding["fingerprint"],
    "cases": results,
}
(a.output / "summary.json").write_text(json.dumps(summary, indent=2))
raise SystemExit(0 if summary["passed"] else 1)
