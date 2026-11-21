#!/usr/bin/env python3

# Resolve the checkout independently of the current working directory.
"""分析已保存性能工况的遥测与判定。"""

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
import json
from pathlib import Path
from performance_analysis import analyze

p = argparse.ArgumentParser()
p.add_argument("output", type=Path)
p.add_argument("--recover-json-error", action="store_true")
a = p.parse_args()
out = a.output
if a.recover_json_error and not (out / "run_status.json").exists():
    console = (out.parent / (out.name + "_console.log")).read_text()
    assert (
        "TypeError: Object of type bool_ is not JSON serializable" in console
    ), "Not a verified serialization-only failure"
    spec = json.loads((out.parent / "spec.json").read_text())[out.name]
    stages = json.loads((out / "stages.json").read_text())
    assert len(stages) == len(spec["stages"])
    start = json.loads((out / "startup.json").read_text())
    assert start["ready"]
    (out / "run_status.json").write_text(
        json.dumps(
            {
                "case": out.name,
                "spec": spec,
                "finished_stages": len(stages),
                "start_sim": start["sim_s"],
                "failure": None,
                "bridge_resumed": False,
                "postprocess_recovered": True,
                "recovery_reason": "All timed stages saved; original analysis failed only at JSON encoding of numpy bool. Recomputed from preserved raw CSV, without rerunning or changing physics.",
            },
            indent=2,
        )
    )
r = analyze(out)
print(
    json.dumps(
        {
            "case": r["case"],
            "passed": r["passed"],
            "failed_checks": [k for k, v in r["checks"].items() if not v],
        },
        indent=2,
    )
)
