#!/usr/bin/env python3
"""从记录的受力分项审计整船重心处外力矩收支。

Audit recorded external wrenches about whole-vessel COM; never fit forces.
"""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np


def analyze(folder, stage_index=0):
    """将记录的各类外力矩换算到整船重心并进行收支审计。

    Args:
        folder: 含 boat.sdf、推进状态、阶段及各分项受力 CSV 的结果目录；声明的插件缺少遥测时拒绝审计。
        stage_index: 审计阶段索引；负数按 Python 规则从末尾计数。
    """
    p = Path(folder)
    d = np.loadtxt(p / "propulsion_state.csv", delimiter=",", ndmin=2)
    stages = json.loads((p / "stages.json").read_text())
    end = stages[stage_index]["end_sim"]
    start = max(stages[stage_index]["start_sim"], end - 15)
    selected = d[(d[:, 0] > start) & (d[:, 0] < end)]
    if len(selected) < 5:
        raise ValueError("Insufficient samples in audit stage")
    t = selected[:, 0]
    roll = selected[:, 15]
    pitch = selected[:, 16]
    rz = np.column_stack(
        (-np.sin(pitch), np.cos(pitch) * np.sin(roll), np.cos(pitch) * np.cos(roll))
    )
    components = {}
    total = np.zeros((len(t), 6))
    topics = [
        ("hull", ["pressure", "hull_drag", "added_mass", "air"]),
        ("propulsion", ["thrust", "water_couple"]),
    ]
    model = E.parse(p / "boat.sdf").find("model")
    if model.find("plugin[@name='usv::AppendageHydrodynamics']") is not None:
        topics.append(("appendages", ["appendages"]))
    if model.find("plugin[@name='usv::PassiveStabilizer']") is not None:
        topics.append(("stabilizer", ["stabilizer_lift_drag_buoyancy"]))
    for topic, names in topics:
        filename = (
            "stabilizer_audit_appendages.csv"
            if topic == "stabilizer"
            else "audit_" + topic + ".csv"
        )
        a = np.loadtxt(p / filename, delimiter=",", ndmin=2)
        if a.shape[1] != 4 + 6 * len(names) or a[0, 0] > t[0] or a[-1, 0] < t[-1] - 0.2:
            raise ValueError("Incomplete audit: " + topic)
        for j, name in enumerate(names):
            w = np.column_stack(
                [np.interp(t, a[:, 0], a[:, 4 + 6 * j + k]) for k in range(6)]
            )
            total += w
            components[name] = {
                "mean_body_wrench_N_Nm": w.mean(axis=0).tolist(),
                "mean_world_vertical_moment_Nm": float(
                    np.mean(np.sum(rz * w[:, 3:], axis=1))
                ),
            }
    h = np.unwrap(selected[:, 14])
    rate = np.polyfit(t, h, 1)[0]
    return {
        "window_sim_s": [float(t[0]), float(t[-1])],
        "yaw_rate_deg_s": float(np.degrees(rate)),
        "speed_km_h": float(selected[:, 10].mean() * 3.6),
        "roll_deg": float(np.degrees(roll.mean())),
        "pitch_deg": float(np.degrees(pitch.mean())),
        "components": components,
        "total_external_world_vertical_moment_Nm": float(
            np.mean(np.sum(rz * total[:, 3:], axis=1))
        ),
        "note": "Gravity has zero resultant moment about whole-vessel COM; joint motor/servo torques are internal. Water-couple overrides are diagnostic counter-couples, not physical propulsion modes.",
    }


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("root", type=Path)
    ap.add_argument("--stage", type=int, default=0)
    args = ap.parse_args()
    results = {
        p.name: analyze(p, args.stage)
        for p in sorted(args.root.iterdir())
        if p.is_dir() and (p / "result.json").exists()
    }
    (
        args.root
        / (
            "force_audit.json"
            if args.stage == 0
            else f"force_audit_stage_{args.stage}.json"
        )
    ).write_text(json.dumps(results, indent=2) + "\n")
    print(json.dumps(results, indent=2))
