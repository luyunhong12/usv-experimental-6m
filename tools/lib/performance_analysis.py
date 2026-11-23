"""分析航行遥测并计算工况性能指标。

Reproducible analysis of already-saved native physics/ROS traces.
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
import json
import math
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as E
import numpy as np


def analyze(out):
    """读取性能记录并计算本轮工况指标。

    Args:
        out: 包含运行状态和遥测文件的结果目录
    """
    out = Path(out)
    meta = json.loads((out / "run_status.json").read_text())
    spec = meta["spec"]
    a = SimpleNamespace(case=meta["case"])
    stage_log = json.loads((out / "stages.json").read_text())
    stages = spec["stages"]
    stage = meta["finished_stages"]
    start_sim = meta["start_sim"]
    failure = meta["failure"]
    resumed = meta["bridge_resumed"]
    health = json.loads((out / "health.json").read_text())
    bm = E.parse(out / "boat.sdf").find("model")
    energy = bm.find("plugin[@name='usv::AzimuthPropulsion']/energy")
    rows = {
        k: np.loadtxt(
            out / (k.replace("/", "_") + ".csv"), delimiter=",", ndmin=2
        ).tolist()
        for k in [
            "propulsion/state",
            "energy/truth",
            "actuators/state",
            "environment/state",
            "fuel_mass/state",
            "hydrodynamics/resistance",
        ]
    }
    d = np.array(rows["propulsion/state"])
    e = np.array(rows["energy/truth"])
    m = np.array(rows["fuel_mass/state"])
    dr = np.array(rows["hydrodynamics/resistance"])
    act = np.array(rows["actuators/state"])
    checks = {
        "startup_ready": start_sim is not None,
        "completed": stage == len(stages),
        "no_failure": failure is None,
        "finite": all(len(v) and np.isfinite(v).all() for v in rows.values()),
    }
    metrics = {}
    if all(len(x) for x in [d, e, m, dr, act]):
        dry = float(
            bm.find("plugin[@name='usv::AzimuthPropulsion']/fuel_mass/dry_mass").text
        )
        checks.update(
            attitude_bounded=bool(np.max(abs(d[:, 15:17])) < math.radians(20)),
            power_limited=bool(e[:, 7].max() <= 40000.01),
            energy_balance=bool(np.max(abs(e[:, 21])) < 1e-6),
            fuel_mass_consistent=bool(
                np.max(abs(m[:, 2] - dry - 0.84 * m[:, 1])) < 1e-7
            ),
            resistance_dissipative=bool(dr[:, 9].min() > -1e-8),
            command_reached_physics=bool(d[:, 1].max() > 0.1 or d[:, 1].min() < -0.1),
            health_command_observed=any(h["command_sequence"] > 10 for h in health),
            initial_soc_matches=bool(
                abs(e[0, 1] - float(energy.findtext("initial_soc"))) < 1e-4
            ),
        )
        for s in stage_log:
            v = d[(d[:, 0] >= s["start_sim"]) & (d[:, 0] < s["end_sim"])]
            tail = v[v[:, 0] > s["end_sim"] - 8]
            ee = e[(e[:, 0] > s["end_sim"] - 8) & (e[:, 0] < s["end_sim"])]
            rr = dr[(dr[:, 0] > s["end_sim"] - 8) & (dr[:, 0] < s["end_sim"])]
            heading = np.unwrap(v[:, 14])
            delta = heading[-1] - heading[0]
            distance = float(np.linalg.norm(np.diff(v[:, 12:14], axis=0), axis=1).sum())
            metric = {
                "tail_speed_m_s": float(tail[:, 10].mean()),
                "tail_speed_std_m_s": float(tail[:, 10].std()),
                "mean_motor_w": float(ee[:, 7].mean()),
                "distance_m": distance,
                "heading_change_rad": float(delta),
                "mean_wet_area_m2": float(rr[:, 1].mean()),
                "mean_projected_x_m2": float(rr[:, 2].mean()),
                "mean_surge_drag_n": float(-rr[:, 10].mean()),
            }
            if s.get("steering_deg", 0):
                tail_heading = np.unwrap(tail[:, 14])
                dtheta = tail_heading[-1] - tail_heading[0]
                metric["tail_turn_radius_m"] = (
                    float(
                        np.linalg.norm(np.diff(tail[:, 12:14], axis=0), axis=1).sum()
                        / abs(dtheta)
                    )
                    if abs(dtheta) > 0.01
                    else None
                )
            if s["name"] in ["coast", "reverse_brake"]:
                stopped = v[abs(v[:, 10]) < 0.1]
                metric["time_to_abs_speed_0_1_s"] = (
                    float(stopped[0, 0] - s["start_sim"]) if len(stopped) else None
                )
                if len(stopped):
                    metric["distance_to_abs_speed_0_1_m"] = float(
                        np.linalg.norm(
                            np.diff(v[v[:, 0] <= stopped[0, 0], 12:14], axis=0), axis=1
                        ).sum()
                    )
            metrics[s["name"]] = metric
            if s["name"] == "fixed":
                checks["steady_fixed_throttle"] = (
                    metric["tail_speed_std_m_s"] < 0.12
                    and metric["tail_speed_m_s"] > 0.2
                )
                checks["fixed_throttle_exact"] = bool(
                    np.max(abs(tail[:, 1] - s["throttle"])) < 1e-6
                )
                checks["heading_held"] = abs(delta) < 0.1
            if s["name"] == "reverse":
                checks["reverses"] = metric["tail_speed_m_s"] < -0.2
            if s["name"] == "reverse_brake":
                checks["reverse_brakes"] = metric["time_to_abs_speed_0_1_s"] is not None
            if s["name"] == "turn":
                checks["turn_direction"] = bool(
                    delta * s["steering_deg"] < -0.5
                    and metric.get("tail_turn_radius_m") is not None
                )
            if s["name"].startswith("zig"):
                checks[s["name"] + "_direction"] = bool(
                    (heading[-1] - heading[len(heading) // 2]) * s["steering_deg"]
                    < -0.05
                )
        if spec.get("kind") == "neutral":
            selected = act[(act[:, 0] > start_sim + 35) & (act[:, 0] < start_sim + 44)]
            checks["neutral_servo_tracks"] = bool(
                np.max(abs(selected[:, 1])) < math.radians(0.5)
            )
        if spec.get("kind") == "clock_independent":
            checks["moves_without_ros_clock_bridge"] = bool(d[:, 10].max() > 1.0)
        if spec.get("kind") == "sensor_dropout":
            gap = d[(d[:, 0] > start_sim + 16) & (d[:, 0] < start_sim + 24)]
            checks.update(
                dropout_paused_and_resumed=resumed,
                dropout_stops_throttle=bool(len(gap) and abs(gap[:, 1]).max() < 1e-8),
                recovers_after_dropout=bool(d[d[:, 0] > start_sim + 35, 10].max() > 1),
                health_reports_dropout=any(
                    not h["checks"]["navigation_fresh"] and h["sim_s"] > start_sim + 13
                    for h in health
                ),
            )
        if spec.get("kind") == "command_dropout":
            gap = d[(d[:, 0] > start_sim + 16) & (d[:, 0] < start_sim + 24)]
            recovered = d[d[:, 0] > start_sim + 35]
            checks.update(
                command_timeout_stops_throttle=bool(
                    len(gap) and abs(gap[:, 1]).max() < 1e-8
                ),
                command_resumes_after_gap=bool(
                    len(recovered) and recovered[:, 1].min() > 0.49
                ),
            )
        if spec.get("kind") == "low_soc":
            checks["soc_derating"] = bool(
                e[-1, 1] < 0.15 and e[-1, 24] < e[e[:, 0] > start_sim + 5, 24][0]
            )
        if spec.get("kind") == "hot":
            checks["thermal_derating"] = bool(
                e[:, 4].max() > 45 and e[-1, 24] < e[e[:, 0] > start_sim + 5, 24][0]
            )
        if spec.get("kind") == "fuel_reserve":
            checks["fuel_reserve_fault"] = bool(
                e[-1, 5] <= 1.00001 and e[-1, 16] == 1 and e[-1, 10] == 0
            )
        if spec.get("generator_cycle"):
            states = e[:, 11].astype(int)
            checks["two_generator_runs"] = (
                int(np.count_nonzero((states[1:] == 3) & (states[:-1] != 3))) >= 2
            )
            checks["fuel_consumed"] = bool(e[0, 5] - e[-1, 5] > 0.01)
        summary = {
            "max_roll_pitch_deg": np.rad2deg(abs(d[:, 15:17]).max(axis=0)).tolist(),
            "soc_start_end": [float(e[0, 1]), float(e[-1, 1])],
            "temperature_max_c": float(e[:, 4].max()),
            "fuel_consumed_l": float(e[0, 5] - e[-1, 5]),
            "sim_end": float(d[-1, 0]),
        }
    else:
        summary = {}
    checks = {k: bool(v) for k, v in checks.items()}
    r = {
        "case": a.case,
        "spec": spec,
        "checks": checks,
        "passed": all(checks.values()),
        "failure": failure,
        "start_sim": start_sim,
        "metrics": metrics,
        "summary": summary,
        "scope": "Native Gazebo/ROS; GPU image generation omitted, all device masses/power retained; no ArduPilot",
    }
    r["postprocess_recovered"] = meta.get("postprocess_recovered", False)
    (out / "result.json").write_text(json.dumps(r, indent=2))
    return r
