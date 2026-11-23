#!/usr/bin/env python3
"""依据独立门槛评估已完成的操纵工况，不修改原始试验数据。

Independent physical metrics for direct-command manoeuvres, no fitted limits.
"""
import argparse
import hashlib
import json
import math
import sys
from pathlib import Path
import numpy as np
import xml.etree.ElementTree as E

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/lib"))


def _load_evidence_verifier():
    """在项目共享库路径配置完成后加载证据完整性校验函数。"""
    from trial_evidence import verify_case

    return verify_case


verify_case = _load_evidence_verifier()


def speed_rate(x):
    """通过展开航向的时间拟合计算平均偏航角速度。

    Args:
        x: 按时间排序的推进状态矩阵
    """
    h = np.unwrap(x[:, 14])
    return float(np.polyfit(x[:, 0] - x[0, 0], h, 1)[0])


def path_distance(x):
    """累加平面位置记录得到航迹长度。

    Args:
        x: 含时间和位置的推进状态矩阵
    """
    return float(np.linalg.norm(np.diff(x[:, 12:14], axis=0), axis=1).sum())


def stage_metrics(x):
    """计算工况持续时间及末段速度、姿态统计。

    Args:
        x: 该阶段的推进状态矩阵
    """
    tail = x[x[:, 0] > x[-1, 0] - 15]
    return {
        "duration_s": float(x[-1, 0] - x[0, 0]),
        "tail_surge_m_s": float(tail[:, 10].mean()),
        "tail_speed_std_m_s": float(tail[:, 10].std()),
        "tail_yaw_deg_s": math.degrees(speed_rate(tail)),
        "heading_change_deg": float(np.degrees(np.unwrap(x[:, 14])[-1] - x[0, 14])),
        "distance_m": path_distance(x),
    }


def turning_metrics(x):
    """计算回转半径、进距与战术直径等操纵指标。

    Args:
        x: 回转阶段的推进状态矩阵
    """
    tail = x[x[:, 0] > x[-1, 0] - 20]
    delta = np.unwrap(tail[:, 14])
    theta = abs(delta[-1] - delta[0])
    radius = float(path_distance(tail) / theta) if theta > 1e-6 else None
    q = x[:, 12:14] - x[0, 12:14]
    h = x[0, 14]
    along = q[:, 0] * np.cos(h) + q[:, 1] * np.sin(h)
    across = -q[:, 0] * np.sin(h) + q[:, 1] * np.cos(h)
    dh = np.abs(np.unwrap(x[:, 14]) - x[0, 14])
    out = {"radius_m": radius}
    for angle, label in [(90, "advance_90_m"), (180, "tactical_diameter_m")]:
        hit = np.flatnonzero(dh >= np.radians(angle))
        out[label] = (
            float(along[hit[0]] if angle == 90 else abs(across[hit[0]]))
            if len(hit)
            else None
        )
    return out


def stop_metrics(x, reverse=False):
    """识别滑行停止或反推纵向过零并记录对应位移。

    Args:
        x: 停止阶段的推进状态矩阵
        reverse: 是否按反推纵向速度过零判定
    """
    if reverse:
        hit = np.flatnonzero(x[:, 10] <= 0)
    else:
        horizontal = np.linalg.norm(np.gradient(x[:, 12:14], x[:, 0], axis=0), axis=1)
        hit = np.array(
            [
                i
                for i in np.flatnonzero(horizontal < 0.1)
                if x[-1, 0] - x[i, 0] >= 1
                and np.all(
                    horizontal[(x[:, 0] >= x[i, 0]) & (x[:, 0] <= x[i, 0] + 1)] < 0.1
                )
            ],
            dtype=int,
        )
    if not len(hit):
        return {"stop_time_s": None, "stop_distance_m": None}
    i = hit[0]
    return {
        "stop_time_s": float(x[i, 0] - x[0, 0]),
        "stop_distance_m": path_distance(x[: i + 1]),
        "surge_at_detection_m_s": float(x[i, 10]),
        "sway_at_detection_m_s": float(x[i, 11]),
        "definition": (
            "first nonpositive surge (remaining lateral motion reported)"
            if reverse
            else "horizontal speed <0.1 m/s sustained for 1 s"
        ),
    }


def recovery_metrics(trace, yaw_limit):
    """检查回零舵后转向停止所需时间及额外航向变化。

    Args:
        trace: 从回零舵指令开始的推进状态矩阵。
        yaw_limit: 稳态允许的偏航角速度绝对值，度/秒。
    """
    time = trace[:, 0]
    heading = np.degrees(np.unwrap(trace[:, 14]))
    rates = np.gradient(heading, time)
    settled = []
    for index in np.flatnonzero(abs(rates) < yaw_limit):
        window = (time >= time[index]) & (time <= time[index] + 5)
        if time[-1] - time[index] >= 5 and np.all(abs(rates[window]) < yaw_limit):
            settled.append(index)
    return {
        "extra_heading_deg": float(np.max(abs(heading - heading[0]))),
        "settling_time_s": float(time[settled[0]] - time[0]) if settled else None,
        "peak_yaw_deg_s": float(np.max(abs(rates))),
    }


def valid_trace(trace):
    """拒绝重复时间、缺样和非有限遥测，避免回归统计误判。

    Args:
        trace: 第一列为仿真时间的采样矩阵。
    """
    return bool(
        len(trace) >= 5
        and np.isfinite(trace).all()
        and np.all(np.diff(trace[:, 0]) > 0)
        and np.max(np.diff(trace[:, 0])) < 0.5
    )


def servo_checks(model, actuators):
    """分别核对实际关节上限和参考轨迹速率。

    Args:
        model: 已保存的实际 SDF 模型元素。
        actuators: 执行器遥测矩阵。
    """
    actual_limit = float(model.findtext("joint[@name='steering']/axis/limit/velocity"))
    reference_limit = float(
        model.findtext("plugin[@name='usv::AzimuthPropulsion']/steering/rate")
    )
    actual_peak = float(np.max(abs(actuators[:, 2])))
    reference_peak = float(
        np.max(abs(np.diff(actuators[:, 3]) / np.diff(actuators[:, 0])))
    )
    return {
        "servo_rate": actual_peak <= actual_limit + 1e-6,
        "reference_rate": reference_peak <= reference_limit + 1e-6,
    }, {
        "actual_peak_rad_s": actual_peak,
        "actual_limit_rad_s": actual_limit,
        "reference_peak_rad_s": reference_peak,
        "reference_limit_rad_s": reference_limit,
    }


def held_stage_checks(stage, trace, actuators, criteria):
    """检查直接指令及稳定阶段的实际舵角和速度。

    Args:
        stage: 保存的阶段配置与时间边界。
        trace: 该阶段的推进遥测。
        actuators: 整次试验的执行器遥测。
        criteria: 工程验收门槛。
    """
    settled = trace[trace[:, 0] >= stage["start_sim"] + 1]
    checks = {
        "throttle_tracks": bool(
            len(settled) and np.max(abs(settled[:, 1] - stage["throttle"])) < 1e-6
        )
    }
    if "until_heading_deg" not in stage:
        tail = actuators[
            (actuators[:, 0] > stage["end_sim"] - 3)
            & (actuators[:, 0] <= stage["end_sim"])
        ]
        checks["servo_tracks"] = bool(
            len(tail)
            and np.max(abs(np.degrees(tail[:, 1]) - stage.get("steering_deg", 0)))
            < criteria["max_servo_error_deg"]
        )
    return checks


def motion_checks(stage, trace, metrics, criteria):
    """按阶段类型评估操纵品质，不能用运行成功替代物理判据。

    Args:
        stage: 当前阶段配置。
        trace: 当前阶段遥测。
        metrics: 可补充的阶段指标映射。
        criteria: 工程验收门槛。
    """
    name = stage["name"]
    checks = {}
    if name in ["neutral", "recover"]:
        checks.update(
            yaw=abs(metrics["tail_yaw_deg_s"]) < criteria["max_neutral_yaw_deg_s"],
            steady_speed=metrics["tail_speed_std_m_s"]
            < criteria["max_tail_speed_std_m_s"],
            forward=metrics["tail_surge_m_s"] > 0.3,
        )
    if name == "recover":
        metrics.update(recovery_metrics(trace, criteria["max_recovered_yaw_deg_s"]))
        checks["return_heading"] = (
            metrics["extra_heading_deg"] < criteria["max_recovery_heading_deg"]
        )
        checks["return_time"] = (
            metrics["settling_time_s"] is not None
            and metrics["settling_time_s"] < criteria["max_recovery_time_s"]
        )
    if name in ["turn", "astern_turn"]:
        metrics.update(turning_metrics(trace))
        direction = -np.sign(stage["steering_deg"]) * np.sign(stage["throttle"])
        checks.update(
            direction=metrics["heading_change_deg"] * direction > 90,
            radius=metrics["radius_m"] is not None
            and criteria["min_turn_radius_lengths"] * criteria["length_reference_m"]
            < metrics["radius_m"]
            < criteria["max_turn_radius_lengths"] * criteria["length_reference_m"],
            half_circle=metrics["tactical_diameter_m"] is not None,
            steady_speed=metrics["tail_speed_std_m_s"]
            < criteria["max_tail_speed_std_m_s"],
        )
    if name in ["coast", "reverse_brake"]:
        metrics.update(stop_metrics(trace, name == "reverse_brake"))
        key = (
            "max_reverse_brake_stop_distance_lengths"
            if name == "reverse_brake"
            else "max_coast_stop_distance_lengths"
        )
        checks["stop"] = (
            metrics["stop_distance_m"] is not None
            and metrics["stop_distance_m"]
            < criteria[key] * criteria["length_reference_m"]
        )
    if name == "reverse":
        checks.update(
            astern=metrics["tail_surge_m_s"] < -0.2,
            directional_quality=abs(metrics["tail_yaw_deg_s"])
            < criteria["max_reverse_yaw_deg_s"],
        )
    return checks


def zigzag_checks(stages, traces, criteria):
    """从实际航向核对反舵触发及各次超越角。

    Args:
        stages: 实际执行阶段列表。
        traces: 与阶段一一对应的推进遥测。
        criteria: 工程验收门槛。
    """
    checks = {}
    overshoots = []
    for index, (stage, trace) in enumerate(zip(stages, traces)):
        if "until_heading_deg" not in stage:
            continue
        reference = stage["heading_reference_rad"]
        heading = np.degrees(np.unwrap(trace[:, 14]) - reference)
        target = stage["until_heading_deg"]
        checks[stage["name"] + "_trigger"] = bool(
            stage.get("heading_triggered")
            and heading[-1] * np.sign(target) >= abs(target) - 1
        )
        if index and "until_heading_deg" in stages[index - 1]:
            previous = stages[index - 1]["until_heading_deg"]
            overshoots.append(
                max(0.0, float(np.max(heading * np.sign(previous)) - abs(previous)))
            )
    if overshoots:
        checks["zigzag_overshoots"] = (
            len(overshoots) == 3
            and max(overshoots) < criteria["max_zigzag_overshoot_deg"]
        )
    return checks, overshoots


def environment_quality(folder, trace, expected, criteria):
    """测量环境响应与浸水，按预先给定的使用范围门槛独立评估。

    Args:
        folder: 完整遥测目录。
        trace: 按时间排序的推进遥测。
        expected: 运行工况配置。
        criteria: 预先声明的工程筛查门槛。
    """
    wave = np.loadtxt(folder / "wave_response_state.csv", delimiter=",", ndmin=2)
    wave = wave[wave[:, 0] >= trace[0, 0] + 10]
    active = trace[trace[:, 0] >= trace[0, 0] + 10]
    roll, pitch = np.degrees(np.max(abs(active[:, 15:17]), axis=0))
    metrics = {
        "max_roll_deg": float(roll),
        "max_pitch_deg": float(pitch),
        "minimum_propeller_wet_fraction": float(active[:, 8].min()),
        "partial_immersion_fraction": float(np.mean(active[:, 8] < 0.99)),
        "wave_height_range_m": float(np.ptp(wave[:, 1])),
        "heave_std_m": float(np.std(wave[:, 6])),
        "heading_range_deg": float(np.degrees(np.ptp(np.unwrap(active[:, 14])))),
    }
    checks = {
        "roll_envelope": roll < criteria["environment_max_roll_deg"],
        "pitch_envelope": pitch < criteria["environment_max_pitch_deg"],
        "propeller_immersion": metrics["minimum_propeller_wet_fraction"]
        >= criteria["minimum_propeller_wet_fraction"],
        "wave_trace_valid": valid_trace(wave),
    }
    if expected.get("wave_gain", 0) > 0:
        checks["waves_present"] = metrics["wave_height_range_m"] > 0.02
        checks["heave_responds"] = metrics["heave_std_m"] > 0.003
    return checks, metrics


def assess_case(folder, criteria, expected, fingerprint):
    """核对版本绑定后评估运行完整性与操纵品质。

    Args:
        folder: 已封存的单工况目录。
        criteria: 工程验收门槛。
        expected: 本次要求的实际工况配置。
        fingerprint: 套件运行指纹。
    """
    result = verify_case(folder, folder.name, expected, fingerprint)
    stages = json.loads((folder / "stages.json").read_text())
    trace = np.loadtxt(folder / "propulsion_state.csv", delimiter=",", ndmin=2)
    actuators = np.loadtxt(folder / "actuators_state.csv", delimiter=",", ndmin=2)
    if not valid_trace(trace) or not valid_trace(actuators):
        raise ValueError("Nonfinite, unordered or incomplete telemetry")
    model = E.parse(folder / "boat.sdf").find("model")
    health = json.loads((folder / "health.json").read_text())
    checks, servo = servo_checks(model, actuators)
    checks.update(
        runtime=result["passed"],
        evidence_bound=True,
        direct_only=bool(health)
        and all(
            h["command_mode"] in ["direct", "idle"]
            and h.get("heading_reference_rad") is None
            for h in health
        ),
        attitude=bool(
            np.max(abs(trace[:, 15:17])) < np.radians(criteria["max_roll_pitch_deg"])
        ),
    )
    metrics = {"servo": servo}
    traces = []
    for stage in stages:
        selected = trace[
            (trace[:, 0] >= stage["start_sim"]) & (trace[:, 0] <= stage["end_sim"])
        ]
        if len(selected) < 5:
            raise ValueError("Missing stage samples")
        traces.append(selected)
        stage_result = stage_metrics(selected)
        stage_checks = held_stage_checks(stage, selected, actuators, criteria)
        stage_checks.update(motion_checks(stage, selected, stage_result, criteria))
        checks.update(
            {stage["name"] + "_" + key: value for key, value in stage_checks.items()}
        )
        metrics[stage["name"]] = stage_result
    zig_checks, overshoots = zigzag_checks(stages, traces, criteria)
    checks.update(zig_checks)
    if overshoots:
        metrics["zigzag_overshoots_deg"] = overshoots
    if expected.get("kind") == "environment":
        env_checks, metrics["environment"] = environment_quality(
            folder, trace, expected, criteria
        )
        checks.update(env_checks)
    checks = {key: bool(value) for key, value in checks.items()}
    return {
        "passed": all(checks.values()),
        "runtime_passed": result["passed"],
        "checks": checks,
        "metrics": metrics,
    }


def main():
    """读取工况与判据，保存独立操纵验收结果。"""
    ap = argparse.ArgumentParser()
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--criteria", type=Path, required=True)
    ap.add_argument("--spec", type=Path, required=True)
    ap.add_argument(
        "--suite-kind", choices=["maneuvers", "environment"], default="maneuvers"
    )
    args = ap.parse_args()
    c = json.loads(args.criteria.read_text())
    spec = json.loads(args.spec.read_text())
    binding = json.loads((args.output / "suite_binding.json").read_text())
    if binding["specs"] != spec:
        raise ValueError("Requested suite differs from recorded executed suite")
    cases = {}
    for name in spec:
        try:
            cases[name] = assess_case(
                args.output / name, c, spec[name], binding["fingerprint"]
            )
        except Exception as e:
            cases[name] = {"passed": False, "failure": str(e)}
    comparisons = {}
    for label in ["055", "100"] if args.suite_kind == "maneuvers" else []:
        try:
            left_radius = cases["turn_" + label + "_left"]["metrics"]["turn"][
                "radius_m"
            ]
            r = cases["turn_" + label + "_right"]["metrics"]["turn"]["radius_m"]
            difference = float(abs(left_radius - r) / ((left_radius + r) / 2))
            comparisons["turn_" + label] = {
                "relative_radius_difference": difference,
                "passed": difference
                < c["max_port_starboard_radius_relative_difference"],
            }
        except (KeyError, TypeError):
            comparisons["turn_" + label] = {"passed": False}
    if args.suite_kind == "maneuvers":
        try:
            u = [
                cases["neutral_" + k]["metrics"]["neutral"]["tail_surge_m_s"]
                for k in ["035", "065", "100"]
            ]
            comparisons["speed_monotonic"] = {
                "passed": u[0] < u[1] < u[2],
                "speeds_m_s": u,
            }
        except KeyError:
            comparisons["speed_monotonic"] = {"passed": False}
    out = {
        "passed": all(v["passed"] for v in [*cases.values(), *comparisons.values()]),
        "execution_fingerprint": binding["fingerprint"],
        "criteria": c,
        "criteria_sha256": hashlib.sha256(args.criteria.read_bytes()).hexdigest(),
        "assessor_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "spec_sha256": hashlib.sha256(args.spec.read_bytes()).hexdigest(),
        "cases": cases,
        "comparisons": comparisons,
    }
    (args.output / "maneuver_assessment.json").write_text(
        json.dumps(out, indent=2) + "\n"
    )
    print(json.dumps({k: v["passed"] for k, v in cases.items()}))
    raise SystemExit(0 if out["passed"] else 1)


if __name__ == "__main__":
    main()
