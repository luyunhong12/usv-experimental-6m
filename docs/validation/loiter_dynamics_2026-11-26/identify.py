"""从既有开环记录辨识局部偏航响应，保留跨片段预测误差。

这是本次证据的复现附件，不是运行控制器。固定使用静水左右满舵片段拟合，
回中及有流片段只验证，不反向优化系数。所有角度内部为rad、时间为s。
"""

import hashlib
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / "artifacts/diagnostics/loiter_dynamics_20261126/model"


def load_case(label):
    """读取并按推进遥测时钟对齐已完成记录。

    Args:
        label: artifacts/navigation下的既有开环试验目录名。

    Returns:
        时间、阶段、实际横向推进力、偏航角速度、其导数和输入哈希。
    """
    folder = ROOT / "artifacts/navigation" / label / "run"
    names = ["propulsion.csv", "actuators.csv", "response.csv"]
    p, a, stages = [np.loadtxt(folder / name, delimiter=",") for name in names]
    t = p[:, 0]
    if not np.all(np.diff(t) > 0) or not np.all(np.isfinite(p)):
        raise ValueError("推进遥测时间或数值无效")
    delta = np.interp(t, a[:, 0], a[:, 1])
    # 使用最近的已开始阶段；不把离散阶段线性插值成伪阶段。
    indices = np.clip(
        np.searchsorted(stages[:, 0], t, side="right") - 1, 0, len(stages) - 1
    )
    stage = stages[indices, 2].astype(int)
    rate = np.gradient(np.unwrap(p[:, 14]), t)
    force = p[:, 4] * np.sin(delta)
    hashes = {
        name: hashlib.sha256((folder / name).read_bytes()).hexdigest() for name in names
    }
    return t, stage, force, rate, np.gradient(rate, t), hashes


def predict(t, force, rate, mask, coef):
    """从片段首个实测转速递推，随后不再用实测转速校正。

    Args:
        t: 严格递增的仿真秒数组。
        force: 实际横向推进力，N，按舵机局部符号。
        rate: 世界ENU偏航角速度，rad/s，仅取初始值及计算误差。
        mask: 单个连续验证片段的布尔掩码。
        coef: dr/dt=a*r+b*force+c的三个最小二乘系数。

    Returns:
        递推样本表和以deg/s表示的RMSE、峰值误差。
    """
    ids = np.flatnonzero(mask)
    if len(ids) < 20 or not np.all(np.diff(ids) == 1):
        raise ValueError("预测片段必须连续且有足够样本")
    value = rate[ids[0]]
    rows = []
    for i in ids:
        rows.append([t[i], rate[i], value, force[i]])
        dt = t[min(i + 1, len(t) - 1)] - t[i]
        decay = np.exp(coef[0] * dt)
        value = decay * value + np.expm1(coef[0] * dt) / coef[0] * (
            coef[1] * force[i] + coef[2]
        )
    rows = np.asarray(rows)
    error = np.rad2deg(rows[:, 2] - rows[:, 1])
    return rows, {
        "rmse_deg_s": float(np.sqrt(np.mean(error**2))),
        "peak_deg_s": float(np.max(abs(error))),
    }


def main():
    """固定训练/验证划分，输出模型、预测和来源，不给出导航通过结论。"""
    OUT.mkdir(parents=True, exist_ok=False)
    labels = {
        key: f"response_{key}_20261125_02" for key in ["calm", "north030", "west015"]
    }
    data = {key: load_case(label) for key, label in labels.items()}
    t, stage, force, rate, derivative, _ = data["calm"]
    result = {
        "source_cases": labels,
        "hashes": {key: value[-1] for key, value in data.items()},
        "models": {},
    }
    for name, train_stages, validate_stage in [
        ("forward", [2, 3], 4),
        ("reverse", [6, 7], 8),
    ]:
        mask = np.isin(stage, train_stages)
        matrix = np.column_stack([rate, force, np.ones(len(t))])
        coef, _, rank, singular = np.linalg.lstsq(
            matrix[mask], derivative[mask], rcond=None
        )
        if rank != 3 or coef[0] >= 0:
            raise ValueError("辨识缺秩或一阶模型不稳定")
        entry = {
            "coef_a_b_c": coef.tolist(),
            "time_constant_s": float(-1 / coef[0]),
            "steady_force_gain_rad_s_N": float(-coef[1] / coef[0]),
            "training_stages": train_stages,
            "training_samples": int(mask.sum()),
            "singular_values": singular.tolist(),
            "predictions": {},
        }
        for key, (tt, ss, ff, rr, _, _) in data.items():
            for part, select in [
                ("turns", np.isin(ss, train_stages)),
                ("recenter", ss == validate_stage),
            ]:
                rows, metrics = predict(tt, ff, rr, select, coef)
                filename = f"{name}_{key}_{part}.csv"
                np.savetxt(
                    OUT / filename,
                    rows,
                    delimiter=",",
                    header="sim_s,measured_yaw_rate_rad_s,predicted_yaw_rate_rad_s,measured_lateral_thrust_N",
                    comments="",
                )
                entry["predictions"][key + "_" + part] = {
                    **metrics,
                    "is_training_interval": key == "calm" and part == "turns",
                    "csv": filename,
                }
        result["models"][name] = entry
    (OUT / "model.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result["models"], indent=2))


if __name__ == "__main__":
    main()
