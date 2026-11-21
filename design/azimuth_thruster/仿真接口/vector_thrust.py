"""提供单机方位推进的平面力与力矩接口，不代替船体动力学。

Single azimuth thruster force interface; NOT a hull dynamics simulator.

Coordinates: model FLU (forward, left, up); ArduPilot uses FRD (forward, right, down).
Thrust is calibrated newtons supplied by caller, never guessed from throttle or kW.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Wrench:
    """保存水平推力及关于指定重心的偏航力矩。"""

    fx_n: float
    fy_n: float
    yaw_nm: float

    def as_frd(self):
        """将 FLU 水平力和偏航力矩转换为 FRD 约定。"""
        return Wrench(self.fx_n, -self.fy_n, -self.yaw_nm)


def finite(*values):
    """拒绝非有限数值输入。

    Args:
        values: 需要检查有限性的标量数值。
    """
    if not all(math.isfinite(v) for v in values):
        raise ValueError("Inputs must be finite")


def steering_target(command, limit_deg=35.0):
    """将归一化转向请求转换为 FLU 坐标下的推力方位角。

    Normalized right-turn command in forward operation -> FLU thrust azimuth.

    Positive port-directed stern force creates starboard bow yaw.
    Reverse-operation steering semantics must be handled by the vehicle controller.

    Args:
        command: 归一化转向请求；正向含义沿用当前车辆控制约定。
        limit_deg: 转向角限幅，单位度，当前不得超过 35。
    """
    finite(command, limit_deg)
    if not 0 < limit_deg <= 35:
        raise ValueError("This concept model supports a limit in (0, 35] deg")
    return max(-1.0, min(1.0, command)) * limit_deg


def slew_angle(current_deg, target_deg, dt_s, max_rate_deg_s, limit_deg=35.0):
    """按时间步长和速率限制更新转向角。

    Rate is supplied by caller after actuator selection. No guessed default.

    Args:
        current_deg: 当前角度，单位度。
        target_deg: 目标角度，单位度。
        dt_s: 更新步长，单位 s。
        max_rate_deg_s: 最大角速度，单位度/s。
        limit_deg: 角度绝对限幅，单位度。
    """
    finite(current_deg, target_deg, dt_s, max_rate_deg_s, limit_deg)
    if dt_s < 0 or max_rate_deg_s <= 0 or not 0 < limit_deg <= 35:
        raise ValueError("Invalid rate, time or angle limit")
    if abs(current_deg) > limit_deg:
        raise ValueError("Current angle is outside the model range")
    target = max(-limit_deg, min(limit_deg, target_deg))
    step = max_rate_deg_s * dt_s
    return current_deg + max(-step, min(step, target - current_deg))


def wrench(thrust_n, azimuth_deg, cg_xy_m, axis_xy_m=(-3.624, 0.0)):
    """将调用方给定的推力转为水平力及关于重心的力矩。

    Return body-frame horizontal force and moment about the supplied CG.

    The thrust line passes through the steering axis in plan view.
    CG must be supplied; original twin-motor mass budget is not reused.

    Args:
        thrust_n: 有符号推力，单位 N；不从油门或功率猜测推力。
        azimuth_deg: 推力方位角，单位度。
        cg_xy_m: 给定重心的平面坐标，单位 m。
        axis_xy_m: 转向轴的平面坐标，单位 m。
    """
    finite(thrust_n, azimuth_deg, *cg_xy_m, *axis_xy_m)
    if abs(azimuth_deg) > 35:
        raise ValueError("Azimuth outside verified concept range")
    a = math.radians(azimuth_deg)
    fx = thrust_n * math.cos(a)
    fy = thrust_n * math.sin(a)
    rx = axis_xy_m[0] - cg_xy_m[0]
    ry = axis_xy_m[1] - cg_xy_m[1]
    return Wrench(fx, fy, rx * fy - ry * fx)
