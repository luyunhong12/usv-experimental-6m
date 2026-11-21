"""Single azimuth thruster force interface; NOT a hull dynamics simulator.
Coordinates: model FLU (forward, left, up); ArduPilot uses FRD (forward, right, down).
Thrust is calibrated newtons supplied by caller, never guessed from throttle or kW.
"""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class Wrench:
    fx_n: float
    fy_n: float
    yaw_nm: float

    def as_frd(self):
        return Wrench(self.fx_n, -self.fy_n, -self.yaw_nm)


def finite(*values):
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Inputs must be finite')


def steering_target(command, limit_deg=35.0):
    """Normalized right-turn command in forward operation -> FLU thrust azimuth.
    Positive port-directed stern force creates starboard bow yaw.
    Reverse-operation steering semantics must be handled by the vehicle controller.
    """
    finite(command, limit_deg)
    if not 0 < limit_deg <= 35:
        raise ValueError('This concept model supports a limit in (0, 35] deg')
    return max(-1.0, min(1.0, command))*limit_deg


def slew_angle(current_deg, target_deg, dt_s, max_rate_deg_s, limit_deg=35.0):
    """Rate is supplied by caller after actuator selection. No guessed default."""
    finite(current_deg,target_deg,dt_s,max_rate_deg_s,limit_deg)
    if dt_s < 0 or max_rate_deg_s <= 0 or not 0 < limit_deg <= 35:
        raise ValueError('Invalid rate, time or angle limit')
    if abs(current_deg)>limit_deg:
        raise ValueError('Current angle is outside the model range')
    target=max(-limit_deg,min(limit_deg,target_deg));step=max_rate_deg_s*dt_s
    return current_deg+max(-step,min(step,target-current_deg))


def wrench(thrust_n, azimuth_deg, cg_xy_m, axis_xy_m=(-3.624,0.0)):
    """Return body-frame horizontal force and moment about the supplied CG.
    The thrust line passes through the steering axis in plan view.
    CG must be supplied; original twin-motor mass budget is not reused.
    """
    finite(thrust_n,azimuth_deg,*cg_xy_m,*axis_xy_m)
    if abs(azimuth_deg)>35:
        raise ValueError('Azimuth outside verified concept range')
    a=math.radians(azimuth_deg);fx=thrust_n*math.cos(a);fy=thrust_n*math.sin(a)
    rx=axis_xy_m[0]-cg_xy_m[0];ry=axis_xy_m[1]-cg_xy_m[1]
    return Wrench(fx,fy,rx*fy-ry*fx)
