"""按明确的工程约定分配原轴向阻力预算，不识别或拟合水动力系数。

Explicit allocation of legacy axial drag; not hydrodynamic identification.
"""

import math


def allocate_resistance(config, appendages):
    """按完整浸水零攻角基准分配原轴向阻力预算，不辨识新系数。

    Args:
        config: 包含原集中阻力预算的配置
        appendages: 包含轮廓、密度和型阻系数的附体配置
    """
    legacy = config["legacy_quadratic_drag_N_s2_m2"]
    angular = config["hull_quadratic_rotational_drag_Nm_s2_rad2"]
    if (
        len(legacy) != 3
        or len(angular) != 3
        or not all(math.isfinite(v) and v >= 0 for v in legacy + angular)
    ):
        raise ValueError("Invalid legacy hull damping")
    rho = appendages["density_kg_m3"]
    if not math.isfinite(rho) or rho <= 0:
        raise ValueError("Invalid appendage density")
    terms = {}
    for surface in appendages["surfaces"]:
        # Recompute polygon area rather than trusting stale cached area_m2.
        points = surface["outline_xz_m"]
        if len(points) < 3 or not all(math.isfinite(v) for p in points for v in p):
            raise ValueError("Invalid appendage polygon")
        area = (
            abs(
                sum(
                    p[0] * q[1] - q[0] * p[1]
                    for p, q in zip(points, points[1:] + points[:1])
                )
            )
            / 2
        )
        cd = surface["profile_drag_coefficient"]
        if not math.isfinite(cd) or cd < 0 or area <= 0:
            raise ValueError("Invalid appendage profile drag")
        terms[surface["name"]] = 0.5 * rho * area * cd
    allocated = sum(terms.values())
    if allocated > legacy[0]:
        raise ValueError("Appendage allocation exceeds legacy axial budget")
    hull = [legacy[0] - allocated, *legacy[1:]]
    return {
        "legacy_quadratic_drag_N_s2_m2": legacy,
        "hull_quadratic_drag_N_s2_m2": hull,
        "appendage_reference_axial_N_s2_m2": terms,
        "allocated_axial_N_s2_m2": allocated,
        "hull_quadratic_rotational_drag_Nm_s2_rad2": angular,
        "scope": "Full-wet zero-incidence reference bookkeeping; local runtime appendage lift/drag remains independent. Lateral/angular overlap remains unidentified.",
    }
