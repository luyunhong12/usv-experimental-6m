"""低速导航验收的必需检查集合；未执行阶段不得默认为通过。"""


def required_checks(failsafe_only=False, response_only=False):
    """返回全部初始化为失败的检查项。

    Args:
        failsafe_only: 是否仅验收独立 GCS 心跳失联。
        response_only: 是否仅检查开环响应辨识的数据完整性。
    """
    if response_only:
        return dict.fromkeys(
            [
                "low_speed_parameters",
                "normal_arming",
                "navigation_ready_before_arming",
                "completed",
                "finite",
                "estimator_healthy_after_arming",
                "response_manual",
                "response_excitation",
                "response_coverage",
            ],
            False,
        )
    names = [
        "low_speed_parameters",
        "normal_arming",
        "navigation_ready_before_arming",
        "guided_mode",
        "moving_before_loss",
        "gcs_failsafe_hold",
        "gcs_failsafe_stops_command",
        "failsafe_status_reported",
        "independent_rc_link_healthy",
        "completed",
        "finite",
        "estimator_healthy_after_arming",
    ]
    if not failsafe_only:
        names += [
            "mission_upload",
            "auto_waypoints",
            "auto_path_corridor",
            "loiter_mode",
            "loiter_radius",
            "loiter_target_observed",
            "loiter_target_radius",
            "rtl_mode",
            "rtl_home",
            "hold_stops_command",
            "hold_slows_vessel",
            "manual_takeover",
            "low_speed_tracking",
        ]
    return dict.fromkeys(names, False)


def passed_checks(checks, failsafe_only=False, response_only=False):
    """仅当全部必需项存在且为布尔真时判为通过。

    Args:
        checks: 实际检查结果。
        failsafe_only: 是否使用仅失联试验的必需集合。
        response_only: 是否使用响应辨识的独立必需集合。
    """
    return all(
        checks.get(name) is True
        for name in required_checks(failsafe_only, response_only)
    )


def sustained_stop(samples, minimum_duration=5.0, maximum_gap=0.5):
    """验证连续停推窗口，拒绝单点、缺测和非 HOLD 样本。

    Args:
        samples: (仿真秒、推进请求、模式) 样本序列。
        minimum_duration: 至少持续的仿真秒数。
        maximum_gap: 允许的最大相邻采样间隔，秒。
    """
    import math

    return bool(
        len(samples) >= 2
        and samples[-1][0] - samples[0][0] >= minimum_duration
        and all(
            math.isfinite(t) and math.isfinite(cmd) and abs(cmd) < 0.01 and mode == 4
            for t, cmd, mode in samples
        )
        and all(0 < b[0] - a[0] <= maximum_gap for a, b in zip(samples, samples[1:]))
    )


def observed_loiter_target(samples):
    """验证稳定保持段的目标观测覆盖，拒绝缺测或目标移动。

    Args:
        samples: [阶段秒、GPS到目标距离、GPS纬经度、目标纬经度]。
    """
    import math

    if not all(len(row) == 6 and all(math.isfinite(v) for v in row) for row in samples):
        return False
    stable = [row for row in samples if row[0] > 15]
    if len(stable) < 2:
        return False
    lat, lon = stable[0][4:6]
    return bool(
        stable[-1][0] - stable[0][0] >= 100
        and all(0 <= b[0] - a[0] <= 1 for a, b in zip(stable, stable[1:]))
        and all(
            math.hypot(
                (r[4] - lat) * 111319.49,
                (r[5] - lon) * 111319.49 * math.cos(math.radians(lat)),
            )
            < 0.25
            for r in stable
        )
    )


def sustained_motion(samples):
    """验证GUIDED断链前持续运动，拒绝转向停顿、缺测或仅有油门指令。

    Args:
        samples: [仿真秒、实测纵向速度、推进请求、模式] 连续窗口。
    """
    import math

    return bool(
        len(samples) >= 2
        and samples[-1][0] - samples[0][0] >= 2.0
        and all(
            math.isfinite(t)
            and math.isfinite(speed)
            and math.isfinite(command)
            and abs(speed) > 0.3
            and abs(command) > 0.05
            and mode == 15
            for t, speed, command, mode in samples
        )
        and all(0 < b[0] - a[0] <= 0.5 for a, b in zip(samples, samples[1:]))
    )


# 时长为仿真秒，输入为原生RC通道1/2/3的PWM；1563/1437在已核对的30 us死区后约为正反8.9%驾驶员油门。
RESPONSE_PLAN = (
    ("settle", 20, (1500, 1500, 1500)),
    ("forward", 40, (1500, 1500, 1563)),
    ("left_input", 30, (1100, 1500, 1563)),
    ("right_input", 30, (1900, 1500, 1563)),
    ("center", 30, (1500, 1500, 1563)),
    ("reverse", 40, (1500, 1500, 1437)),
    ("reverse_left_input", 30, (1100, 1500, 1437)),
    ("reverse_right_input", 30, (1900, 1500, 1437)),
    ("stop", 40, (1500, 1500, 1500)),
)


def response_command(elapsed):
    """根据仿真时间返回辨识阶段与PWM；非法时间拒绝，结束后回中。

    Args:
        elapsed: 已运行的仿真秒数。
    """
    import math

    if not math.isfinite(elapsed) or elapsed < 0:
        raise ValueError("Response elapsed time must be finite and non-negative")
    end = 0
    for index, (_, duration, channels) in enumerate(RESPONSE_PLAN):
        end += duration
        if elapsed < end:
            return index, channels
    return len(RESPONSE_PLAN), (1500, 1500, 1500)


def response_coverage(samples):
    """检查响应辨识覆盖完整时长、全部阶段且没有长时间缺测。

    Args:
        samples: 每行以仿真秒、阶段累计秒和阶段序号开头的有限数据。
    """
    import math

    duration = sum(stage[1] for stage in RESPONSE_PLAN)
    return bool(
        len(samples) > 1
        and samples[0][1] < 1
        and samples[-1][1] > duration - 1
        and {int(row[2]) for row in samples} == set(range(len(RESPONSE_PLAN)))
        and all(all(math.isfinite(v) for v in row) for row in samples)
        and all(0 < b[0] - a[0] <= 0.5 for a, b in zip(samples, samples[1:]))
    )
