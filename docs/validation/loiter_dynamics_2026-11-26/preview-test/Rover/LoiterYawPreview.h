/** @file LoiterYawPreview.h
 * @brief 根据实测偏航率生成有界航向预见量，不计算或改变物理力。
 */
#pragma once
#include <cmath>

/** @brief 计算应从本周期几何航向减去的预见量，避免舵机迟滞下过晚收舵。
 * @param yaw_rate_rad_s EKF地球系偏航角速度，rad/s，NED向右为正。
 * @param horizon_s 工程预见时域，秒；非正/非有限为关闭，最大5秒。
 * @return 有符号角度修正rad，绝对值不超过pi/2；非法角速度返回0。
 * @note 90度上限是异常/极端状态的控制约束，不是船体动力学认证。
 */
inline float loiter_yaw_preview_rad(float yaw_rate_rad_s, float horizon_s)
{
    if (!std::isfinite(yaw_rate_rad_s) || !std::isfinite(horizon_s) || horizon_s <= 0) {
        return 0.0f;
    }
    constexpr float half_pi = 1.5707963267948966f;
    const float horizon = std::fmin(horizon_s, 5.0f);
    const float rate_bound = half_pi / horizon;
    const float bounded_rate = std::fmax(-rate_bound, std::fmin(yaw_rate_rad_s, rate_bound));
    return bounded_rate * horizon;
}
