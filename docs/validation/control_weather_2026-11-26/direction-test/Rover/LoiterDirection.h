/** @file LoiterDirection.h
 * @brief 动力船LOITER的独立角度迟滞判定；不计算或施加物理力。
 */
#pragma once
#include <cmath>

/** @brief 根据目标相对方位和上一方向选择正反向；不修改调用方状态。
 * @param yaw_error_cd 未按倒车折算的相对方位，单位厘度，通常在[-18000,18000]。
 * @param was_reverse 上一周期是否选择倒车。
 * @param margin_deg 迟滞半宽，单位度；非有限或非正值使用原90度规则，最大钳制到30。
 * @return true选择倒车；调用方保存返回值。此函数不是定时器，不保证最小驻留时间。
 */
inline bool loiter_direction_reverse(float yaw_error_cd, bool was_reverse, float margin_deg)
{
    if (!std::isfinite(margin_deg) || margin_deg <= 0.0f) {
        return std::fabs(yaw_error_cd) > 9000.0f;
    }
    const float margin_cd = std::fmin(margin_deg, 30.0f) * 100.0f;
    const float boundary_cd = 9000.0f + (was_reverse ? -margin_cd : margin_cd);
    return std::fabs(yaw_error_cd) > boundary_cd;
}
