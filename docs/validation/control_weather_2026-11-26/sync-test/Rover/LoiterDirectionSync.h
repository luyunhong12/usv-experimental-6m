/** @file LoiterDirectionSync.h
 * @brief 桨舵一体LOITER的实验指令同步判定，不施加或改变船体受力。
 */
#pragma once
#include <cmath>

/** @brief 等待推进指令同向后接受新的航向框架，首次决策直接采用几何方向。
 * @param requested_reverse 本周期按目标相对方位选择的行驶方向。
 * @param heading_reverse 上一周期已接受的航向框架是否倒车。
 * @param initialized 当前LOITER是否已有几何决策；重新进入时必须置false。
 * @param throttle_pct 上一周期推进指令，百分比[-100,100]，不是实际推力或真值。
 * @return 航向框架是否倒车；制动指令不改变未变更的方向请求。
 * @note 1%试验死区对应当前MOT_THR_MIN=1；并非通用标定值。零或非有限指令等待。
 */
inline bool loiter_synced_heading_reverse(bool requested_reverse, bool heading_reverse,
                                         bool initialized, float throttle_pct)
{
    if (!initialized || requested_reverse == heading_reverse) {
        return requested_reverse;
    }
    if (!std::isfinite(throttle_pct)) {
        return heading_reverse;
    }
    const bool command_agrees = requested_reverse ? throttle_pct < -1.0f : throttle_pct > 1.0f;
    return command_agrees ? requested_reverse : heading_reverse;
}
