/** @file direction_sync_test.cc
 * @brief 按方向切换需求检查补丁实际头文件，包括制动和无效指令。
 */
#include "Rover/LoiterDirectionSync.h"
#include <cassert>
#include <limits>
#include <iostream>

/** @brief 覆盖首次选择、双向零区、制动保持、非法输入和进入重置。 */
int main()
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    for (const bool reverse : {false, true}) {
        assert(loiter_synced_heading_reverse(reverse, !reverse, false, nan) == reverse);
        for (const float throttle : {-30.0f, 0.0f, 30.0f, nan, inf}) {
            // 不改变行驶方向请求时，反向制动不得翻转航向框架。
            assert(loiter_synced_heading_reverse(reverse, reverse, true, throttle) == reverse);
        }
        for (const float throttle : {-1.0f, 0.0f, 1.0f, nan, inf, -inf}) {
            assert(loiter_synced_heading_reverse(reverse, !reverse, true, throttle) == !reverse);
        }
        const float accepted = reverse ? -1.01f : 1.01f;
        assert(loiter_synced_heading_reverse(reverse, !reverse, true, accepted) == reverse);
        assert(loiter_synced_heading_reverse(reverse, !reverse, true, -accepted) == !reverse);
    }
    // 前进→倒车：请求先变，推进未变期间保持；变号后接受，随后制动也不改框架。
    bool heading = false;
    for (const float command : {15.0f, 5.0f, 0.0f, -1.0f}) {
        heading = loiter_synced_heading_reverse(true, heading, true, command);
        assert(!heading);
    }
    heading = loiter_synced_heading_reverse(true, heading, true, -5.0f);
    assert(heading);
    assert(loiter_synced_heading_reverse(true, heading, true, 5.0f));
    assert(!loiter_synced_heading_reverse(false, heading, false, -5.0f));
    std::cout << "direction sync decision boundaries: PASS\n";
}
