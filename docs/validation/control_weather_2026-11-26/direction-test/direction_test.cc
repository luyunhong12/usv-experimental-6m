/** @file direction_test.cc
 * @brief 编译实际实验补丁中的方向判定，验证边界、记忆和默认回退。
 */
#include "Rover/LoiterDirection.h"
#include <cassert>
#include <iostream>
#include <limits>
#include <utility>

/** @brief 执行独立于实现公式的角度状态序列；任何偏差立即断言失败。 */
int main()
{
    const std::pair<float, bool> samples[] = {
        {80, false}, {92, false}, {88, false}, {96, false}, {109, false},
        {110, false}, {111, true}, {100, true}, {80, true}, {71, true}, {70, false}};
    for (float sign : {-1.0f, 1.0f}) {
        bool reverse = false;
        for (const auto &sample : samples) {
            reverse = loiter_direction_reverse(sign * sample.first * 100, reverse, 20);
            assert(reverse == sample.second);
        }
    }
    for (bool previous : {false, true}) {
        assert(!loiter_direction_reverse(9000, previous, 0));
        assert(loiter_direction_reverse(9001, previous, 0));
        assert(loiter_direction_reverse(-9001, previous, -1));
        assert(!loiter_direction_reverse(8999, previous, std::numeric_limits<float>::quiet_NaN()));
        assert(loiter_direction_reverse(9001, previous, std::numeric_limits<float>::infinity()));
    }
    assert(!loiter_direction_reverse(12000, false, 1000));
    assert(loiter_direction_reverse(12001, false, 1000));
    assert(!loiter_direction_reverse(6000, true, 1000));
    assert(loiter_direction_reverse(6001, true, 1000));
    std::cout << "Direction boundary, signed symmetry, memory, disabled/invalid fallback and clamp checks passed\n";
}
