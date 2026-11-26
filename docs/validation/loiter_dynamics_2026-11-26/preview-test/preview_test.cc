/** @file preview_test.cc
 * @brief 用已知角速度积分、异常输入和边界验证实际补丁头文件。
 */
#include "Rover/LoiterYawPreview.h"
#include <cassert>
#include <limits>
#include <iostream>

/** @brief 正右转10度每秒、4秒预见应提前40度；左右对称且异常关闭。 */
int main()
{
    constexpr float pi = 3.14159265358979323846f;
    const float ten_degrees_s = pi / 18;
    assert(std::fabs(loiter_yaw_preview_rad(ten_degrees_s, 4) - 2*pi/9) < 1e-6f);
    assert(std::fabs(loiter_yaw_preview_rad(-ten_degrees_s, 4) + 2*pi/9) < 1e-6f);
    assert(loiter_yaw_preview_rad(0, 4) == 0);
    for (const float horizon : {0.0f, -1.0f, std::numeric_limits<float>::infinity(),
                               std::numeric_limits<float>::quiet_NaN()}) {
        assert(loiter_yaw_preview_rad(ten_degrees_s, horizon) == 0);
    }
    for (const float rate : {std::numeric_limits<float>::infinity(),
                            -std::numeric_limits<float>::infinity(),
                            std::numeric_limits<float>::quiet_NaN()}) {
        assert(loiter_yaw_preview_rad(rate, 4) == 0);
    }
    assert(std::fabs(loiter_yaw_preview_rad(ten_degrees_s, 100) - 5*pi/18) < 1e-6f);
    for (const float rate : {1.0f, -1.0f, std::numeric_limits<float>::max(),
                            -std::numeric_limits<float>::max()}) {
        const float value = loiter_yaw_preview_rad(rate, 5);
        assert(std::isfinite(value));
        assert(std::fabs(value) <= pi/2);
        assert((value > 0) == (rate > 0));
    }
    const float tiny = loiter_yaw_preview_rad(1, std::numeric_limits<float>::denorm_min());
    assert(std::isfinite(tiny) && tiny >= 0);
    std::cout << "yaw preview integration, bounds and invalid-input behavior: PASS\n";
}
