/** @file SteeringIntegral.hh
 * @brief 同时考虑转矩和能源饱和的积分抗饱和项。
 */
#pragma once
#include <algorithm>
#include <cmath>
namespace usv {
// Integral torque with anti-windup against BOTH effort and energy saturation.
/** @brief 保存转向积分转矩，单位 N·m。
 */
struct SteeringIntegral {
  double torque = 0;
  /** @brief 根据角度和角速度跟踪误差求执行器请求力矩。
   * @param positionError 关节参考角度减实际角度，rad。
   * @param velocityError 关节参考速度减实际速度，rad/s。
   * @param kp 角度比例增益，Nm/rad。
   * @param kd 角速度反馈增益，Nm/(rad/s)。
   */
  double Request(double positionError, double velocityError, double kp,
                 double kd) const {
    return kp * positionError + kd * velocityError + torque;
  }

  /** @brief 在未饱和或误差帮助退出饱和时累积积分转矩。
   * @param error 转向角误差，rad。
   * @param dt 时间步长，s。
   * @param ki 积分转矩增益。
   * @param limit 积分转矩绝对上限，N·m。
   * @param requested 所请求转矩，N·m。
   * @param applied 实际施加转矩，N·m。
   */
  void Update(double error, double dt, double ki, double limit,
              double requested, double applied) {
    if (std::abs(requested - applied) < 1e-7 || error * requested < 0)
      torque = std::clamp(torque + ki * error * dt, -limit, limit);
  }
};
} // namespace usv
