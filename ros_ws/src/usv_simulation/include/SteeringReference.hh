/** @file SteeringReference.hh
 * @brief 带角度、角速度和角加速度约束的转向参考轨迹。
 */
#pragma once
#include <algorithm>
#include <cmath>
namespace usv {
/** @brief 保存参考角度 rad 与角速度 rad/s。
 */
struct SteeringReference {
 double position=0,velocity=0;
/** @brief 按离散制动距离更新转向参考状态。
 * @param target 目标角度，rad。
 * @param dt 步长，s；非正步长不更新。
 * @param rate 最大角速度，rad/s。
 * @param acceleration 最大角加速度，rad/s²。
 * @param limit 角度绝对限幅，rad。
 */
 void Step(double target,double dt,double rate,double acceleration,double limit) {
  if(dt<=0)return;
  target=std::clamp(target,-limit,limit);
  double error=target-position;
  // Discrete stopping bound: reserve one step of travel before braking.
  double stop=std::max(0.,std::sqrt(acceleration*acceleration*dt*dt+2*acceleration*std::abs(error))-acceleration*dt);
  double desired=std::copysign(std::min(rate,stop),error);
  velocity+=std::clamp(desired-velocity,-acceleration*dt,acceleration*dt);
  position+=velocity*dt;
  if(position>limit){position=limit;velocity=0;}
  if(position< -limit){position=-limit;velocity=0;}
 }
};
}
