/** @file test_steering_integral.cc
 * @brief 当前模块的解析回归验证。
 */
#include "SteeringIntegral.hh"
#include <cassert>

/** @brief 验证比例、速度反馈和积分转矩按原公式叠加。 */
void CheckRequestTerms() {
  usv::SteeringIntegral controller;
  controller.torque = .4;
  assert(std::abs(controller.Request(.3, .7, 2, 3) - 3.1) < 1e-12);
}

/** @brief 验证饱和边界与零请求时不会错误累积积分。 */
void CheckSaturationBoundaries() {
  usv::SteeringIntegral controller;
  controller.Update(1, .1, 2, 100, 1e-7, 0);
  assert(controller.torque == 0); // strict tolerance boundary
  controller.Update(.5, .1, 2, 100, 0, 1);
  assert(controller.torque == 0); // zero request is not unwinding
  controller.Update(-.5, .1, 2, 100, 0, 1);
  assert(controller.torque == 0); // multiplication by zero, not division
  controller.Update(1, .1, 2, 100, 0, 0);
  assert(std::abs(controller.torque - .2) < 1e-12);
}

/** @brief 执行该文件定义的解析或场景回归验证。
 */
int main() {
  CheckRequestTerms();
  CheckSaturationBoundaries();
  // A moving reference must not produce damping against perfectly tracked
  // motion.
  usv::SteeringIntegral moving;
  assert(std::abs(moving.Request(0, .15 - .15, 300, 80)) < 1e-12);
  assert(std::abs(moving.Request(0, 0 - .15, 300, 80) + 12) < 1e-12);
  assert(moving.Request(.1, .15, 300, 80) > 0);
  usv::SteeringIntegral s;
  double x = 0, v = 0, dt = .001;
  for (int i = 0; i < 60000; ++i) {
    double e = -x, raw = 300 * e - 80 * v + s.torque,
           applied = std::clamp(raw, -100., 100.);
    s.Update(e, dt, 100, 100, raw, applied);
    v += (applied + 10) * dt;
    x += v * dt;
  }
  assert(std::abs(x) < 1e-6);
  assert(std::abs(s.torque + 10) < 1e-4);
  double saved = s.torque;
  s.Update(1, .1, 100, 100, 200, 100);
  assert(s.torque == saved);
  s.Update(1, .1, 100, 100, 50, 0);
  assert(s.torque == saved); // power budget saturation
  s.Update(-1, .1, 100, 100, 200, 100);
  assert(s.torque < saved); // unwind
}
