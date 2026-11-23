/** @file test_appendages.cc
 * @brief 当前模块的解析回归验证。
 */
#include "AppendageHydrodynamics.hh"
#include <iostream>
#include <sstream>
#include <stdexcept>
using namespace usv;

/** @brief 校验测试判据，失败时终止本项验证。
 * @param x 应为真的测试判据。
 */
void check(bool x) {
  if (!x)
    throw std::runtime_error("Appendage invariant failed");
}

/** @brief 执行该文件定义的解析或场景回归验证。
 */
int main() {
  check(AppendageTopicSuffix("").empty());
  check(AppendageTopicSuffix("stabilizer") == "/stabilizer");
  bool invalidGroup = false;
  try {
    AppendageTopicSuffix("bad/group");
  } catch (const std::runtime_error &) {
    invalidGroup = true;
  }
  check(invalidGroup);
  std::istringstream valid("0 0 0 1 0 0 0 1 0  ");
  auto mesh = ReadAppendageBuoyancyMesh(valid);
  check(mesh.size() == 1 && mesh[0][2] == V(0, 1, 0));
  for (const char *text :
       {"", "0 0 0 1 0", "0 0 0 1 0 0 0 1 0 trailing", "nan 0 0 1 0 0 0 1 0"}) {
    std::istringstream input(text);
    bool rejected = false;
    try {
      ReadAppendageBuoyancyMesh(input);
    } catch (const std::runtime_error &) {
      rejected = true;
    }
    check(rejected);
  }

  std::vector<V> p{{0, 0, -1}, {1, 0, -1}, {1, 0, 1}, {0, 0, 1}};
  auto wet = ClipAppendage(p, {1, 1, -1, -1});
  check(std::abs(wet.area - 1) < 1e-12);
  check(std::abs(wet.center.Z() + .5) < 1e-12);
  check(std::abs(wet.span - 1) < 1e-12);
  check(ClipAppendage(p, {-1, -1, -2, -2}).area == 0);
  for (double u : {-4., -1., 0., 1., 4.})
    for (double v : {-3., -.1, 0., .1, 3.}) {
      V vel(u, v, 0), f = AppendageForce(1000, .04, .4, .03, .7, vel);
      check(f.Dot(vel) <= 1e-9);
      auto mirror = AppendageForce(1000, .04, .4, .03, .7, V(u, -v, 0));
      check(std::abs(f.X() - mirror.X()) < 1e-9 &&
            std::abs(f.Y() + mirror.Y()) < 1e-9);
      check(f.Y() * v <= 1e-9);
      auto reverse = AppendageForce(1000, .04, .4, .03, .7, -vel);
      check((f + reverse).Length() < 1e-9);
    }
  check(AppendageForce(1000, 0, .4, .03, .7, V(4, .1, 0)).Length() == 0);
  // A stern surface provides a restoring yaw moment against a lateral velocity;
  // no heading command or attitude reference enters the force law.
  V force = AppendageForce(1000, .04, .4, .03, .7, V(4, .1, 0));
  check(V(-3, 0, 0).Cross(force).Z() > 0);
  std::cout << "Wet clipping, center, force passivity, mirror/reverse symmetry "
               "and restoring moment passed\n";
}
