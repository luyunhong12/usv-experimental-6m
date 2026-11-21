/** @file test_resistance.cc
 * @brief 当前模块的解析回归验证。
 */
#include "Resistance.hh"
#include <cassert>
using namespace usv;

/** @brief 用均匀船长解析积分验证横移预算与自然产生的偏航阻尼。 */
void CheckDistributedIntegrals() {
  std::vector<LateralPatch> patches;
  const int count = 2000;
  for (int i = 0; i < count; ++i)
    patches.push_back({-3 + (i + .5) * 6 / count, 1.2 / count, 10.3 / count});
  const auto sway =
      DistributedLateralDrag(patches, 2, 0, 600 / 10.3, 1500 / 1.2);
  assert(std::abs(sway.force + 7200) < 1e-8);
  assert(std::abs(sway.moment) < 1e-8);
  const auto yaw =
      DistributedLateralDrag(patches, 0, .1, 600 / 10.3, 1500 / 1.2);
  assert(std::abs(yaw.force) < 1e-8);
  // Integral x^2 / L = L^2/12; integral |x|^3 / L = L^3/32.
  const double expected = -600 * 36 / 12 * .1 - 1500 * 216 / 32 * .01;
  assert(std::abs(yaw.moment - expected) < .001);
}
/** @brief 验证反向、耦合运动和干面片均不产生流体动能。 */
void CheckDistributedPassivity() {
  const std::vector<LateralPatch> patches = {
      {-2, .3, 2}, {-.5, .4, 3}, {1.5, .2, 1}};
  for (double sway : {-2., 0., 2.})
    for (double rate : {-.5, 0., .5}) {
      const auto force = DistributedLateralDrag(patches, sway, rate, 500, 1250);
      assert(force.force * sway + force.moment * rate <= 1e-9);
      const auto opposite =
          DistributedLateralDrag(patches, -sway, -rate, 500, 1250);
      assert(std::abs(force.force + opposite.force) < 1e-8);
      assert(std::abs(force.moment + opposite.moment) < 1e-8);
    }
  const auto dry = DistributedLateralDrag({}, 2, .1, 500, 1250);
  assert(dry.force == 0 && dry.moment == 0);
}

/** @brief 执行该文件定义的解析或场景回归验证。
 */
int main() {
  CheckDistributedIntegrals();
  CheckDistributedPassivity();
  V l(120, 600, 1400), q(180, 1500, 1800), ref(.36, 1.24, 4.49);
  auto nominal = GeometryResistanceScales(10.3, ref, 10.3, ref);
  for (double x : {-4., -1., 0., .1, 4.})
    for (double y : {-2., 0., 2.}) {
      V v(x, y, .5);
      auto f = GeometryDrag(l, q, v, nominal);
      for (int i = 0; i < 3; ++i)
        assert(std::abs(f[i] + l[i] * v[i] + q[i] * v[i] * std::abs(v[i])) <
               1e-8);
      assert(f.Dot(v) <= 0);
      auto wet = GeometryResistanceScales(20.6, ref * 2, 10.3, ref);
      assert((GeometryDrag(l, q, v, wet) - f * 2).Length() < 1e-8);
      assert(
          GeometryDrag(l, q, v, GeometryResistanceScales(0, V::Zero, 10.3, ref))
              .Length() == 0);
    }
  bool rejected = false;
  try {
    GeometryResistanceScales(1, ref, 0, ref);
  } catch (...) {
    rejected = true;
  }
  assert(rejected);
}
