/** @file test_added_mass.cc
 * @brief 当前模块的解析回归验证。
 */
#include "AddedMass.hh"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace usv;

/** @brief 校验测试判据，失败时终止本项验证。
 * @param b 应为真的测试判据。
 */
void check(bool b){if(!b)throw std::runtime_error("Added-mass invariant failed");}

/** @brief 用平面势流解析值验证前进/倒车的 Munk 力矩和侧向惯性力。
 * @note MIT 2.154 第8讲的 Munk 力矩来自方向不同的流体惯性，
 *       不是阻尼；不能为了恢复测试通过而删除。坐标为船体 FLU。
 */
void CheckPlanarCoupling() {
  const V mass(70, 250, 250), inertia(30, 200, 200);
  // u=±2 m/s、v=0.1 m/s：N=-(250-70)uv=∓36 N·m。
  // r=0.02 rad/s：Y=-70ur=∓2.8 N；X=250vr=0.5 N。
  const auto forward = AddedMassWrench(mass, inertia, V(2, .1, 0),
                                      V(0, 0, .02), V::Zero, V::Zero);
  const auto reverse = AddedMassWrench(mass, inertia, V(-2, .1, 0),
                                      V(0, 0, .02), V::Zero, V::Zero);
  check((forward.force - V(.5, -2.8, 0)).Length() < 1e-12);
  check((reverse.force - V(.5, 2.8, 0)).Length() < 1e-12);
  check(std::abs(forward.torque.Z() + 36) < 1e-12);
  check(std::abs(reverse.torque.Z() - 36) < 1e-12);
  // 球形等向附加质量的 Munk 力矩为零，即使存在斜流。
  const auto isotropic = AddedMassWrench(V(250, 250, 250), inertia,
                                        V(-2, .1, 0), V::Zero,
                                        V::Zero, V::Zero);
  check(isotropic.torque.Length() < 1e-12);
}

/** @brief 验证偏置流体惯性的一阶矩、互易性及无耗散耦合。
 * @note 两个100 kg横向流体质点位于x=0和2 m，独立给出
 *       A22=200 kg、A26=200 kg·m、A66=400 kg·m²。
 */
void CheckOffsetFluidInertia() {
  const V mass(70, 200, 250), inertia(30, 200, 400);
  const auto sway = AddedMassWrench(mass, inertia, V::Zero, V::Zero,
                                    V(0, 1, 0), V::Zero, 200);
  const auto yaw = AddedMassWrench(mass, inertia, V::Zero, V::Zero,
                                   V::Zero, V(0, 0, 1), 200);
  check(sway.force.Y() == -200 && sway.torque.Z() == -200);
  check(yaw.force.Y() == -200 && yaw.torque.Z() == -400);
  const V velocity(-2, .3, -.1), rate(.1, -.2, .3);
  const auto force = AddedMassWrench(mass, inertia, velocity, rate,
                                     V::Zero, V::Zero, 200);
  check(std::abs(velocity.Dot(force.force) + rate.Dot(force.torque)) < 1e-10);
}

/** @brief 执行该文件定义的解析或场景回归验证。
 */
int main(){
 CheckPlanarCoupling();
 CheckOffsetFluidInertia();
 V m(70,250,250),j(30,200,200),v(2,.3,-.1),w(.1,-.2,.3);auto c=AddedMassWrench(m,j,v,w,V::Zero,V::Zero);check(std::abs(v.Dot(c.force)+w.Dot(c.torque))<1e-10);
 auto a=AddedMassWrench(m,j,V::Zero,V::Zero,V(1,2,3),V(4,5,6));check((a.force-V(-70,-500,-750)).Length()<1e-10);check((a.torque-V(-120,-1000,-1200)).Length()<1e-10);
 // Exact constant-force response after the explicit one-step transient.
 for(double dt:{.004,.002}){double u=0,acc=0;for(int k=0;k<int(4/dt);++k){acc=(1000-250*acc)/1500;u+=acc*dt;}check(std::abs(u-4000./1750)<.001);}
 std::cout<<"Added inertia signs, zero-speed acceleration, skew power and timestep convergence passed\n";
}
