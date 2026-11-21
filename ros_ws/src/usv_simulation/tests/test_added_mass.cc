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

/** @brief 执行该文件定义的解析或场景回归验证。
 */
int main(){
 V m(70,250,250),j(30,200,200),v(2,.3,-.1),w(.1,-.2,.3);auto c=AddedMassWrench(m,j,v,w,V::Zero,V::Zero);check(std::abs(v.Dot(c.force)+w.Dot(c.torque))<1e-10);
 auto a=AddedMassWrench(m,j,V::Zero,V::Zero,V(1,2,3),V(4,5,6));check((a.force-V(-70,-500,-750)).Length()<1e-10);check((a.torque-V(-120,-1000,-1200)).Length()<1e-10);
 // Exact constant-force response after the explicit one-step transient.
 for(double dt:{.004,.002}){double u=0,acc=0;for(int k=0;k<int(4/dt);++k){acc=(1000-250*acc)/1500;u+=acc*dt;}check(std::abs(u-4000./1750)<.001);}
 std::cout<<"Added inertia signs, zero-speed acceleration, skew power and timestep convergence passed\n";
}
