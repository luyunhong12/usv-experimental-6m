/** @file test_propulsion.cc
 * @brief 当前模块的解析回归验证。
 */
#include "PropulsionModel.hh"
#include <stdexcept>

/** @brief 校验测试判据，失败时报告错误。
 * @param x 应为真的测试判据。
 */
void require(bool x){if(!x)throw std::runtime_error("Propulsion analytic check failed");}

/** @brief 执行该文件定义的解析或场景回归验证。
 */
int main(){using namespace usv;PropulsionParameters p;
 require(std::abs(WetFraction(0,.15)-.5)<1e-12);require(WetFraction(.3,.15)==1);require(WetFraction(-.3,.15)==0);
 auto f=PropellerForces(p,1,p.maxOmega,p.maxOmega,0,1);require(std::abs(f.thrust-1000)<1e-9);
 auto r=PropellerForces(p,-1,-p.maxOmega,-p.maxOmega,0,1);require(std::abs(r.thrust+600)<1e-9);
 require(PropellerForces(p,0,100,0,0,1).motorTorque==0);
 require(PropellerForces(p,1,100,100,0,0).thrust==0);
 require(PropellerForces(p,1,0,100,0,1).thrust==0);
 for(double w:{-200.,-100.,0.,100.,200.})for(double v:{-8.,-2.,0.,2.,8.})for(double cmd:{-1.,0.,1.}){
  auto o=PropellerForces(p,cmd,w,cmd*p.maxOmega,v,1);
  require(std::abs(o.motorTorque)<=p.stallTorque+1e-9);require(std::abs(o.shaftPower)<=p.shaftPower+1e-9);
  require(o.waterTorque*w<=1e-9);require(o.thrust*v<=-o.waterTorque*w+1e-9);
 }
 // A reverse command cannot teleport a still-forward-spinning prop to reverse thrust.
 require(PropellerForces(p,-1,100,-100,0,1).thrust>0);
}
