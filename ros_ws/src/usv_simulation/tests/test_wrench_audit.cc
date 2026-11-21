/** @file test_wrench_audit.cc
 * @brief 当前模块的解析回归验证。
 */
#include "WrenchAudit.hh"
#include <cassert>
#include <cmath>

/** @brief 执行该文件定义的解析或场景回归验证。
 */
int main(){
 using V=gz::math::Vector3d;
 usv::AuditFrame a;a.hull=gz::math::Pose3d(3,4,0,0,0,M_PI/2);a.center=V(3,5,0);
 auto msg=usv::AuditMessage(2,a);
 // World +Y is body +X. Force at body origin with CG 1 m ahead has no moment.
 usv::AuditWrench(msg,a,V(0,10,0),V::Zero,V(3,4,0));
 assert(std::abs(msg.data(4)-10)<1e-10);assert(std::abs(msg.data(9))<1e-10);
 // World -X is body +Y: a lateral force one metre aft gives -10 Nm yaw.
 auto m=usv::AuditMessage(2,a);usv::AuditWrench(m,a,V(-10,0,0),V::Zero,V(3,4,0));
 assert(std::abs(m.data(5)-10)<1e-10);assert(std::abs(m.data(9)+10)<1e-10);
 // Equivalent origin/couple representations must produce the same COM wrench.
 auto n=usv::AuditMessage(2,a);usv::AuditWrench(n,a,V(-10,0,0),V(0,0,-10),a.center);
 for(int i=4;i<10;++i)assert(std::abs(n.data(i)-m.data(i))<1e-10);
}
