/** @file test_steering_reference.cc
 * @brief 当前模块的解析回归验证。
 */
#include "SteeringReference.hh"
#include <stdexcept>

/** @brief 执行该文件定义的解析或场景回归验证。
 */
int main(){usv::SteeringReference ref;double dt=.004,limit=.610865238;
 for(int i=0;i<20000;++i){double target=i<3000?-limit:i<9000?limit:0.;double old=ref.position,v=ref.velocity;
 ref.Step(target,dt,.15,.3,limit);
 if(std::abs(ref.position)>limit+1e-10 || std::abs(ref.position-old)/dt>.150001 || std::abs(ref.velocity-v)/dt>.30001)throw std::runtime_error("Reference bound violated");
 }
 if(std::abs(ref.position)>.0001)throw std::runtime_error("Reference did not settle");
}
