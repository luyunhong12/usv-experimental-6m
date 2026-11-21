#pragma once
#include <algorithm>
#include <cmath>
namespace usv {
// Integral torque with anti-windup against BOTH effort and energy saturation.
struct SteeringIntegral {
 double torque=0;
 void Update(double error,double dt,double ki,double limit,double requested,double applied){
  if(std::abs(requested-applied)<1e-7 || error*requested<0)
   torque=std::clamp(torque+ki*error*dt,-limit,limit);
 }
};
}
