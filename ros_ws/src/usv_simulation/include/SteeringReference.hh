#pragma once
#include <algorithm>
#include <cmath>
namespace usv {
struct SteeringReference {
 double position=0,velocity=0;
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
