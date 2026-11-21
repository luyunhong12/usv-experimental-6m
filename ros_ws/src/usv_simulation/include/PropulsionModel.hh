#pragma once
#include <algorithm>
#include <cmath>
namespace usv {
struct PropulsionParameters {
 double maxOmega=1500*2*M_PI/60,staticThrust=1000,reverseRatio=.6;
 double shaftPower=5040,stallTorque=60,gain=.3,tau=.6,deadband=.05,advanceSpeed=5.;
};
inline double WetFraction(double depth,double verticalRadius) {
 if(verticalRadius<1e-8)return depth>=0?1:0;
 double x=std::clamp(depth/verticalRadius,-1.,1.);
 return .5+(std::asin(x)+x*std::sqrt(std::max(0.,1-x*x)))/M_PI;
}
struct PropulsionOutput {double motorTorque=0,waterTorque=0,thrust=0,shaftPower=0;};
inline PropulsionOutput PropellerForces(const PropulsionParameters&p,double command,double omega,double reference,double axialVelocity,double wet) {
 PropulsionOutput out;
 double kq=p.shaftPower/std::pow(p.maxOmega,3);
 out.waterTorque=-wet*kq*omega*std::abs(omega);
 if(std::abs(command)>p.deadband) {
  double feed=wet*kq*reference*std::abs(reference);
  double demand=feed+p.gain*(reference-omega);
  double cap=std::min(p.stallTorque,p.shaftPower/std::max(std::abs(omega),1.));
  out.motorTorque=std::clamp(demand,-cap,cap);
 }
 double direction=omega>=0?1.:-1.;
 double speedRatio=std::abs(omega)/p.maxOmega;
 double inflow=direction*axialVelocity;
 double advance=std::clamp(1.-inflow/std::max(.2,p.advanceSpeed*speedRatio),0.,1.5);
 out.thrust=wet*p.staticThrust*speedRatio*speedRatio*direction*(omega>=0?1.:p.reverseRatio)*advance;
 // Positive translational work cannot exceed the current shaft-to-water work.
 double absorbed=std::max(0.,-out.waterTorque*omega);
 if(out.thrust*axialVelocity>absorbed && std::abs(axialVelocity)>1e-6)out.thrust=absorbed/axialVelocity;
 out.shaftPower=out.motorTorque*omega;
 return out;
}
}
