#pragma once
#include <algorithm>
#include "Hydrostatics.hh"
#include <cmath>
namespace usv {
struct WaveComponent {double amplitude,k,omega,dx,dy;};
struct WaveFrame {
 struct Sample {double height=0,dx=0,dy=0;V waterVelocity{0,0,0},airVelocity{0,0,0};};
 double time=0,level=0,ramp=1,rampRate=0;
 V current{0,0,0},wind{0,0,0};std::vector<WaveComponent> components;
 double Height(const V&p)const{
  double z=0;for(auto&w:components)z+=w.amplitude*ramp*std::cos(w.k*(p.X()*w.dx+p.Y()*w.dy)-w.omega*time);return z;
 }
 Sample At(const V&p)const{
  Sample s;s.waterVelocity=current;s.airVelocity=wind;
  for(auto&w:components){
   double theta=w.k*(p.X()*w.dx+p.Y()*w.dy)-w.omega*time,c=std::cos(theta),sn=std::sin(theta),a=w.amplitude;
   s.height+=a*ramp*c;s.dx-=a*ramp*w.k*w.dx*sn;s.dy-=a*ramp*w.k*w.dy*sn;
   // Linear deep-water orbital velocity, with depth attenuation. The ramp
   // derivative preserves the free-surface kinematic condition during startup.
   double decay=std::exp(w.k*std::min(0.,p.Z()-level));
   double horizontal=a*decay*(w.omega*ramp*c-rampRate*sn);
   s.waterVelocity+=V(w.dx*horizontal,w.dy*horizontal,a*decay*(w.omega*ramp*sn+rampRate*c));
  }
  return s;
 }
};
}
