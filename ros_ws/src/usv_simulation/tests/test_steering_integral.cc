#include "SteeringIntegral.hh"
#include <cassert>
int main(){
 usv::SteeringIntegral s;double x=0,v=0,dt=.001;
 for(int i=0;i<60000;++i){double e=-x,raw=300*e-80*v+s.torque,applied=std::clamp(raw,-100.,100.);s.Update(e,dt,100,100,raw,applied);v+=(applied+10)*dt;x+=v*dt;}
 assert(std::abs(x)<1e-6);assert(std::abs(s.torque+10)<1e-4);
 double saved=s.torque;s.Update(1,.1,100,100,200,100);assert(s.torque==saved);
 s.Update(1,.1,100,100,50,0);assert(s.torque==saved); // power budget saturation
 s.Update(-1,.1,100,100,200,100);assert(s.torque<saved); // unwind
}
