#include "WavePressure.hh"
#include <iostream>
#include <cmath>
#include <stdexcept>
using namespace usv;
void close(double a,double b,double tol=1e-8){if(std::abs(a-b)>tol)throw std::runtime_error("Distributed pressure analytic mismatch: "+std::to_string(a)+" vs "+std::to_string(b));}
std::vector<Triangle> box(){
 std::vector<V> v={{-1,-1,-1},{1,-1,-1},{1,1,-1},{-1,1,-1},{-1,-1,1},{1,-1,1},{1,1,1},{-1,1,1}};
 int f[][3]={{0,2,1},{0,3,2},{4,5,6},{4,6,7},{0,1,5},{0,5,4},{1,2,6},{1,6,5},{2,3,7},{2,7,6},{3,0,4},{3,4,7}};
 std::vector<Triangle> m;for(auto&t:f)m.push_back({v[t[0]],v[t[1]],v[t[2]]});return m;
}
template<class Height>PressureResult integrate(const std::vector<Triangle>&m,Height eta){PressureResult r;for(auto&t:m)AccumulatePressure(t,{eta(t[0])-t[0].Z(),eta(t[1])-t[1].Z(),eta(t[2])-t[2].Z()},9810,r);return r;}
int main(){
 auto m=box();
 for(double level:{-2.,-.75,0.,.25,2.}){
  auto p=integrate(m,[=](V){return level;});auto v=submerged(m,level,V::Zero);
  close(p.volume,v.volume);close(p.force.Z(),9810*v.volume);close(p.force.X(),0);close(p.force.Y(),0);
  if(v.volume>0){close((p.Centroid()-v.centroid).Length(),0);close((p.torque-v.centroid.Cross(p.force)).Length(),0);}
 }
 const double sx=.2,sy=-.1;auto tilt=integrate(m,[=](V p){return sx*p.X()+sy*p.Y();});
 close(tilt.volume,4);close(tilt.force.X(),-9810*4*sx);close(tilt.force.Y(),-9810*4*sy);close(tilt.force.Z(),9810*4);
 close(tilt.Centroid().X(),sx/3);close(tilt.Centroid().Y(),sy/3);close(tilt.Centroid().Z(),-.5+(sx*sx+sy*sy)/6);
 double previous=1e9,last=0;const double a=.15,k=1.3,exact=4+4*a*std::sin(k)/k;
 for(double edge:{.7,.35,.175}){std::vector<Triangle> refined;for(auto&t:m)RefineTriangle(t,edge,refined);auto p=integrate(refined,[=](V v){return a*std::cos(k*v.X());});double error=std::abs(p.volume-exact);if(error>=previous)throw std::runtime_error("Curved-wave refinement did not converge");previous=error;last=error;close(p.force.X(),0,1e-7);close(p.force.Y(),0,1e-7);}
 if(last/exact>.0002)throw std::runtime_error("Curved-wave quadrature error too large");
 // Translation must not change forces; moments transform by offset x force.
 V offset(4,-3,2);auto moved=m;for(auto&t:moved)for(auto&v:t)v+=offset;auto p=integrate(moved,[](V){return 2.;});close((p.force-V(0,0,39240)).Length(),0);close((p.torque-offset.Cross(p.force)).Length(),0);close((p.Centroid()-V(4,-3,1.5)).Length(),0);
 std::cout<<"Flat/tilted surfaces, dry/full immersion, exact moments, translation and curved-wave convergence passed\n";
}
