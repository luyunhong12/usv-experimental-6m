#include "Resistance.hh"
#include <cassert>
using namespace usv;
int main(){
 V l(120,600,1400),q(180,1500,1800),ref(.36,1.24,4.49);
 auto nominal=GeometryResistanceScales(10.3,ref,10.3,ref);
 for(double x:{-4.,-1.,0.,.1,4.})for(double y:{-2.,0.,2.}){
  V v(x,y,.5);auto f=GeometryDrag(l,q,v,nominal);
  for(int i=0;i<3;++i)assert(std::abs(f[i]+l[i]*v[i]+q[i]*v[i]*std::abs(v[i]))<1e-8);
  assert(f.Dot(v)<=0);
  auto wet=GeometryResistanceScales(20.6,ref*2,10.3,ref);
  assert((GeometryDrag(l,q,v,wet)-f*2).Length()<1e-8);
  assert(GeometryDrag(l,q,v,GeometryResistanceScales(0,V::Zero,10.3,ref)).Length()==0);
 }
 bool rejected=false;try{GeometryResistanceScales(1,ref,0,ref);}catch(...){rejected=true;}assert(rejected);
}
