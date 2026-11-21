#pragma once
#include "Hydrostatics.hh"
#include <algorithm>
#include <cmath>
#include <vector>
namespace usv {
struct WetSurface {double area=0,span=0;V center=V::Zero;};
// Convex x-z outline clipped against the (linearly sampled) local free surface.
inline WetSurface ClipAppendage(const std::vector<V>&points,const std::vector<double>&depth){
 std::vector<V> p;
 for(size_t i=0;i<points.size();++i){size_t j=(i+1)%points.size();
  if(depth[i]>=0)p.push_back(points[i]);
  if((depth[i]>=0)!=(depth[j]>=0))p.push_back(points[i]+(points[j]-points[i])*(depth[i]/(depth[i]-depth[j])));
 }
 WetSurface out;if(p.size()<3)return out;
 double lo=p[0].Z(),hi=lo;
 for(auto&v:p){lo=std::min(lo,v.Z());hi=std::max(hi,v.Z());}
 for(size_t i=1;i+1<p.size();++i){double a=.5*(p[i]-p[0]).Cross(p[i+1]-p[0]).Length();out.area+=a;out.center+=(p[0]+p[i]+p[i+1])*(a/3);}
 if(out.area>1e-12)out.center/=out.area;out.span=hi-lo;return out;
}
// Lift perpendicular to relative water velocity; drag always removes relative
// kinetic energy. Independent of heading, commanded rudder and desired path.
inline V AppendageForce(double rho,double area,double span,double cd0,double e,const V&velocity){
 double u=velocity.X(),v=velocity.Y(),speed=std::hypot(u,v);
 if(area<=1e-12||span<=1e-12||speed<=1e-9)return V::Zero;
 double ar=span*span/area;
 double slope=M_PI*ar/(1+std::sqrt(1+ar*ar/4));
 double alpha=std::atan2(v,std::abs(u));
 double blend=std::clamp((std::abs(alpha)-M_PI/12)/(M_PI/12),0.,1.);blend=blend*blend*(3-2*blend);
 double cl=(1-blend)*slope*alpha+blend*2*std::copysign(1.,alpha)*std::sin(alpha)*std::sin(alpha)*std::cos(alpha);
 double cd=cd0+cl*cl/(M_PI*e*ar)+blend*2*std::sin(alpha)*std::sin(alpha);
 double q=.5*rho*speed*speed*area,sign=u>=0?1.:-1.;
 return q*(cl*V(sign*v/speed,-std::abs(u)/speed,0)-cd*V(u/speed,v/speed,0));
}
}
