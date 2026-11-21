#pragma once
#include "Hydrostatics.hh"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace usv {
// Integrate p=rho*g*(eta(x,y)-z) over the clipped, outward-oriented hull.
// eta is linearly interpolated on each refined triangle. This is a
// distributed quasi-hydrostatic/Froude-Krylov approximation, not CFD.
struct PressureResult {
 V force{0,0,0},torque{0,0,0},firstMoment{0,0,0};
 double volume=0,wetArea=0;
 V Centroid()const{return volume>1e-10?firstMoment/volume:V::Zero;}
};
inline void AccumulatePressure(const Triangle&t,const std::array<double,3>&h,
                               double rhoG,PressureResult&r){
 struct Point {V p;double h;};std::array<Point,5> wet;int count=0;
 for(int i=0;i<3;++i){int j=(i+1)%3;bool ai=h[i]>=0,bi=h[j]>=0;
  if(ai)wet[count++]={t[i],h[i]};
  if(ai!=bi){double u=h[i]/(h[i]-h[j]);wet[count++]={t[i]+u*(t[j]-t[i]),0};}
 }
 for(int j=1;j+1<count;++j){auto a=wet[0],b=wet[j],c=wet[j+1];
  V n=(b.p-a.p).Cross(c.p-a.p)*.5;double area=n.Length();if(area<1e-14)continue;
  double sum=a.h+b.h+c.h,mean=sum/3.;
  // Exact integral of position*linear pressure over a triangle, divided by A.
  V weighted=((a.p+b.p+c.p)*sum+a.p*a.h+b.p*b.h+c.p*c.h)/12.;
  double h2=(a.h*a.h+b.h*b.h+c.h*c.h+a.h*b.h+a.h*c.h+b.h*c.h)/6.;
  r.force-=n*(rhoG*mean);r.torque-=weighted.Cross(n)*rhoG;r.wetArea+=area;
  // Divergence-theorem displaced-volume moments; no artificial planar cap.
  r.volume-=n.Z()*mean;r.firstMoment.X()-=n.Z()*weighted.X();r.firstMoment.Y()-=n.Z()*weighted.Y();
  r.firstMoment.Z()-=n.Z()*(weighted.Z()+.5*h2);
 }
}
inline void RefineTriangle(const Triangle&t,double maxEdge,std::vector<Triangle>&out,int depth=0){
 double longest=std::max({(t[0]-t[1]).Length(),(t[1]-t[2]).Length(),(t[2]-t[0]).Length()});
 if(longest<=maxEdge){out.push_back(t);return;}
 if(depth>=10)throw std::runtime_error("Hydrostatic mesh exceeds refinement depth bound");
 V a=(t[0]+t[1])*.5,b=(t[1]+t[2])*.5,c=(t[2]+t[0])*.5;
 for(const Triangle&v:{Triangle{t[0],a,c},Triangle{a,t[1],b},Triangle{c,b,t[2]},Triangle{a,b,c}})RefineTriangle(v,maxEdge,out,depth+1);
}
}
