#pragma once
#include <gz/math/Vector3.hh>
#include <array>
#include <vector>
namespace usv {
using V=gz::math::Vector3d;
using Triangle=std::array<V,3>;
struct Volume { double volume=0; V centroid{0,0,0}; };
// Oriented closed mesh clipped against a horizontal free surface. Cap lies in
// reference plane, so its signed tetrahedral volume and first moment are zero.
inline Volume submerged(const std::vector<Triangle>& tris,double level,const V& reference) {
  Volume result; V moment(0,0,0); V ref(reference.X(),reference.Y(),level);
  for(const auto& t:tris) {
    std::vector<V> p;
    for(int i=0;i<3;++i) {
      const V &a=t[i], &b=t[(i+1)%3]; bool ia=a.Z()<=level,ib=b.Z()<=level;
      if(ia) p.push_back(a);
      if(ia!=ib) p.push_back(a+(b-a)*((level-a.Z())/(b.Z()-a.Z())));
    }
    for(size_t j=1;j+1<p.size();++j) {
      double vol=(p[0]-ref).Dot((p[j]-ref).Cross(p[j+1]-ref))/6.;
      result.volume+=vol;moment+=vol*(ref+p[0]+p[j]+p[j+1])/4.;
    }
  }
  if(result.volume>1e-10) result.centroid=moment/result.volume;
  else {result.volume=0;result.centroid=ref;}
  return result;
}
}
