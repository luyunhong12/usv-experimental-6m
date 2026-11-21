#pragma once
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <gz/math/Inertial.hh>
#include <gz/math/eigen3/Conversions.hh>
#include <array>
#include <vector>
#include <cmath>
#include <stdexcept>
namespace usv::fuel {
using V=Eigen::Vector3d;using M=Eigen::Matrix3d;
struct Props {double mass=0;V cg=V::Zero();M inertia=M::Zero();};
inline M Shift(const V&v){return v.squaredNorm()*M::Identity()-v*v.transpose();}
inline Props Combine(const Props&a,const Props&b){Props r;r.mass=a.mass+b.mass;if(r.mass<=0)return r;r.cg=(a.mass*a.cg+b.mass*b.cg)/r.mass;r.inertia=a.inertia+b.inertia+a.mass*Shift(a.cg-r.cg)+b.mass*Shift(b.cg-r.cg);return r;}
inline gz::math::Inertiald ToInertial(const Props&p){
 gz::math::MassMatrix3d m;m.SetMass(p.mass);m.SetMoi(gz::math::eigen3::convert(p.inertia));
 if(!m.IsValid() || !p.cg.allFinite())throw std::runtime_error("Invalid fuel-composite inertia");
 return {m,gz::math::Pose3d(gz::math::eigen3::convert(p.cg),gz::math::Quaterniond::Identity)};
}
struct Moments {double volume=0;V first=V::Zero();M second=M::Zero();};
class Tank {
 public:V size{.6,.4,.25},center{.08,0,.355};double density=840;
 // Outward box triangles. Coordinates about tank centre keep integrals conditioned.
 std::vector<std::array<V,3>> Faces()const {
  std::array<V,8> v;for(int i=0;i<8;++i)v[i]=V((i&1?1:-1)*size.x()/2,(i&2?1:-1)*size.y()/2,(i&4?1:-1)*size.z()/2);
  constexpr int f[12][3]={{0,2,3},{0,3,1},{4,5,7},{4,7,6},{0,1,5},{0,5,4},{2,6,7},{2,7,3},{0,4,6},{0,6,2},{1,3,7},{1,7,5}};
  std::vector<std::array<V,3>> result;for(auto &x:f)result.push_back({v[x[0]],v[x[1]],v[x[2]]});return result;
 }
 Moments Clip(const V&n,double h,const std::vector<std::array<V,3>>&faces,bool moments=true)const {
  Moments r;V ref=n*h;
  for(const auto&t:faces){std::vector<V> poly;for(int i=0;i<3;++i){const V&a=t[i],&b=t[(i+1)%3];double da=n.dot(a)-h,db=n.dot(b)-h;if(da<=0)poly.push_back(a);if((da<=0)!=(db<=0))poly.push_back(a+(b-a)*da/(da-db));}
   for(std::size_t i=1;i+1<poly.size();++i){const V&a=poly[0],&b=poly[i],&c=poly[i+1];double dv=(a-ref).dot((b-ref).cross(c-ref))/6.;V sum=ref+a+b+c;
    r.volume+=dv;if(moments){r.first+=dv*sum/4.;r.second+=dv*(sum*sum.transpose()+ref*ref.transpose()+a*a.transpose()+b*b.transpose()+c*c.transpose())/20.;}}
  }return r;
 }
 Props Fuel(double litres,const V&up)const {
  double full=size.prod(),target=litres*.001;
  if(!size.allFinite()||size.minCoeff()<=0||!center.allFinite()||!std::isfinite(density)||density<=0||!std::isfinite(litres)||target<0||target>full+1e-10||!up.allFinite()||up.norm()<1e-9)throw std::runtime_error("Invalid tank geometry, fill or gravity normal");
  Props p;if(target<=1e-15)return p;p.mass=target*density;
  if(target>=full-1e-12){p.cg=center;p.inertia.diagonal()=p.mass/12.*V(size.y()*size.y()+size.z()*size.z(),size.x()*size.x()+size.z()*size.z(),size.x()*size.x()+size.y()*size.y());return p;}
  V n=up.normalized();double extent=n.cwiseAbs().dot(size)/2,lo=-extent,hi=extent;auto faces=Faces();
  for(int i=0;i<42;++i){double mid=(lo+hi)/2;if(Clip(n,mid,faces,false).volume<target)lo=mid;else hi=mid;}
  auto q=Clip(n,(lo+hi)/2,faces);V c=q.first/q.volume;M covariance=q.second/q.volume-c*c.transpose();p.cg=center+c;p.inertia=p.mass*(covariance.trace()*M::Identity()-covariance);p.inertia=(p.inertia+p.inertia.transpose()).eval()/2;return p;
 }
};
}
