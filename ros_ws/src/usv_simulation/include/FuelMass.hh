/** @file FuelMass.hh
 * @brief 无人艇FuelMass接口与实现；工程模型边界见项目文档。
 */
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

/** @brief 保存质量、重心以及重心处惯性张量。
 */
struct Props {double mass=0;V cg=V::Zero();M inertia=M::Zero();};

/** @brief 构造平行轴定理中的位移矩阵。
 * @param v 参考点之间的位移，m。
 */
inline M Shift(const V&v){return v.squaredNorm()*M::Identity()-v*v.transpose();}

/** @brief 合并同坐标系中的两组质量属性。
 * @param a 第一组同坐标系质量属性。
 * @param b 第二组同坐标系质量属性。
 */
inline Props Combine(const Props&a,const Props&b){Props r;r.mass=a.mass+b.mass;if(r.mass<=0)return r;r.cg=(a.mass*a.cg+b.mass*b.cg)/r.mass;r.inertia=a.inertia+b.inertia+a.mass*Shift(a.cg-r.cg)+b.mass*Shift(b.cg-r.cg);return r;}

/** @brief 校验质量属性并转换为 Gazebo 惯性对象。
 * @param p 待转换的质量、重心和惯性属性。
 */
inline gz::math::Inertiald ToInertial(const Props&p){
 gz::math::MassMatrix3d m;m.SetMass(p.mass);m.SetMoi(gz::math::eigen3::convert(p.inertia));
 if(!m.IsValid() || !p.cg.allFinite())throw std::runtime_error("Invalid fuel-composite inertia");
 return {m,gz::math::Pose3d(gz::math::eigen3::convert(p.cg),gz::math::Quaterniond::Identity)};
}

/** @brief 保存裁切多面体的体积及一阶、二阶几何矩。
 */
struct Moments {double volume=0;V first=V::Zero();M second=M::Zero();};

/** @brief 用准静态自由液面近似计算箱形油箱中的燃油属性。
 */
class Tank {
 public:V size{.6,.4,.25},center{.08,0,.355};double density=840;
 // Outward box triangles. Coordinates about tank centre keep integrals conditioned.

/** @brief 返回关于油箱中心的外向三角面片。
 */
std::vector<std::array<V,3>> Faces()const {
  std::array<V,8> v;for(int i=0;i<8;++i)v[i]=V((i&1?1:-1)*size.x()/2,(i&2?1:-1)*size.y()/2,(i&4?1:-1)*size.z()/2);
  constexpr int f[12][3]={{0,2,3},{0,3,1},{4,5,7},{4,7,6},{0,1,5},{0,5,4},{2,6,7},{2,7,3},{0,4,6},{0,6,2},{1,3,7},{1,7,5}};
  std::vector<std::array<V,3>> result;for(auto &x:f)result.push_back({v[x[0]],v[x[1]],v[x[2]]});return result;
 }

/** @brief 按给定平面裁切箱体并积分几何矩。
 * @param n 裁切平面的单位法向量。
 * @param h 沿法向的平面高度，m。
 * @param faces 外向定向的箱体三角面片。
 * @param moments 是否同时计算一阶与二阶矩。
 */
Moments Clip(const V&n,double h,const std::vector<std::array<V,3>>&faces,bool moments=true)const {
  Moments r;V ref=n*h;
  for(const auto&t:faces){std::vector<V> poly;for(int i=0;i<3;++i){const V&a=t[i],&b=t[(i+1)%3];double da=n.dot(a)-h,db=n.dot(b)-h;if(da<=0)poly.push_back(a);if((da<=0)!=(db<=0))poly.push_back(a+(b-a)*da/(da-db));}
   for(std::size_t i=1;i+1<poly.size();++i){const V&a=poly[0],&b=poly[i],&c=poly[i+1];double dv=(a-ref).dot((b-ref).cross(c-ref))/6.;V sum=ref+a+b+c;
    r.volume+=dv;if(moments){r.first+=dv*sum/4.;r.second+=dv*(sum*sum.transpose()+ref*ref.transpose()+a*a.transpose()+b*b.transpose()+c*c.transpose())/20.;}}
  }return r;
 }

/** @brief 按油量和重力方向求燃油质量、重心及惯性。
 * @param litres 燃油体积，L。
 * @param up 油箱坐标系中的向上方向。
 */
Props Fuel(double litres,const V&up)const {
  double full=size.prod(),target=litres*.001;
  if(!ValidGeometry() || !ValidFill(litres,target,full,up))throw std::runtime_error("Invalid tank geometry, fill or gravity normal");
  Props p;if(target<=1e-15)return p;p.mass=target*density;
  if(target>=full-1e-12){p.cg=center;p.inertia.diagonal()=p.mass/12.*V(size.y()*size.y()+size.z()*size.z(),size.x()*size.x()+size.z()*size.z(),size.x()*size.x()+size.y()*size.y());return p;}
  V n=up.normalized();double extent=n.cwiseAbs().dot(size)/2,lo=-extent,hi=extent;auto faces=Faces();
  for(int i=0;i<42;++i){double mid=(lo+hi)/2;if(Clip(n,mid,faces,false).volume<target)lo=mid;else hi=mid;}
  auto q=Clip(n,(lo+hi)/2,faces);V c=q.first/q.volume;M covariance=q.second/q.volume-c*c.transpose();p.cg=center+c;p.inertia=p.mass*(covariance.trace()*M::Identity()-covariance);p.inertia=(p.inertia+p.inertia.transpose()).eval()/2;return p;
 }
 private:
/** @brief 检查箱体尺寸、中心和燃油密度有效性。 */
bool ValidGeometry() const {
 return size.allFinite() && size.minCoeff()>0 && center.allFinite() && std::isfinite(density) && density>0;
}
/** @brief 检查油量和自由液面法向有效性。
 * @param litres 输入油量，L。
 * @param target 目标体积，m³。
 * @param full 箱体容积，m³。
 * @param up 油箱坐标系中的向上方向。
 */
bool ValidFill(double litres,double target,double full,const V&up) const {
 return std::isfinite(litres) && target>=0 && target<=full+1e-10 && up.allFinite() && up.norm()>=1e-9;
}

};
}
