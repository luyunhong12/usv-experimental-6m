/** @file test_hydrostatics.cc
 * @brief 当前模块的解析回归验证。
 */
#include "Hydrostatics.hh"
#include <iostream>
#include <cmath>
#include <stdexcept>

/** @brief 验证两个解析结果在规定容差内一致。
 * @param a 实际计算结果。
 * @param b 解析参考结果。
 */
void close(double a,double b){if(std::abs(a-b)>1e-8)throw std::runtime_error("Analytic hydrostatics mismatch");}

/** @brief 执行该文件定义的解析或场景回归验证。
 */
int main(){
 using namespace usv;
 std::vector<V> v={{-1,-1,-1},{1,-1,-1},{1,1,-1},{-1,1,-1},{-1,-1,1},{1,-1,1},{1,1,1},{-1,1,1}};
 int f[][3]={{0,2,1},{0,3,2},{4,5,6},{4,6,7},{0,1,5},{0,5,4},{1,2,6},{1,6,5},{2,3,7},{2,7,6},{3,0,4},{3,4,7}};
 std::vector<Triangle> m;for(auto&t:f)m.push_back({v[t[0]],v[t[1]],v[t[2]]});
 for(double h:{-2.,-1.,-.75,0.,.25,1.,2.}){auto s=submerged(m,h,V::Zero);double d=std::max(0.,std::min(2.,h+1));close(s.volume,4*d);if(d>0){close(s.centroid.X(),0);close(s.centroid.Y(),0);close(s.centroid.Z(),-1+d/2);}}
 auto moved=m;V off(1000,-500,20);for(auto&t:moved)for(auto&p:t)p+=off;
 auto s=submerged(moved,20,off);close(s.volume,4);close(s.centroid.X(),1000);close(s.centroid.Y(),-500);close(s.centroid.Z(),19.5);
 // Symmetric body rotated 45 degrees about X: plane bisects its volume.
 auto rotated=m;for(auto&t:rotated)for(auto&p:t){double y=p.Y(),z=p.Z();p.Y()=(y-z)/std::sqrt(2.);p.Z()=(y+z)/std::sqrt(2.);}
 close(submerged(rotated,0,V::Zero).volume,4);
 std::cout<<"dry, partial, full, translated and rotated analytic cases passed\n";
}
