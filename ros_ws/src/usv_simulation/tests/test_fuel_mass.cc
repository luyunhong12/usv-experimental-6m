#include "FuelMass.hh"
#include <iostream>
using namespace usv::fuel;
void check(bool ok,const char*what){if(!ok)throw std::runtime_error(what);}
int main(){Tank t;
 for(double l:{0.,1.,6.,30.,60.}){
  auto p=t.Fuel(l,V::UnitZ());check(std::abs(p.mass-l*.84)<1e-10,"fuel density");if(l==0)continue;
  double h=l*.001/(.6*.4);check((p.cg-V(.08,0,.355-.125+h/2)).norm()<1e-10,"fill centroid");
  M expected=M::Zero();expected.diagonal()=p.mass/12*V(.4*.4+h*h,.6*.6+h*h,.6*.6+.4*.4);check((p.inertia-expected).norm()<1e-9,"box inertia");}
 for(double l:{1.,6.,30.,59.})for(V n:{V(.3,.4,1),V(-.3,.4,1),V(0,1,1),V(1,0,0)}){
  auto p=t.Fuel(l,n);check((p.inertia-p.inertia.transpose()).norm()<1e-10,"symmetry");check(Eigen::SelfAdjointEigenSolver<M>(p.inertia).eigenvalues().minCoeff()>0,"positive inertia");
  auto rest=t.Fuel(60-l,-n);auto full=Combine(p,rest);auto ref=t.Fuel(60,V::UnitZ());check((full.cg-ref.cg).norm()<1e-9 && (full.inertia-ref.inertia).norm()<1e-8,"complement recombines full tank");}
 auto p=t.Fuel(30,V(0,1,1));check(p.cg.y()<0,"fuel moves down tilt");
 check(t.Fuel(0,V::UnitZ()).mass==0,"empty tank");
 std::cout<<"PASS analytic levels, tilted complementary volumes, tensors and bounds\n";
}
