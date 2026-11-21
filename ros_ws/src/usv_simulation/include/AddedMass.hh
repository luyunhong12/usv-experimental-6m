#pragma once
#include "Hydrostatics.hh"
namespace usv {
struct AddedWrench {V force,torque;};
// Positive diagonal fluid inertia at a fixed body point. C_A is skew-symmetric:
// nu dot C_A(nu) nu = 0. Coefficients are NOT rigid mass or gravity loads.
inline AddedWrench AddedMassWrench(const V&m,const V&j,const V&v,const V&w,const V&dv,const V&dw){
 V p(m.X()*v.X(),m.Y()*v.Y(),m.Z()*v.Z()),h(j.X()*w.X(),j.Y()*w.Y(),j.Z()*w.Z());
 return {-V(m.X()*dv.X(),m.Y()*dv.Y(),m.Z()*dv.Z())-w.Cross(p),
         -V(j.X()*dw.X(),j.Y()*dw.Y(),j.Z()*dw.Z())-v.Cross(p)-w.Cross(h)};
}
}
