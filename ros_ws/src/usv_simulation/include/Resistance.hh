#pragma once
#include "Hydrostatics.hh"
#include <stdexcept>
namespace usv {
struct ResistanceScales {double surface;V projected;};
inline ResistanceScales GeometryResistanceScales(double area,const V&projected,double reference,const V&referenceProjected){
 if(!std::isfinite(area)||area<0||!std::isfinite(reference)||reference<=0)throw std::runtime_error("Invalid wet area reference");
 ResistanceScales s{area/reference,V::Zero};
 for(int i=0;i<3;++i){
  if(!std::isfinite(projected[i])||projected[i]<0||!std::isfinite(referenceProjected[i])||referenceProjected[i]<=0)throw std::runtime_error("Invalid projected wet area");
  s.projected[i]=projected[i]/referenceProjected[i];
 }
 return s;
}
inline V GeometryDrag(const V&linear,const V&quadratic,const V&velocity,const ResistanceScales&s){
 V f;for(int i=0;i<3;++i)f[i]=-linear[i]*s.surface*velocity[i]-quadratic[i]*s.projected[i]*velocity[i]*std::abs(velocity[i]);return f;
}
}
