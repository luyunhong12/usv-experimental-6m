#pragma once
#include <gz/sim/components/Inertial.hh>
namespace usv::physics {
using RuntimeInertiaCmd=gz::sim::components::Component<gz::math::Inertiald,class RuntimeInertiaCmdTag,gz::sim::serializers::InertialSerializer>;
using RuntimeInertiaApplied=gz::sim::components::Component<gz::math::Inertiald,class RuntimeInertiaAppliedTag,gz::sim::serializers::InertialSerializer>;
GZ_SIM_REGISTER_COMPONENT("usv.RuntimeInertiaCmd",RuntimeInertiaCmd)
GZ_SIM_REGISTER_COMPONENT("usv.RuntimeInertiaApplied",RuntimeInertiaApplied)
}
