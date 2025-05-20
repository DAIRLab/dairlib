/**This is used by the franka kinematics system. This system is also present under examples/franka/systems but has been 
 * copied here so as to be used by the sampling_c3 controller examples. */
#include "systems/franka_kinematics_vector.h"

#include "drake/common/default_scalars.h"

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::dairlib::systems::FrankaKinematicsVector)
