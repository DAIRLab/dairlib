#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/common/drake_throw.h"

using Eigen::Matrix3D

namespace dairlib {
namespace examples {
namespace trifinger {

template<typename T>
class TrifingerPlant: public systems::LeafSystem
} // namespace trifinger
} // namespace examples
} // namespace dairlib