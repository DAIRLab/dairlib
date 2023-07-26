#pragma once

#include <vector>
#include <utility>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "systems/framework/output_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/context.h"
#include <drake/systems/framework/vector_base.h>
#include "drake/multibody/plant/multibody_plant.h"

#include "dairlib/lcmt_robot_output.hpp"

using drake::systems::Context;
using drake::systems::LeafSystem;

namespace dairlib {
namespace systems {

class RobotOutputPassthrough : public LeafSystem<double> {
 public:
  RobotOutputPassthrough(
    const drake::multibody::MultibodyPlant<double>& plant);
  
 private: 
  void Passthrough(const drake::systems::Context<double>& context,
                    dairlib::lcmt_robot_output* output) const;
  int num_positions_;
  int num_velocities_;
  int num_efforts_;
};

}  // namespace systems
}  // namespace dairlib