#pragma once

#include <vector>
#include <utility>
#include <iostream>


#include <Eigen/Core>
#include <Eigen/Dense>

#include "systems/framework/output_vector.h"
#include "drake/systems/framework/leaf_system.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"

#include <drake/systems/framework/continuous_state.h>
#include <drake/systems/framework/vector_base.h>
#include <drake/multibody/math/spatial_velocity.h>

using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::systems::LeafSystem;

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace dairlib {
namespace systems {

class GravityCompensationRemover : public LeafSystem<double> {
 public:
  GravityCompensationRemover(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>& context);

 private:
  // adds gravity compensation to output
  void CancelGravityCompensation(const drake::systems::Context<double>& context,
                                 TimestampedVector<double>* output) const;

  // constructor variables
  const MultibodyPlant<double>& plant_;
  drake::systems::Context<double>& context_;
  int num_actuators_;
};

}  // namespace systems
}  // namespace dairlib