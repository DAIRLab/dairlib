#pragma once

#include <vector>
#include <utility>
#include <iostream>


#include <Eigen/Core>
#include <Eigen/Dense>

#include "systems/framework/output_vector.h"
#include "drake/systems/framework/leaf_system.h"

using drake::systems::Context;
using drake::systems::LeafSystem;

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace dairlib {
namespace systems {

class C3TrajectoryConverter : public LeafSystem<double> {
 public:
  C3TrajectoryConverter();
  
 private:
  // adds gravity conpensation to output
  void Convert(const drake::systems::Context<double>& context,
                    TimestampedVector<double>* output) const;
};

}  // namespace systems
}  // namespace dairlib