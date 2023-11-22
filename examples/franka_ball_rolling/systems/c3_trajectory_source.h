#pragma once

#include <vector>
#include <utility>
#include <iostream>


#include <Eigen/Core>
#include <Eigen/Dense>

#include "systems/framework/output_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include <drake/common/trajectories/piecewise_polynomial.h>

using drake::systems::Context;
using drake::systems::LeafSystem;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using drake::trajectories::PiecewisePolynomial; 

namespace dairlib {
namespace systems {

class C3TrajectorySource : public LeafSystem<double> {
 public:
  C3TrajectorySource(const PiecewisePolynomial<double>& trajectory);
  
 private:
  // adds gravity conpensation to output
  void CalcOutput(const drake::systems::Context<double>& context,
                    TimestampedVector<double>* output) const;
  PiecewisePolynomial<double> trajectory_;
  PiecewisePolynomial<double> derivative_;
};

}  // namespace systems
}  // namespace dairlib