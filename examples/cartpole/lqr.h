#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/timestamped_vector.h"
#include "systems/framework/output_vector.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using drake::systems::LeafSystem;
using drake::systems::Context;

namespace dairlib::systems {

class LQR : public LeafSystem<double> {
 public:
  LQR(int num_positions, int num_velocities, int num_inputs,
      const VectorXd& x_des, const MatrixXd& K);

 private:
  void CalcControl(const Context<double> &context,
                   TimestampedVector<double> *output) const;

  VectorXd x_des_;
  MatrixXd K_;
};
}