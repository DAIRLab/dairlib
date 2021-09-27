#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/timestamped_vector.h"
#include "systems/framework/output_vector.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using drake::systems::LeafSystem;
using drake::systems::Context;

namespace dairlib{
namespace systems {

class LQR : public LeafSystem<double> {
 public:
  LQR(int num_positions, int num_velocities, int num_inputs,
      VectorXd x_des, MatrixXd K);

  const drake::systems::InputPort<double> &get_input_port_config()
  const {
    return this->get_input_port(config_input_port_);
  }

  const drake::systems::InputPort<double> &get_input_port_output()
  const {
    return this->get_input_port(output_input_port_);
  }

 private:
  void CalcControl(const Context<double> &context,
                   TimestampedVector<double> *output) const;

  int output_input_port_;
  int config_input_port_;

  VectorXd x_des_;
  MatrixXd K_;
};

}}