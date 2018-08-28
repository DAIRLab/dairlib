#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/output_vector.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using drake::systems::LeafSystem;
using drake::systems::Context;

namespace dairlib{
namespace systems {

class AffineController : public LeafSystem<double> {
 public:
  AffineController(int num_positions, int num_velocities, int num_inputs, VectorXd u_des, VectorXd x_des);

// get input port. output_input_port_ and config_input_port_ record 
  const drake::systems::InputPort<double>& get_input_port_config()
      const {
    return this->get_input_port(config_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_output()
      const {
    return this->get_input_port(output_input_port_);
  }

 private:
  void CalcControl(const Context<double>& context,
                   TimestampedVector<double>* output) const;

  int output_input_port_;
  int config_input_port_;

  // desired states and gain matrix
  VectorXd x_des_;
  MatrixXd K_;
  VectorXd u_des_;
  int num_states_;
  int num_inputs_;
};


}
}
