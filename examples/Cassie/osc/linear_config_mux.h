#pragma once

#include "systems/controllers/control_utils.h"
#include "systems/controllers/linear_controller.h"
#include "systems/framework/output_vector.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace cassie {
namespace osc {

/// LinearConfigMux combines K and x_desired into systems::LinearConfig for
/// systems::LinearController where the control input is
///   u = K * (x_desired - x)
/// The input format for K is a Eigen::VectorXd which consists of columns of K
class LinearConfigMux : public drake::systems::LeafSystem<double> {
 public:
  LinearConfigMux(const drake::multibody::MultibodyPlant<double>& plant);

  // Input/output ports
  const drake::systems::InputPort<double>& get_desired_state_input_port()
      const {
    return this->get_input_port(desired_state_port_);
  }
  const drake::systems::InputPort<double>& get_gains_input_port() const {
    return this->get_input_port(gains_port_);
  }

 private:
  void SetConfig(const drake::systems::Context<double>& context,
                 systems::LinearConfig* output) const;

  int num_positions_;
  int num_velocities_;
  int num_states_;
  int num_actuators_;

  int desired_state_port_;
  int gains_port_;
};

}  // namespace osc
}  // namespace cassie
}  // namespace dairlib
