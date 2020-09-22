#pragma once

#include "dairlib/lcmt_pd_control.hpp"
#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"
#include "drake/systems/framework/leaf_system.h"

using drake::systems::Context;
using drake::systems::LeafSystem;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace dairlib {
namespace systems {

class LinearController : public LeafSystem<double> {
 public:
  LinearController(int num_positions, int num_velocities, int num_inputs,
                   const drake::multibody::MultibodyPlant<double>& plant);

  const drake::systems::InputPort<double>& get_input_port_config() const {
    return this->get_input_port(config_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_output() const {
    return this->get_input_port(output_input_port_);
  }

  const drake::systems::OutputPort<double>& get_debug_port() const {
    return this->get_output_port(lcm_output_port_);
  }

 private:
  void CalcControl(const Context<double>& context,
                   TimestampedVector<double>* output) const;
  void AssignLcmOutput(const drake::systems::Context<double>& context,
                       dairlib::lcmt_pd_control* output) const;

  int output_input_port_;
  int config_input_port_;
  int lcm_output_port_;

  VectorXd x_des_;
  MatrixXd K_;
  int num_states_;
  int num_inputs_;

  // lcm message (for debugging)
  int num_positions_;
  int num_velocities_;
  int num_efforts_;
  std::vector<std::string> ordered_position_names_;
  std::vector<std::string> ordered_velocity_names_;
  std::vector<std::string> ordered_effort_names_;
  std::map<std::string, int> positionIndexMap_;
  std::map<std::string, int> velocityIndexMap_;
  std::map<std::string, int> effortIndexMap_;
};

/// Implementation of TimestampedVector to store, set, and get a linear
/// controller configuration
class LinearConfig : public TimestampedVector<double> {
 public:
  LinearConfig(int num_states, int num_inputs)
      : TimestampedVector<double>(num_states * num_inputs + num_states),
        num_states_(num_states),
        num_inputs_(num_inputs){};

  // Getters and setters
  VectorXd GetDesiredState() const { return desired_state_; };

  MatrixXd GetK() const { return K_; };

  void SetDesiredState(VectorXd desired_state) {
    desired_state_ = desired_state;
  }

  void SetK(MatrixXd K) { K_ = K; }

 private:
  LinearConfig* DoClone() const override {
    return new LinearConfig(num_states_, num_inputs_);
  }

  int num_states_;
  int num_inputs_;

  VectorXd desired_state_;
  MatrixXd K_;
};

}  // namespace systems
}  // namespace dairlib