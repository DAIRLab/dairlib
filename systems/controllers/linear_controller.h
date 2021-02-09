#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/output_vector.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using drake::systems::LeafSystem;
using drake::systems::Context;

static constexpr double kMaxError = 1.0;

namespace dairlib{
namespace systems {

class LinearController : public LeafSystem<double> {
 public:
  LinearController(int num_positions, int num_velocities, int num_inputs);

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

  VectorXd x_des_;
  MatrixXd K_;
};

/// Implementation of TimestampedVector to store, set, and get a linear
/// controller configuration
class LinearConfig : public TimestampedVector<double> {
  public:
    LinearConfig(int num_states, int num_inputs) :
        TimestampedVector<double>(num_states * num_inputs + num_states),
        num_states_(num_states), num_inputs_(num_inputs) {};

    //Getters and setters
    VectorXd GetDesiredState() const {return desired_state_;};

    MatrixXd GetK() const {return K_;};

    void SetDesiredState(VectorXd desired_state) {
      desired_state_ = desired_state;
    }

    void SetK(MatrixXd K) {
      K_ = K;
    }

  private:
    LinearConfig* DoClone() const override {
      return new LinearConfig(num_states_, num_inputs_);
    }

    int num_states_;
    int num_inputs_;

    VectorXd desired_state_;
    MatrixXd K_;
};

}
}