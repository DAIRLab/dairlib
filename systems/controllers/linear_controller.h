#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/output_vector.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using drake::systems::LeafSystem;
using drake::systems::Context;

namespace dairlib{
namespace systems {

class LinearController : public LeafSystem<double> {
 public:
  LinearController(int num_positions, int num_velocities, int num_inputs);

  int config_input_port() {return config_input_port_;};
  int state_input_port() {return state_input_port_;};

 private:
  void CalcControl(const Context<double>& context,
                   TimestampedVector<double>* output) const;

  int state_input_port_;
  int config_input_port_;

  VectorXd x_des_;
  MatrixXd K_;
  int num_states_;
  int num_inputs_;
};

/// Implementation of TimestampedVector to store, set, and get a linear
/// controller configuration
class LinearConfig : public TimestampedVector<double> {
  public:
    LinearConfig(int num_states, int num_inputs);

    //Getters and setters
    VectorXd getDesiredState() const {return desired_state_;};

    VectorXd getK() const {return K_;};

    void setDesiredState(VectorXd desired_state) {
      desired_state_ = desired_state;
    }

    void setK(MatrixXd K) {
      K_ = K;
    }

  private:
    LinearConfig* DoClone() const override;

    VectorXd desired_state_;
    MatrixXd K_;
};

}
}