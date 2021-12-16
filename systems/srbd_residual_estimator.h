#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "systems/controllers/mpc/srbd_cmpc.h"
#include "multibody/single_rigid_body_plant.h"
#include <fstream>

namespace dairlib::systems {
class SRBDResidualEstimator : public drake::systems::LeafSystem<double> {
 public:
  SRBDResidualEstimator(const multibody::SingleRigidBodyPlant &plant,
                        double rate, unsigned int buffer_len,
                        bool use_fsm, double dt, bool continuous);

  // Want to connect this to a callback that adds the state to a deque
  const drake::systems::InputPort<double> &get_state_input_port() const {
    return this->get_input_port(state_in_port_);
  };

  const drake::systems::InputPort<double> &get_mpc_input_port() const {
    return this->get_input_port(mpc_in_port_);
  };

  const drake::systems::InputPort<double> &get_fsm_input_port() const {
    return this->get_input_port(fsm_port_);
  };

  const drake::systems::OutputPort<double> &get_residual_output_port() const {
    return this->get_output_port(residual_out_port_);
  };

  void AddMode(
      const LinearSrbdDynamics &dynamics,
      BipedStance stance, const Eigen::MatrixXd &reset, int N);

 private:
  // keep this plant to use its utility functions like getting the dynamics, etc.
  const multibody::SingleRigidBodyPlant &plant_;
  double rate_;
  unsigned int buffer_len_;

  // states from estimator get added to this, used to build least squares problem
  mutable Eigen::MatrixXd X_;
  // Transition states for least squares estimator.
  mutable Eigen::MatrixXd y_;
  mutable std::ofstream ofs_;


  // Output matrices
  mutable Eigen::Matrix<double, 12, 15> cur_A_hat_;
  mutable Eigen::Matrix<double, 12, 4> cur_B_hat_;
  mutable Eigen::Matrix<double, 12, 1> cur_b_hat_;

  int state_in_port_,
      residual_out_port_,
      fsm_port_,
      mpc_in_port_;

  int nx_ = 12;
  int nu_ = 4;
  // state + foot + force + b
  int num_X_cols = nx_ + 3 + nu_ + 1;

  bool use_fsm_;
  double dt_;
  bool continuous_;
  mutable unsigned int ticks_ = 0;
  std::vector<dairlib::SrbdMode> modes_;
  int nmodes_ = 0;

  // discrete update indices
  int current_fsm_state_idx_;
  int prev_event_time_idx_;

  // debugging variables
  mutable Eigen::VectorXd prev_state_;
  mutable Eigen::VectorXd prev_input_;

  // Solves the Least Squares Problem, connects matrices to outputs
  drake::systems::EventStatus PeriodicUpdate(
      const drake::systems::Context<double> &context,
      drake::systems::DiscreteValues<double> *discrete_state) const;

  // Only gets called when a new LCM message is present, since it's wrapped in an LCM driven loop.
  // Add the new state and dynamics to the least squares problem.
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double> &context,
      drake::systems::DiscreteValues<double> *discrete_state) const;

  void UpdateLstSqEquation(Eigen::VectorXd state,
                           Eigen::VectorXd input,
                           Eigen::Vector3d stance_foot_loc,
                           BipedStance stance_mode) const;

  // Returns the finite difference of the states here.
  // TODO(hersh500): add some filtering here.
  Eigen::VectorXd ComputeYDot(Eigen::MatrixXd state_history) const;

  // Solve the least squares equation periodically
  void SolveLstSq() const;

  void GetDynamics(const drake::systems::Context<double>& context,
                   residual_dynamics* dyn) const;

};
}
