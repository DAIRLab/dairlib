//
// Created by hersh on 11/15/21.
//
#include <iostream>
#include <fstream>
#include "srbd_residual_estimator.h"
#include "systems/framework/output_vector.h"
#include "dairlib/lcmt_saved_traj.hpp"
#include "lcm/lcm_trajectory.h"
#include "systems/controllers/mpc/mpc_periodic_residual_manager.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using drake::systems::BasicVector;
using dairlib::systems::OutputVector;
using dairlib::systems::residual_dynamics;

namespace dairlib::systems {
SRBDResidualEstimator::SRBDResidualEstimator(
    const multibody::SingleRigidBodyPlant &plant, double rate,
    unsigned int buffer_len, bool use_fsm, double dt, bool continuous) :
    plant_(plant),
    rate_(rate),
    buffer_len_(buffer_len),
    use_fsm_(use_fsm),
    dt_(dt),
    continuous_(continuous) {

  // Initialize data matrices
  X_ = MatrixXd::Zero(buffer_len_, num_X_cols);
  y_ = MatrixXd::Zero(buffer_len_, nx_);
  prev_state_ = VectorXd::Zero(nx_ + 3);
  prev_input_ = VectorXd::Zero(nu_);

  // Declare all ports
  state_in_port_ = this->DeclareVectorInputPort(
          "x, u, t",
          OutputVector<double>(
              plant_.nq(),plant_.nv(),plant_.nu()))
          .get_index();

  mpc_in_port_ = this->DeclareAbstractInputPort(
      "mpc_traj_input",
      drake::Value<lcmt_saved_traj>{}).get_index();

  residual_out_port_ = this->DeclareAbstractOutputPort(
      "residuals_out",
      residual_dynamics{MatrixXd::Zero(0,0),
                                MatrixXd::Zero(0,0),
                                VectorXd::Zero(0)},
      &SRBDResidualEstimator::GetDynamics).get_index();

  if (use_fsm_) {
    fsm_port_ = this->DeclareVectorInputPort(
            "fsm",
            BasicVector<double>(1))
        .get_index();

    current_fsm_state_idx_ =
        this->DeclareDiscreteState(VectorXd::Zero(1));
    prev_event_time_idx_ = this->DeclareDiscreteState(-0.1 * VectorXd::Ones(1));
  }


  DeclarePerStepDiscreteUpdateEvent(&SRBDResidualEstimator::DiscreteVariableUpdate);
  DeclarePeriodicDiscreteUpdateEvent(rate_, 0, &SRBDResidualEstimator::PeriodicUpdate);
  ofs_ = std::ofstream("log2.txt");
  if (! ofs_) {
    std::cout << "could not log!" << std::endl;
  }
}

void SRBDResidualEstimator::AddMode(const LinearSrbdDynamics &dynamics,
                                    BipedStance stance, const MatrixXd &reset, int N) {
  DRAKE_DEMAND(stance == nmodes_);
  SrbdMode mode = {dynamics, reset, stance, N};
  modes_.push_back(mode);
  nmodes_++;
}

drake::systems::EventStatus SRBDResidualEstimator::PeriodicUpdate(
    const drake::systems::Context<double> &context,
    drake::systems::DiscreteValues<double> *discrete_state) const {
  // Solve least squares only if it's ready.
  if (ticks_ >= buffer_len_) {
    SolveLstSq();
  }
  return drake::systems::EventStatus::Succeeded();
}

void SRBDResidualEstimator::GetDynamics(const drake::systems::Context<double> &context,
                                        residual_dynamics *dyn) const {
  *dyn = residual_dynamics { cur_A_hat_, cur_B_hat_, cur_b_hat_ };
}

// For now, calling this as a discrete variable update though it doesn't have to be.
drake::systems::EventStatus
SRBDResidualEstimator::DiscreteVariableUpdate(const drake::systems::Context<double> &context,
                                              drake::systems::DiscreteValues<double> *discrete_state) const {
  const OutputVector<double> *robot_output =
      (OutputVector<double> *) this->EvalVectorInput(context, state_in_port_);

  const drake::AbstractValue* mpc_cur_input = this->EvalAbstractInput(context, mpc_in_port_);
  const auto& input_msg = mpc_cur_input->get_value<lcmt_saved_traj>();
  LcmTrajectory lcm_traj(input_msg);
  Eigen::VectorXd u_nom;
  if (lcm_traj.GetTrajectoryNames().size() > 0) {
    LcmTrajectory::Trajectory input_traj = lcm_traj.GetTrajectory("input_traj");
    u_nom = input_traj.datapoints.col(0);
  } else {
    u_nom = Eigen::VectorXd::Zero(nu_);
  }

  double timestamp = robot_output->get_timestamp();

  // FSM stuff (copied from the srbd_mpc)
  const BasicVector<double> *fsm_output =
  (BasicVector<double> *) this->EvalVectorInput(context, fsm_port_);
  VectorXd fsm_state = fsm_output->get_value();
  if (use_fsm_) {
    auto current_fsm_state =
        discrete_state->get_mutable_vector(current_fsm_state_idx_)
            .get_mutable_value();

    if (fsm_state(0) != current_fsm_state(0)) {
      current_fsm_state(0) = fsm_state(0);
      discrete_state->get_mutable_vector(prev_event_time_idx_).get_mutable_value()
          << timestamp;
    }
  }

  // Full robot state
  VectorXd x = robot_output->GetState();

  // Get the srbd state and foot state
  VectorXd srbd_state = plant_.CalcSRBStateFromPlantState(x);
//  std::cout << "--------------" << std::endl;
//  std::cout << "srbd state" << std::endl;
//  std::cout << srbd_state << std::endl;
  // switch based on the contact state.
  std::vector<Eigen::Vector3d> foot_locs = plant_.CalcFootPositions(x);

  Eigen::Vector3d foot_loc = foot_locs[fsm_state(0)];
//  std::cout << foot_loc << std::endl;
  BipedStance cur_stance_mode = modes_.at(fsm_state(0)).stance;

  UpdateLstSqEquation(srbd_state, u_nom, foot_loc, cur_stance_mode);

  if (ticks_ < buffer_len_) {
    ticks_++;
  }
  return drake::systems::EventStatus::Succeeded();
}

Eigen::VectorXd SRBDResidualEstimator::ComputeYDot(Eigen::MatrixXd state_history) const {
  if (state_history.cols() < 2) {
    std::cout << "Error! Need at least 2 states to compute velocity" << std::endl;
  }
  Eigen::VectorXd diff = 1/dt_ * (state_history.col(1) - state_history.col(0));
  return diff;
}


void SRBDResidualEstimator::UpdateLstSqEquation(Eigen::VectorXd state,
                                                Eigen::VectorXd input,
                                                Eigen::Vector3d stance_foot_loc,
                                                BipedStance stance_mode) const {
  VectorXd vec_joined(nx_ + 3);
  vec_joined << state, stance_foot_loc;
  // Rotate X and y up by a row.
  X_.block(0, 0, buffer_len_ - 1, num_X_cols) = X_.block(1, 0, buffer_len_ - 1, num_X_cols);
  y_.block(0, 0, buffer_len_ - 1, nx_) = y_.block(1, 0, buffer_len_ - 1, nx_);

  // set the last row of X to the current state and input, and ones
  VectorXd alls(num_X_cols);
  alls << state, stance_foot_loc, input, Eigen::VectorXd::Ones(1);
//  std::cout << "alls \n" << alls << std::endl;
  X_.row(buffer_len_ - 1) = alls;

  if (ofs_ && ticks_ == 0) {
    ofs_ << "#" << std::endl;
    ofs_ << modes_.at(stance_mode).dynamics.A << std::endl;
    ofs_ << "#" << std::endl;
    ofs_ << modes_.at(stance_mode).dynamics.B << std::endl;
    ofs_ << "#" << std::endl;
    ofs_ << modes_.at(stance_mode).dynamics.b << std::endl;

    //std::cout << "A matrix \n" << modes_.at(stance_mode).dynamics.A << std::endl;
    //std::cout << "B matrix \n" << modes_.at(stance_mode).dynamics.B << std::endl;
    //std::cout << "B matrix \n" << modes_.at(stance_mode).dynamics.b << std::endl;
  }

  if (ofs_) {
    Eigen::VectorXd vec_row(nx_ + 3 + nu_ + 1);
    vec_row << state, stance_foot_loc, input, Eigen::VectorXd::Ones(1);
    ofs_ << "#" << std::endl;
    ofs_ << vec_row << std::endl;
  }

  Eigen::MatrixXd states(nx_, 2);
  states.col(0) = prev_state_.head(nx_);
  states.col(1) = state;
  // Eventually can do some filtering in this step.
  Eigen::VectorXd ydot = ComputeYDot(states);
  // std::cout << "ydot \n" << ydot << std::endl;
  Eigen::VectorXd nominal_deriv = modes_.at(stance_mode).dynamics.A * prev_state_ + modes_.at(stance_mode).dynamics.B * prev_input_ +
      modes_.at(stance_mode).dynamics.b;
  // std::cout << "prev_state:" << prev_state_ << std::endl;
  // std::cout << "prev_input:" << prev_input_ << std::endl;
  // std::cout << "nominal derivative: " << nominal_deriv << std::endl;
  if (continuous_) {
    y_.row(buffer_len_ - 2) = ydot - nominal_deriv;
  } else {
    y_.row(buffer_len_ - 2) =
        state - nominal_deriv;
  }
  // std::cout << "last row of y:" << std::endl << y_.row(buffer_len_ - 2)<< std::endl;
  if (ofs_) {
    ofs_ << "#" << std::endl;
    ofs_ << X_ << std::endl;
    ofs_ << "#" << std::endl;
    ofs_ << y_ << std::endl;
  }

  if (ticks_ >= buffer_len_) {
    // debugging step: check if the dynamics of the past state + residual dynamics of past state are close to this state.

    Eigen::VectorXd exp_deriv = modes_.at(stance_mode).dynamics.A * prev_state_ + modes_.at(stance_mode).dynamics.B * prev_input_ +
        modes_.at(stance_mode).dynamics.b;
    Eigen::VectorXd res_state = cur_A_hat_ * prev_state_ + cur_B_hat_ * prev_input_ + cur_b_hat_;
    std::cout << "-----------------" << std::endl;
    std::cout << "prev_state:" << prev_state_ << std::endl;
    std::cout << "prev_input:" << prev_input_ << std::endl;
    std::cout << "nominal derivative: " << exp_deriv << std::endl;
    std::cout << "residual: " << res_state << std::endl;
    std::cout << "actual next deriv: " << ydot << std::endl;
    std::cout << "nominal + residual: " << exp_deriv + res_state << std::endl;
    std::cout << "-----------------" << std::endl;
  }

  prev_state_ = vec_joined;
  prev_input_ = input;
}

void SRBDResidualEstimator::SolveLstSq() const {
  // Solve the least squares equation, excluding the last row of X and y because it is always incomplete.
  Eigen::MatrixXd X_c = X_.block(0, 0, buffer_len_ - 1, num_X_cols);
  Eigen::MatrixXd y_c = y_.block(0, 0, buffer_len_ - 1, nx_);

  Eigen::MatrixXd soln = (X_c.transpose() * X_c).colPivHouseholderQr().solve(X_c.transpose() * y_c).transpose();

  // Select the appropriate parts of the solution matrix for the residuals
  cur_A_hat_ = soln.block(0, 0, nx_, nx_ + 3);
  cur_B_hat_ = soln.block(0, nx_ + 3, nx_, nu_);
  cur_b_hat_ = soln.block(0, nx_+ 3 + nu_, nx_, 1);

//  cur_b_hat_ = soln.block(0, nx_ + nu_ + 3, nx_, nx_).diagonal();
//  std::cout << "#################################" << std::endl;
//  std::cout << modes_.at(0).dynamics.A << std::endl;
//  std::cout << "#################################" << std::endl;
//  std::cout << "Current A_hat " << cur_A_hat_ << std::endl;
//  std::cout << "#################################" << std::endl;
//  std::cout << "Current B_hat" << cur_B_hat_ << std::endl;
//
//  std::cout << "#################################" << std::endl;
//  std::cout << modes_.at(0).dynamics.b << std::endl;
//  std::cout << "#################################" << std::endl;
//  std::cout << "Current b_hat" << cur_b_hat_ << std::endl;
}

}
