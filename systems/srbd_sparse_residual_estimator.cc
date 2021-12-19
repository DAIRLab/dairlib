//
// Created by hersh on 11/15/21.
//
#include <iostream>
#include <fstream>
#include "srbd_sparse_residual_estimator.h"
#include "systems/framework/output_vector.h"
#include "dairlib/lcmt_saved_traj.hpp"
#include "lcm/lcm_trajectory.h"
#include "systems/controllers/mpc/mpc_periodic_residual_manager.h"
#include "common/eigen_utils.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using drake::systems::BasicVector;
using dairlib::systems::OutputVector;
using dairlib::systems::residual_dynamics;

namespace dairlib::systems {
// Currently only setup for continuous time.
SRBDSparseResidualEstimator::SRBDSparseResidualEstimator(
    const multibody::SingleRigidBodyPlant &plant, double rate,
    unsigned int buffer_len, bool use_fsm, double dt, double trans_reg, double rot_reg) :
    plant_(plant),
    rate_(rate),
    buffer_len_(buffer_len),
    use_fsm_(use_fsm),
    dt_(dt),
    trans_reg_(trans_reg),
    rot_reg_(rot_reg) {

  // screw code reusability amirite
  // A_1 x[6:9]
  X_1 = MatrixXd::Zero(buffer_len_, X_cols[0]);
  // A_2x[9:12]
  X_2 = MatrixXd::Zero(buffer_len_, X_cols[1]);
  // B_1u[0:3] + b[6:9]
  X_3 = MatrixXd::Zero(buffer_len_, X_cols[2]);
  // A_3x[0:3] + A_4x[12:15] + B_2u[0:3] + B_3u[3] + b[9:12]
  X_4 = MatrixXd::Zero(buffer_len_, X_cols[3]);
  y_1 = MatrixXd::Zero(buffer_len_, state_block_size);
  y_2 = MatrixXd::Zero(buffer_len_, state_block_size);
  y_3 = MatrixXd::Zero(buffer_len_, state_block_size);
  y_4 = MatrixXd::Zero(buffer_len_, state_block_size);

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
      &SRBDSparseResidualEstimator::GetDynamics).get_index();

  residual_debug_out_port_ = this->DeclareAbstractOutputPort(
      "residual_lcm_out", &SRBDSparseResidualEstimator::GetDynamicsLcm).get_index();

  if (use_fsm_) {
    fsm_port_ = this->DeclareVectorInputPort(
            "fsm",
            BasicVector<double>(1))
        .get_index();

    current_fsm_state_idx_ =
        this->DeclareDiscreteState(VectorXd::Zero(1));
    prev_event_time_idx_ = this->DeclareDiscreteState(-0.1 * VectorXd::Ones(1));
  }


  DeclarePerStepDiscreteUpdateEvent(&SRBDSparseResidualEstimator::DiscreteVariableUpdate);
  DeclarePeriodicDiscreteUpdateEvent(rate_, 0, &SRBDSparseResidualEstimator::PeriodicUpdate);
  // ofs_ = std::ofstream("log2.txt");
  if (!ofs_) {
    std::cout << "could not log!" << std::endl;
  }
}

void SRBDSparseResidualEstimator::AddMode(const LinearSrbdDynamics &dynamics,
                                          BipedStance stance, const MatrixXd &reset, int N) {
  DRAKE_DEMAND(stance == nmodes_);
  SrbdMode mode = {dynamics, reset, stance, N};
  modes_.push_back(mode);
  nmodes_++;
}

drake::systems::EventStatus SRBDSparseResidualEstimator::PeriodicUpdate(
    const drake::systems::Context<double> &context,
    drake::systems::DiscreteValues<double> *discrete_state) const {
  // Solve least squares only if it's ready.
  if (ticks_ >= buffer_len_) {
    SolveLstSq();
  }
  return drake::systems::EventStatus::Succeeded();
}

void SRBDSparseResidualEstimator::GetDynamics(const drake::systems::Context<double> &context,
                                              residual_dynamics *dyn) const {
  *dyn = residual_dynamics { cur_A_hat_, cur_B_hat_, cur_b_hat_ };
}

void SRBDSparseResidualEstimator::GetDynamicsLcm(
    const drake::systems::Context<double> &context,
    lcmt_residual_dynamics *dyn_lcm) const {
  int nx = cur_A_hat_.rows();
  int nu = cur_B_hat_.cols();
  int npx = cur_A_hat_.cols();
  dyn_lcm->utime = context.get_time() * 1e6;
  dyn_lcm->nx = nx;
  dyn_lcm->nu = nu;
  dyn_lcm->npx = npx;
  dyn_lcm->A = std::vector<std::vector<double>>(nx);
  dyn_lcm->B = std::vector<std::vector<double>>(nx);
  for (int i = 0; i < nx; i++) {
    dyn_lcm->A.at(i) = CopyVectorXdToStdVector(cur_A_hat_.row(i).transpose());
    dyn_lcm->B.at(i) = CopyVectorXdToStdVector(cur_B_hat_.row(i).transpose());
  }
  dyn_lcm->b = CopyVectorXdToStdVector(cur_b_hat_);
}

// For now, calling this as a discrete variable update though it doesn't have to be.
drake::systems::EventStatus
SRBDSparseResidualEstimator::DiscreteVariableUpdate(const drake::systems::Context<double> &context,
                                                    drake::systems::DiscreteValues<double> *discrete_state) const {
  const OutputVector<double> *robot_output =
      (OutputVector<double> *) this->EvalVectorInput(context, state_in_port_);

  const drake::AbstractValue* mpc_cur_input = this->EvalAbstractInput(context, mpc_in_port_);
  const auto& input_msg = mpc_cur_input->get_value<lcmt_saved_traj>();
  LcmTrajectory lcm_traj(input_msg);
  Eigen::VectorXd u_nom;
  if (lcm_traj.GetTrajectoryNames().size() > 0) {
    LcmTrajectory::Trajectory input_traj = lcm_traj.GetTrajectory("input_traj");
    double s = (context.get_time() - input_traj.time_vector(0)) /
        (input_traj.time_vector(1) - input_traj.time_vector(0));
    u_nom = (1.0 - s) * input_traj.datapoints.col(0) + s * input_traj.datapoints.col(1);
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

Eigen::VectorXd SRBDSparseResidualEstimator::ComputeYDot(Eigen::MatrixXd state_history) const {
  if (state_history.cols() < 2) {
    std::cout << "Error! Need at least 2 states to compute velocity" << std::endl;
  }
  Eigen::VectorXd diff = 1/dt_ * (state_history.col(1) - state_history.col(0));
  return diff;
}


void SRBDSparseResidualEstimator::UpdateLstSqEquation(Eigen::VectorXd state,
                                                      Eigen::VectorXd input,
                                                      Eigen::Vector3d stance_foot_loc,
                                                      BipedStance stance_mode) const {
  VectorXd vec_joined(nx_ + 3);
  vec_joined << state, stance_foot_loc;
  // Rotate X and y up by a row.
  // This is a terrible way to do this.
  X_1.block(0, 0, buffer_len_ - 1, X_cols[0]) = X_1.block(1, 0, buffer_len_ - 1, X_cols[0]);
  y_1.block(0, 0, buffer_len_ - 1, state_block_size) = y_1.block(1, 0, buffer_len_ - 1, state_block_size);

  X_2.block(0, 0, buffer_len_ - 1, X_cols[1]) = X_2.block(1, 0, buffer_len_ - 1, X_cols[1]);
  y_2.block(0, 0, buffer_len_ - 1, state_block_size) = y_2.block(1, 0, buffer_len_ - 1, state_block_size);

  X_3.block(0, 0, buffer_len_ - 1, X_cols[2]) = X_3.block(1, 0, buffer_len_ - 1, X_cols[2]);
  y_3.block(0, 0, buffer_len_ - 1, state_block_size) = y_3.block(1, 0, buffer_len_ - 1, state_block_size);

  X_4.block(0, 0, buffer_len_ - 1, X_cols[3]) = X_4.block(1, 0, buffer_len_ - 1, X_cols[3]);
  y_4.block(0, 0, buffer_len_ - 1, state_block_size) = y_4.block(1, 0, buffer_len_ - 1, state_block_size);


  // Build the corresponding data matrices
  X_1.row(buffer_len_ - 1) = vec_joined.segment(6, state_block_size);
  // std::cout << "X_1 last row:\n" << X_1.row(buffer_len_ - 1) << std::endl;
  X_2.row(buffer_len_ - 1) = vec_joined.segment(9, state_block_size);
  // std::cout << "X_2 last row:\n" << X_2.row(buffer_len_ - 1) << std::endl;
  X_3.row(buffer_len_ - 1).segment(0, 3) = input.segment(0, 3);
  X_3.row(buffer_len_ - 1).segment(3, 1) = VectorXd::Ones(1);
  // std::cout << "X_3 last row:\n" << X_3.row(buffer_len_ - 1) << std::endl;
  X_4.row(buffer_len_ - 1).segment(0, 3) = vec_joined.segment(0, 3);
  X_4.row(buffer_len_ - 1).segment(3, 3) = vec_joined.segment(12, 3);
  X_4.row(buffer_len_ - 1).segment(6, 5) = input.segment(0, 5);
  X_4.row(buffer_len_ - 1).segment(11, 1) = VectorXd::Ones(1);
  // std::cout << "X_4 last row:\n" << X_4.row(buffer_len_ - 1) << std::endl;

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
  Eigen::VectorXd diff = ydot - (modes_.at(stance_mode).dynamics.A * prev_state_ + modes_.at(stance_mode).dynamics.B * prev_input_ +
      modes_.at(stance_mode).dynamics.b);
  // std::cout << "prev_state:" << prev_state_ << std::endl;
  // std::cout << "prev_input:" << prev_input_ << std::endl;
  // std::cout << "nominal derivative: " << nominal_deriv << std::endl;
  y_1.row(buffer_len_ - 2) = diff.segment(0, 3);
  y_2.row(buffer_len_ - 2) = diff.segment(3, 3);
  y_3.row(buffer_len_ - 2) = diff.segment(6, 3);
  y_4.row(buffer_len_ - 2) = diff.segment(9, 3);

  // std::cout << "last row of y:" << std::endl << y_.row(buffer_len_ - 2)<< std::endl;
  // if (ofs_) {
  //   ofs_ << "#" << std::endl;
  //  ofs_ << X_ << std::endl;
  //  ofs_ << "#" << std::endl;
  //  ofs_ << y_ << std::endl;
  // }
  /*
  if (ticks_ >= buffer_len_) {
    // debugging step: check if the dynamics of the past state + residual dynamics of past state are close to this state.
    Eigen::VectorXd exp_deriv = modes_.at(stance_mode).dynamics.A * prev_state_ + modes_.at(stance_mode).dynamics.B * prev_input_ +
        modes_.at(stance_mode).dynamics.b;
    Eigen::VectorXd res_state = cur_A_hat_ * prev_state_ + cur_B_hat_ * prev_input_ + cur_b_hat_;
    std::cout << "-----------------" << std::endl;
    // std::cout << "prev_state:" << prev_state_ << std::endl;
    // std::cout << "prev_input:" << prev_input_ << std::endl;
    std::cout << "nominal derivative: " << exp_deriv << std::endl;
    std::cout << "residual: " << res_state << std::endl;
    std::cout << "actual next deriv: " << ydot << std::endl;
    std::cout << "nominal + residual: " << exp_deriv + res_state << std::endl;
    std::cout << "-----------------" << std::endl;
  }
  */
  prev_state_ = vec_joined;
  prev_input_ = input;
}

void SRBDSparseResidualEstimator::SolveLstSq() const {
  // Solve the least squares equation, excluding the last row of X and y because it is always incomplete.
  Eigen::MatrixXd X_c1 = X_1.block(0, 0, buffer_len_ - 1, X_cols[0]);
  Eigen::MatrixXd y_c1 = y_1.block(0, 0, buffer_len_ - 1, state_block_size);
  Eigen::MatrixXd A1 = (X_c1.transpose() * X_c1).colPivHouseholderQr().solve(X_c1.transpose() * y_c1).transpose();

  Eigen::MatrixXd X_c2 = X_2.block(0, 0, buffer_len_ - 1, X_cols[1]);
  Eigen::MatrixXd y_c2 = y_2.block(0, 0, buffer_len_ - 1, state_block_size);
  Eigen::MatrixXd A2 = (X_c2.transpose() * X_c2).colPivHouseholderQr().solve(X_c2.transpose() * y_c2).transpose();

  MatrixXd reg3 = trans_reg_ * MatrixXd::Identity(X_cols[2], X_cols[2]);
  Eigen::MatrixXd X_c3 = X_3.block(0, 0, buffer_len_ - 1, X_cols[2]);
  Eigen::MatrixXd y_c3 = y_3.block(0, 0, buffer_len_ - 1, state_block_size);
  Eigen::MatrixXd soln3 = (X_c3.transpose() * X_c3 + reg3).colPivHouseholderQr().solve(X_c3.transpose() * y_c3).transpose();
  // Slice out the appropriate parts of the solution
  Eigen::MatrixXd B1 = soln3.block(0, 0, 3, 3);
  Eigen::MatrixXd b1 = soln3.block(0, 3, 3, 1);

  MatrixXd reg4 = rot_reg_ * MatrixXd::Identity(X_cols[3], X_cols[3]);
  Eigen::MatrixXd X_c4 = X_3.block(0, 0, buffer_len_ - 1, X_cols[3]);
  Eigen::MatrixXd y_c4 = y_3.block(0, 0, buffer_len_ - 1, state_block_size);
  Eigen::MatrixXd soln4 = (X_c4.transpose() * X_c4 + reg4).colPivHouseholderQr().solve(X_c4.transpose() * y_c4).transpose();

  MatrixXd A3 = soln4.block(0, 0, 3, 3);
  MatrixXd A4 = soln4.block(0, 3, 3, 3);
  MatrixXd B2 = soln4.block(0, 6, 3, 3);
  MatrixXd B3 = soln4.block(0, 9, 3, 1);
  MatrixXd B4 = soln4.block(0, 10, 3, 1);
  MatrixXd b2 = soln4.block(0, 11, 3, 1);

  // Set the hat matrices with the appropriate blocks matrices.
  cur_A_hat_.block(0, 6, 3, 3) = A1;
  cur_A_hat_.block(3, 9, 3, 3) = A2;
  cur_A_hat_.block(9, 0, 3, 3) = A3;
  cur_A_hat_.block(9, 12, 3, 3) = A4;

  cur_B_hat_.block(6, 0, 3, 3) = B1;
  cur_B_hat_.block(9, 0, 3, 3) = B2;
  cur_B_hat_.block(9, 3, 3, 1) = B3;
  cur_B_hat_.block(9, 4, 3, 1) = B4;

  cur_b_hat_.segment(6, 3) = b1;
  cur_b_hat_.segment(9, 3) = b2;

  std::cout << "cur A hat: \n" << cur_A_hat_ << std::endl;
  std::cout << "cur B hat: \n" << cur_B_hat_ << std::endl;
  std::cout << "cur b hat: \n" << cur_b_hat_ << std::endl;

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
//
// Created by brian on 12/19/21.
//

