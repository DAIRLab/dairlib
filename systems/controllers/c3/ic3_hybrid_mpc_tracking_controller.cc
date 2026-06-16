#include "ic3_hybrid_mpc_tracking_controller.h"
#include <Eigen/Dense>

#include <iostream>
#include <c3/core/lcs.h>
#include <c3/multibody/lcs_factory.h>
#include "c3/systems/common/quaternion_error_hessian.h"
#include "drake/common/text_logging.h"

#include "drake/solvers/solve.h"

namespace dairlib {

using drake::multibody::ModelInstanceIndex;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Quaterniond;
using c3::LCS;
using c3::LCSFactory;
using dairlib::lcmt_timestamped_saved_traj;
using std::vector;
using systems::TimestampedVector;

namespace systems {

iC3HybridMpcTrackingController::iC3HybridMpcTrackingController(
    const drake::multibody::MultibodyPlant<double>& plant, const LCSFactory lcs_factory,
      HybridMpcOptions mpc_options, iC3Options ic3_options, int example_idx, 
      MatrixXd A_x, VectorXd lb_x, VectorXd ub_x, MatrixXd A_u, VectorXd lb_u, VectorXd ub_u)
    : plant_(plant),
      lcs_factory_(lcs_factory),
      mpc_options_(mpc_options),
      ic3_options_(ic3_options),
      N_(mpc_options.N),
      dt_(ic3_options_.dt),
      example_idx_(example_idx),
      Q_(mpc_options.Q),
      R_(mpc_options.R),
      A_x_(A_x),
      lb_x_(lb_x),
      ub_x_(ub_x),
      A_u_(A_u),
      lb_u_(lb_u),
      ub_u_(ub_u),
      prog_(drake::solvers::MathematicalProgram()),
      osqp_(drake::solvers::OsqpSolver()) { 
  this->set_name("ic3_lqr_tracking_controller");

  n_q_ = plant_.num_positions();
  n_v_ = plant_.num_velocities();
  n_u_ = plant_.num_actuators();
  n_x_ = n_q_ + n_v_;

  std::cout << "n_q: " << n_q_ << std::endl;
  std::cout << "n_v: " << n_v_ << std::endl;
  std::cout << "n_u: " << n_u_ << std::endl;
  std::cout << "n_x: " << n_x_ << std::endl;

  lcs_factory_.SetNewDt(dt_);

  // TODO: make this not bad
  if (example_idx_ == 0) {
    n_lambda_ = 4 * 4;
  } else if (example_idx_ == 1) {
    n_lambda_ = 7 * 4;
  } else if (example_idx_ == 2) {
    n_lambda_ = 11 * 4;
  }

  // Add decision variables
  for (int i = 0; i < N_+1; i++) {
    x_.push_back(prog_.NewContinuousVariables(n_x_, "x" + std::to_string(i)));
    if (i == N_) break;
    u_.push_back(prog_.NewContinuousVariables(n_u_, "u" + std::to_string(i)));
  }

  // Add placeholder costs
  target_costs_.resize(N_ + 1);
  input_costs_.resize(N_);
  for (int i = 0; i < N_+1; i++) {
    target_costs_[i] = 
        prog_.AddQuadraticCost(2*Q_, VectorXd::Zero(n_x_), x_[i]).evaluator().get();
    if (i == N_) break;
    input_costs_[i] = 
        prog_.AddQuadraticCost(2*R_, VectorXd::Zero(n_u_), u_[i]).evaluator().get();
  }

  // Placeholder initial state constraint
  initial_state_constraint_ = 
      prog_.AddLinearEqualityConstraint(
          MatrixXd::Identity(n_x_, n_x_), VectorXd::Zero(n_x_), x_.at(0)
      ).evaluator().get();

  // Placeholder dynamics constraint
  dynamics_constraints_.resize(N_);
  MatrixXd LinEq(MatrixXd::Zero(n_x_, n_x_ + n_u_ + n_x_));
  for (int i = 0; i < N_; i++) {
    dynamics_constraints_[i] = 
        prog_.AddLinearEqualityConstraint(
            LinEq, VectorXd::Zero(n_x_), {x_.at(i), u_.at(i), x_.at(i+1)}
        ).evaluator().get();
  }

  // Add linear constraints
  if (ic3_options_.add_position_constraints) {
    for (int i = 1; i < N_ + 1; i++) { // Don't put constraint on x0
      prog_.AddLinearConstraint(A_x_, lb_x_, ub_x_, x_.at(i)); 
    }
    std::cout << "A_x " << A_x_.diagonal().transpose() << std::endl;
    std::cout << "lb_x " << lb_x_.transpose() << std::endl;
    std::cout << "ub_x " << ub_x_.transpose() << std::endl;
  }
  if (ic3_options_.add_input_constraints) {
    for (int i = 0; i < N_; i++) {
      prog_.AddLinearConstraint(A_u_, lb_u_, ub_u_, u_.at(i));
    }
    std::cout << "A_u " << A_u_.diagonal().transpose() << std::endl;
    std::cout << "lb_u " << lb_u_.transpose() << std::endl;
    std::cout << "ub_u " << ub_u_.transpose() << std::endl;
  }



  auto lcs_placeholder = CreatePlaceholderLCS();

  auto x_desired_placeholder =
      std::vector<VectorXd>(N_ + 1, VectorXd::Zero(n_x_));

  tracking_target_ = VectorXd::Zero(n_u_);
  u_out_ = VectorXd::Zero(n_u_);

  lcs_state_input_port_ =
      this->DeclareVectorInputPort("x_lcs", TimestampedVector<double>(n_x_))
          .get_index();

  ic3_x_port_ =
      this->DeclareAbstractInputPort("ic3_x_port", drake::Value<lcmt_timestamped_saved_traj>())
          .get_index();

  ic3_u_port_ =
      this->DeclareAbstractInputPort("ic3_u_port", drake::Value<lcmt_timestamped_saved_traj>())
          .get_index();

  ic3_lambda_port_ =
      this->DeclareAbstractInputPort("ic3_lambda_port", drake::Value<lcmt_timestamped_saved_traj>())
          .get_index();

  timestep_port_ =  
      this->DeclareVectorInputPort("timestep_port", 1).get_index();
      

  actor_port_ =
      this->DeclareVectorOutputPort("actor_input", BasicVector<double>(n_u_),
                                      &iC3HybridMpcTrackingController::OutputActorInput)
          .get_index();

  tracking_target_port_ =
      this->DeclareVectorOutputPort("tracking_target_port", BasicVector<double>(n_u_),
                                  &iC3HybridMpcTrackingController::OutputTrackingTarget)
          .get_index();

  DeclareForcedDiscreteUpdateEvent(&iC3HybridMpcTrackingController::ComputePlan);

}

LCS iC3HybridMpcTrackingController::CreatePlaceholderLCS() const {
  MatrixXd A = MatrixXd::Ones(n_x_, n_x_);
  MatrixXd B = MatrixXd::Zero(n_x_, n_u_);
  VectorXd d = VectorXd::Zero(n_x_);
  MatrixXd D = MatrixXd::Ones(n_x_, n_lambda_);
  MatrixXd E = MatrixXd::Zero(n_lambda_, n_x_);
  MatrixXd F = MatrixXd::Zero(n_lambda_, n_lambda_);
  MatrixXd H = MatrixXd::Zero(n_lambda_, n_u_);
  VectorXd c = VectorXd::Zero(n_lambda_);
  return LCS(A, B, D, d, E, F, H, c, N_, ic3_options_.dt);
}

drake::systems::EventStatus iC3HybridMpcTrackingController::ComputePlan(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {

  const BasicVector<double>& timestep_vector =
      *this->template EvalVectorInput<BasicVector>(context, timestep_port_);
  int ic3_timestep = static_cast<int>(timestep_vector.get_value()(0));

  // If in teleop or waiting, don't solve
  if (ic3_timestep < 0 || ic3_timestep >= ic3_options_.N) return drake::systems::EventStatus::Succeeded();

  auto start = std::chrono::high_resolution_clock::now();

  std::cout << "ic3 timestep " << ic3_timestep << std::endl;

  const TimestampedVector<double>* lcs_x =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        lcs_state_input_port_);
  drake::VectorX<double> x_lcs = lcs_x->get_data();
  
  // Get iC3 plan
  const std::string trajectory_name = "iteration_" + std::to_string(ic3_options_.iter_to_use);
  const auto& lcm_all_x_trajectories = 
    this->EvalAbstractInput(context, ic3_x_port_)->get_value<lcmt_timestamped_saved_traj>();
  const auto& lcm_all_u_trajectories = 
    this->EvalAbstractInput(context, ic3_u_port_)->get_value<lcmt_timestamped_saved_traj>();
  const auto& lcm_all_lambda_trajectories = 
    this->EvalAbstractInput(context, ic3_lambda_port_)->get_value<lcmt_timestamped_saved_traj>();
  LcmTrajectory x_trajectory = LcmTrajectory(lcm_all_x_trajectories.saved_traj);
  LcmTrajectory u_trajectory = LcmTrajectory(lcm_all_u_trajectories.saved_traj);
  LcmTrajectory lambda_trajectory = LcmTrajectory(lcm_all_lambda_trajectories.saved_traj);
  LcmTrajectory::Trajectory state_trajectory = x_trajectory.GetTrajectory(trajectory_name);
  LcmTrajectory::Trajectory input_trajectory = u_trajectory.GetTrajectory(trajectory_name);
  LcmTrajectory::Trajectory force_trajectory = lambda_trajectory.GetTrajectory(trajectory_name);

  MatrixXd state_data = state_trajectory.datapoints;
  MatrixXd input_data = input_trajectory.datapoints;
  MatrixXd force_data = force_trajectory.datapoints;
      

  // Make LCS about nominal trajectory
  MatrixXd x_hat(n_x_, N_);
  MatrixXd u_hat(n_u_, N_);
  for (int i = 0; i < N_; i++) {
    int idx = std::min(ic3_options_.N-1, ic3_timestep + i);
    x_hat.col(i) = state_data.col(idx);
    u_hat.col(i) = input_data.col(idx);
  }
  LCS lcs = MakeTimeVaryingLCS(x_hat, u_hat);

  // Simulate to get lambda
  vector<VectorXd> lambdas;
  vector<double> quat_norms;

  // TODO: right now assumes 1 quaternion body
  quat_norms.push_back(x_lcs.segment(mpc_options_.quaternion_indices[0], 4).norm());

  VectorXd x_curr = x_lcs;
  for (int i = 0; i < N_; i++) {
    int idx = std::min(ic3_options_.N, ic3_timestep + i);
    VectorXd u_nominal = input_data.col(idx);

    lcs_factory_.UpdateStateAndInput(x_curr, u_nominal);
    LCS lcs_temp = lcs_factory_.GenerateLCS();
    
    c3::LCSSimulateConfig config; 
    config.regularized = true;
    config.max_exp = -6;
    auto [x_out, lambda] = lcs_temp.SimulateAndReturnForce(x_curr, u_nominal, config);
    x_curr = x_out;
    lambdas.push_back(lambda);
    quat_norms.push_back(x_curr.segment(mpc_options_.quaternion_indices[0], 4).norm());
    
    // lambdas.push_back(force_data.col(idx));
  }


  // Update initial state constraint
  initial_state_constraint_->UpdateCoefficients(MatrixXd::Identity(n_x_, n_x_), x_lcs);

  // Update dynamics constraint
  // x_k, u_k, x_{k+1}
  for (int i = 0; i < N_; i++) {
    MatrixXd LinEq(n_x_, n_x_ + n_u_ + n_x_);
    LinEq.block(0, 0, n_x_, n_x_) = lcs.A()[i];
    LinEq.block(0, n_x_, n_x_, n_u_) = lcs.B()[i];
    LinEq.block(0, n_x_+n_u_, n_x_, n_x_) = -1 * MatrixXd::Identity(n_x_, n_x_);

    if (LinEq.array().isNaN().any()) {
      std::cout << "LinEq has NAN " << std::endl;
    }
    
    std::cout << "affine term " << i << " " << (lcs.D()[i] * lambdas[i]).transpose() << std::endl;
    VectorXd affine_term = -(lcs.D()[i] * lambdas[i] + lcs.d()[i]);

    dynamics_constraints_[i]->UpdateCoefficients(LinEq, affine_term);
  }

  // Update tracking and input targets to match plan 
  UpdateQuaternionCosts(x_lcs, state_data.col(std::min(ic3_options_.N-1, ic3_timestep)));

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver_Q(Q_);
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver_R(R_);

  // std::cout << "Q min eigenvalue " << solver_Q.eigenvalues().minCoeff() << std::endl;
  // std::cout << "R min eigenvalue " << solver_R.eigenvalues().minCoeff() << std::endl;

  for (int i = 0; i < N_+1; i++) {
    int idx = std::min(ic3_options_.N-1, ic3_timestep + i);

    VectorXd x_des = state_data.col(idx);
    x_des.segment(mpc_options_.quaternion_indices[0], 4) = 
        quat_norms[i] * x_des.segment(mpc_options_.quaternion_indices[0], 4);
    target_costs_[i]->UpdateCoefficients(2 * Q_, -2 * Q_ * x_des);

    if (i == N_) break;

    VectorXd u_des = input_data.col(idx);
    input_costs_[i]->UpdateCoefficients(2 * R_, -2 * R_ * u_des);
  }

  auto start_qp_solve = std::chrono::high_resolution_clock::now();

  // Solve QP and extract results
  drake::solvers::MathematicalProgramResult result = osqp_.Solve(prog_);

  auto finish_qp_solve = std::chrono::high_resolution_clock::now();
  auto elapsed_qp_solve = finish_qp_solve - start_qp_solve;
  double solve_time_qp_solve =
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed_qp_solve).count() /
      1e6;
  std::cout << "QP solve time " << solve_time_qp_solve << std::endl;

  if (!result.is_success()) {
    const auto& details = result.get_solver_details<drake::solvers::OsqpSolver>();
    drake::log()->warn("OSQP Status: {}", details.status_val); 
    drake::log()->warn("Iterations: {}", details.iter);
    drake::log()->warn("Primal Res: {}", details.primal_res);
    drake::log()->warn("Dual Res: {}", details.dual_res);
  }

  u_out_ = result.GetSolution(u_[0]);

  for (int i = 0; i < N_; i++) {
    VectorXd x_pred = result.GetSolution(x_[i]);
    std::cout << "x " << i << ": " << x_pred.segment(0, 16).transpose() << std::endl;

    double quat_cost = x_pred.segment(9, 4).transpose() * Q_.block(9, 9, 4, 4) * x_pred.segment(9, 4);
    std::cout << "quat cost " << " " << quat_cost << std::endl;

    double finger_cost = x_pred.segment(0, 9).transpose() * Q_.block(0, 0, 9, 9) * x_pred.segment(0, 9);
    std::cout << "finger cost " << " " << finger_cost << std::endl;

    double cube_pos = x_pred.segment(13, 3).transpose() * Q_.block(13, 13, 3, 3) * x_pred.segment(13, 3);
    std::cout << "cube pos cost " << " " << cube_pos << std::endl;

    std::cout << std::endl;
  }


  return drake::systems::EventStatus::Succeeded();
}


void iC3HybridMpcTrackingController::OutputActorInput(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* actor_input) const {
  std::cout << "u_out output port " << u_out_.transpose() << std::endl;
	actor_input->get_mutable_value() = u_out_;
}


void iC3HybridMpcTrackingController::OutputTrackingTarget(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* tracking_target) const {
	tracking_target->get_mutable_value() = tracking_target_;
}


void iC3HybridMpcTrackingController::UpdateQuaternionCosts(
    const Eigen::VectorXd& x_curr, const Eigen::VectorXd& x_des) const {
    
  // Early return if no quaternions or cost parameters not set
  if (mpc_options_.quaternion_indices.size() == 0) {
    return;
  }

  for (int index : mpc_options_.quaternion_indices) {
    Eigen::VectorXd quat_curr_i = x_curr.segment(index, 4);
    Eigen::VectorXd quat_des_i = x_des.segment(index, 4);

    Eigen::MatrixXd quat_hessian_i =
        c3::systems::common::hessian_of_squared_quaternion_angle_difference(
                  quat_curr_i, quat_des_i);

    // Regularize hessian so Q is always PSD
    double min_eigenval = quat_hessian_i.eigenvalues().real().minCoeff();
    Eigen::MatrixXd quat_regularizer_1 =
        std::max(0.0, -min_eigenval) * Eigen::MatrixXd::Identity(4, 4);
    Eigen::MatrixXd quat_regularizer_2 = quat_des_i * quat_des_i.transpose();

    // Additional regularization term to help with numerical issues
    Eigen::MatrixXd quat_regularizer_3 = 1e-8 * Eigen::MatrixXd::Identity(4, 4);

    double quaternion_weight = mpc_options_.quaternion_weight;
    double quaternion_regularizer_fraction = 0.0;

    // Replace quaternion blocks in Q
    double discount_factor = 1;
    for (int i = 0; i < N_ + 1; i++) {
      Q_.block(index, index, 4, 4) =
          mpc_options_.w_Q * quaternion_weight *
          (quat_hessian_i + quat_regularizer_1 +
           quaternion_regularizer_fraction * quat_regularizer_2 +
           quat_regularizer_3);
    }
  }
}



LCS iC3HybridMpcTrackingController::MakeTimeVaryingLCS(MatrixXd x_hat, MatrixXd u_hat) const {
  DRAKE_DEMAND(x_hat.cols() >= u_hat.cols());

  vector<Eigen::MatrixXd> A;
  vector<Eigen::MatrixXd> B;
  vector<Eigen::MatrixXd> D;
  vector<Eigen::VectorXd> d;
  vector<Eigen::MatrixXd> E;
  vector<Eigen::MatrixXd> F;
  vector<Eigen::MatrixXd> H;
  vector<Eigen::VectorXd> c;

  int N = u_hat.cols();

  for (int k = 0; k < N; k++) {
    
    for (auto idx : mpc_options_.quaternion_indices) {
      x_hat.col(k).segment(idx, 4) = x_hat.col(k).segment(idx, 4).normalized(); // Normalize quaternions
    }

    // Linearize about kth xhat, uhat
    lcs_factory_.UpdateStateAndInput(x_hat.col(k), u_hat.col(k));
    LCS lcs = lcs_factory_.GenerateLCS();
    A.push_back(lcs.A()[0]);
    B.push_back(lcs.B()[0]);
    D.push_back(lcs.D()[0]);
    d.push_back(lcs.d()[0]);
    E.push_back(lcs.E()[0]);
    F.push_back(lcs.F()[0]);
    H.push_back(lcs.H()[0]);
    c.push_back(lcs.c()[0]);      
  }

  return LCS(A, B, D, d, E, F, H, c, dt_);
}

}  // namespace systems
}  // namespace dairlib
