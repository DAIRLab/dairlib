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
using Eigen::RowVectorXd;
using Eigen::Quaterniond;
using c3::LCS;
using c3::LCSFactory;
using dairlib::lcmt_timestamped_saved_traj;
using std::vector;
using systems::TimestampedVector;

namespace systems {

iC3HybridMpcTrackingController::iC3HybridMpcTrackingController(
    const drake::multibody::MultibodyPlant<double>& plant, const LCSFactory lcs_factory,
      HybridMpcOptions mpc_options, drake::solvers::SolverOptions solver_options, iC3Options ic3_options, 
      int example_idx, MatrixXd A_x, VectorXd lb_x, VectorXd ub_x, MatrixXd A_u, VectorXd lb_u, VectorXd ub_u)
    : plant_(plant),
      lcs_factory_(lcs_factory),
      mpc_options_(mpc_options),
      ic3_options_(ic3_options),
      solver_options_(solver_options),
      N_(mpc_options.lcs_factory_options.N),
      dt_(mpc_options.lcs_factory_options.dt),
      example_idx_(example_idx),
      Q_(mpc_options.Q),
      R_(mpc_options.R),
      S_(mpc_options.S),
      G_(mpc_options.G),
      lambda_threshold_(mpc_options.lambda_threshold),
      eta_threshold_(mpc_options.eta_threshold),
      A_x_(A_x),
      lb_x_(lb_x),
      ub_x_(ub_x),
      A_u_(A_u),
      lb_u_(lb_u),
      ub_u_(ub_u),
      prog_(drake::solvers::MathematicalProgram()),
      osqp_(drake::solvers::OsqpSolver()) { 
  this->set_name("ic3_hybrid_mpc_tracking_controller");

  n_q_ = plant_.num_positions();
  n_v_ = plant_.num_velocities();
  n_u_ = plant_.num_actuators();
  n_x_ = n_q_ + n_v_;

  std::cout << "n_q: " << n_q_ << std::endl;
  std::cout << "n_v: " << n_v_ << std::endl;
  std::cout << "n_u: " << n_u_ << std::endl;
  std::cout << "n_x: " << n_x_ << std::endl;



  lcs_factory_.SetNewDt(dt_);

  n_lambda_ = c3::multibody::LCSFactory::GetNumContactVariables(mpc_options.lcs_factory_options);
  std::cout << "n lambda " << n_lambda_ << std::endl;

  // Add decision variables
  for (int i = 0; i < N_+1; i++) {
    x_.push_back(prog_.NewContinuousVariables(n_x_, "x_" + std::to_string(i)));
    if (i == N_) break;
    u_.push_back(prog_.NewContinuousVariables(n_u_, "u_" + std::to_string(i)));
    lambda_.push_back(prog_.NewContinuousVariables(n_lambda_, "lambda_" + std::to_string(i)));
    epsilon_.push_back(prog_.NewContinuousVariables(n_lambda_, "epsilon_" + std::to_string(i)));
  }

  // Add placeholder costs
  target_costs_.resize(N_ + 1);
  input_costs_.resize(N_);
  force_costs_.resize(N_);
  slack_costs_.resize(N_);
  for (int i = 0; i < N_+1; i++) {
    target_costs_[i] = 
        prog_.AddQuadraticCost(2*Q_, VectorXd::Zero(n_x_), x_[i]).evaluator().get();
    if (i == N_) break;
    input_costs_[i] = 
        prog_.AddQuadraticCost(2*R_, VectorXd::Zero(n_u_), u_[i]).evaluator().get();
    force_costs_[i] = 
        prog_.AddQuadraticCost(2*S_, VectorXd::Zero(n_lambda_), lambda_[i]).evaluator().get();
    slack_costs_[i] = 
        prog_.AddQuadraticCost(2*G_, VectorXd::Zero(n_lambda_), epsilon_[i]).evaluator().get();
  }

  // End effector acceleration cost
  // Assumes first n_u_ terms of velocity correspond to end effector
  double accel_cost = mpc_options_.accel_cost;
  for (int i = 0; i < N_; i++) {
    MatrixXd Q_accel = accel_cost * MatrixXd::Identity(2*n_u_, 2*n_u_);
    Q_accel.block(0, n_u_, n_u_, n_u_) = -accel_cost * MatrixXd::Identity(n_u_, n_u_);
    Q_accel.block(n_u_, 0, n_u_, n_u_) = -accel_cost * MatrixXd::Identity(n_u_, n_u_);
    prog_.AddQuadraticCost(2*Q_accel, VectorXd::Zero(2*n_u_), {x_[i].segment(n_q_, n_u_), x_[i+1].segment(n_q_, n_u_)});
  }


  // Placeholder initial state constraint
  initial_state_constraint_ = 
      prog_.AddLinearEqualityConstraint(
          MatrixXd::Identity(n_x_, n_x_), VectorXd::Zero(n_x_), x_.at(0)
      ).evaluator().get();

  // Placeholder dynamics constraint
  dynamics_constraints_.resize(N_);
  MatrixXd A_dyn(MatrixXd::Zero(n_x_, n_x_ + n_u_ + n_lambda_ + n_x_));
  for (int i = 0; i < N_; i++) {
    dynamics_constraints_[i] = 
        prog_.AddLinearEqualityConstraint(
            A_dyn, VectorXd::Zero(n_x_), {x_.at(i), u_.at(i), lambda_.at(i), x_.at(i+1)}
        ).evaluator().get();
  }

  // Placeholder mode constraint
  lambda_constraints_.resize(N_);
  // Need 2 constraints for each [λ ε]
  MatrixXd A_lambda(MatrixXd::Zero(2 * n_lambda_, 2 * n_lambda_));
  for (int i = 0; i < N_; i++) {
    lambda_constraints_[i] = 
        prog_.AddLinearConstraint(
            A_lambda, VectorXd::Zero(2 * n_lambda_), VectorXd::Zero(2 * n_lambda_), {lambda_.at(i), epsilon_.at(i)}
        ).evaluator().get();
  }

  eta_constraints_.resize(N_);
  // Need 2 constraints for each [η ε]
  MatrixXd A_eta(MatrixXd::Zero(2 * n_lambda_, n_x_ + n_lambda_ + n_u_ + n_lambda_));
  for (int i = 0; i < N_; i++) {
    eta_constraints_[i] = 
        prog_.AddLinearConstraint(
            A_eta, VectorXd::Zero(2 * n_lambda_), VectorXd::Zero(2 * n_lambda_), {x_.at(i), lambda_.at(i), u_.at(i), epsilon_.at(i)}
        ).evaluator().get();
  }

  // Ensure epsilon is greater than 0
  for (int i = 0; i < N_; i++) {
    prog_.AddBoundingBoxConstraint(0, std::numeric_limits<double>::infinity(), epsilon_[i]);
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

  X_delta_ = drake::math::RigidTransform<double>::Identity();
  solve_time_ = 0;

  auto lcs_placeholder = CreatePlaceholderLCS();

  auto x_desired_placeholder =
      std::vector<VectorXd>(N_ + 1, VectorXd::Zero(n_x_));

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
      
  auto solution = c3::systems::C3Output::C3Solution();
  solution.x_sol_ = MatrixXf::Zero(n_q_ + n_v_, N_);
  solution.lambda_sol_ = MatrixXf::Zero(n_lambda_, N_);
  solution.u_sol_ = MatrixXf::Zero(n_u_, N_);
  solution.time_vector_ = VectorXf::Zero(N_);

  solution_port_ =
    this->DeclareAbstractOutputPort("solution_port", solution,
                                          &iC3HybridMpcTrackingController::OutputSolution)
              .get_index();



  int nominal_position_size;
  if (example_idx_ == 0) {
    nominal_position_size = 3;
  } else if (example_idx_ == 1 || example_idx_ == 2) {
    nominal_position_size = 9;
  }
  nominal_position_port_ =
      this->DeclareVectorInputPort(
              "nominal_position", BasicVector<double>(nominal_position_size))
          .get_index();

  plan_start_time_index_ = DeclareDiscreteState(1);
  filtered_solve_time_index_ = DeclareDiscreteState(1);

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

  std::cout << "ic3 timestep " << ic3_timestep << std::endl;

  // Get iC3 plan
  if (state_data_.cols() == 0 && input_data_.cols() == 0 && force_data_.cols() == 0) {
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

    state_data_ = state_trajectory.datapoints;
    input_data_ = input_trajectory.datapoints;
    force_data_ = force_trajectory.datapoints;

    lcs_ = MakeTimeVaryingLCS(state_data_, input_data_);

  }    
  // If in teleop or waiting, don't solve
  if (ic3_timestep < 0 || ic3_timestep >= ic3_options_.N) return drake::systems::EventStatus::Succeeded();

  
  auto start = std::chrono::high_resolution_clock::now();

  const TimestampedVector<double>* lcs_x =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        lcs_state_input_port_);
  drake::VectorX<double> x_lcs = lcs_x->get_data();  
  discrete_state->get_mutable_value(plan_start_time_index_)[0] =
      lcs_x->get_timestamp();

  for (int quat_idx : mpc_options_.quaternion_indices) {
    x_lcs.segment(quat_idx, 4) = x_lcs.segment(quat_idx, 4).normalized();
  }

  if (example_idx_ == 0) {
    const BasicVector<double>* nominal_position =
      (BasicVector<double>*)this->EvalVectorInput(context, nominal_position_port_);
    x_lcs.segment(0, 3) -= nominal_position->get_value();
  }
  std::cout << "x lcs " << x_lcs.transpose() << std::endl;

  // Update ee target based on current cube pose
  vector<VectorXd> x_des;
  vector<VectorXd> u_des;
  if (example_idx_ == 1 || example_idx_ == 2) {
    if (ic3_timestep % mpc_options_.transform_update_frequency == 0) {
      VectorXd x_plan = state_data_.col(ic3_timestep);
      Eigen::Quaterniond cube_rot_plan(x_plan(9), x_plan(10), x_plan(11), x_plan(12));
      Eigen::Vector3d cube_pos_plan(x_plan.segment(13, 3));
      drake::math::RigidTransform<double> X_W_Nom(cube_rot_plan, cube_pos_plan);

      Eigen::Quaterniond cube_rot_curr(x_lcs(9), x_lcs(10), x_lcs(11), x_lcs(12));
      Eigen::Vector3d cube_pos_curr(x_lcs.segment(13, 3));
      drake::math::RigidTransform<double> X_W_Curr(cube_rot_curr, cube_pos_curr);

      X_delta_ = X_W_Curr * X_W_Nom.inverse();
    }

    for (int i = 0; i < N_+1; i++) {
      int idx = std::min(ic3_timestep + i, ic3_options_.N-1); 
      VectorXd x_plan = state_data_.col(idx);
      VectorXd ee_x(9);
      for (int f = 0; f < 3; f++) {
        ee_x.segment(3*f, 3) = X_delta_ * x_plan.segment(3*f, 3);
      }
      x_plan.segment(0, 9) = ee_x;
      x_des.push_back(x_plan);

      if (i == N_) break;
      VectorXd u_plan = input_data_.col(idx);
      VectorXd ee_u(9);
      for (int f = 0; f < 3; f++) {
        ee_u.segment(3*f, 3) = X_delta_.rotation() * u_plan.segment(3*f, 3);
      }
      u_plan.segment(0, 9) = ee_u;
      u_des.push_back(u_plan);
    }
  } else if (example_idx_ == 0) {
    for (int i = 0; i < N_+1; i++) {
      int idx = std::min(ic3_timestep + i, ic3_options_.N-1); 
      x_des.push_back(state_data_.col(idx));

      if (i == N_) break;
      u_des.push_back(input_data_.col(idx));
    }
  }



  LCS lcs;
  if (mpc_options_.use_nominal_lcs) {
    lcs = GetLCSSegment(ic3_timestep, N_);
  } else {
    lcs_factory_.UpdateStateAndInput(x_lcs, u_des[0]);
    lcs = lcs_factory_.GenerateLCS();  
  }




  auto start_quat_norm = std::chrono::high_resolution_clock::now();

  // Get quaternion norms
  // TODO: right now assumes 1 quaternion body
  vector<double> quat_norms;
  quat_norms.push_back(x_lcs.segment(mpc_options_.quaternion_indices[0], 4).norm());

  c3::LCSSimulateConfig config; 
  config.regularized = true;
  config.min_exp = -16;
  config.max_exp = -6;

  VectorXd x_curr = x_lcs;
  for (int i = 0; i < N_; i++) {
    int idx = std::min(ic3_options_.N-1, ic3_timestep + i);
    VectorXd u_nominal = u_des[i];
    auto [x_out, lambda] = lcs.SimulateAndReturnForceAtTimestep(x_curr, u_nominal, config, i);
    x_curr = x_out;
    quat_norms.push_back(x_curr.segment(mpc_options_.quaternion_indices[0], 4).norm());
  }

  auto finish_quat_norm = std::chrono::high_resolution_clock::now();
  auto elapsed_quat_norm = finish_quat_norm - start_quat_norm;
  double solve_time_quat_norm =
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed_quat_norm).count() /
      1e6;
  std::cout << "Quat norm time " << solve_time_quat_norm << std::endl;


  // Update initial state constraint
  initial_state_constraint_->UpdateCoefficients(MatrixXd::Identity(n_x_, n_x_), x_lcs);
  std::cout << "x lcs " << x_lcs.transpose() << std::endl;

  // Update dynamics constraint
  // x_next = Ax + Bu + Dλ + d
  // x_k, u_k, lambda_k, x_{k+1}
  for (int i = 0; i < N_; i++) {
    MatrixXd A_dyn(n_x_, n_x_ + n_u_ + n_lambda_ + n_x_);
    A_dyn.block(0, 0, n_x_, n_x_) = lcs.A()[i];
    A_dyn.block(0, n_x_, n_x_, n_u_) = lcs.B()[i];
    A_dyn.block(0, n_x_+n_u_, n_x_, n_lambda_) = lcs.D()[i];
    A_dyn.block(0, n_x_+n_u_+n_lambda_, n_x_, n_x_) = -1 * MatrixXd::Identity(n_x_, n_x_);

    if (A_dyn.array().isNaN().any()) {
      std::cout << "A_dyn has NAN " << std::endl;
    }
    
    VectorXd affine_term = -lcs.d()[i];
    dynamics_constraints_[i]->UpdateCoefficients(A_dyn, affine_term);
  }

  std::cout << "after update dynamics constraints " << std::endl;

  /* Update lambda/eta constraints
    -ε <= λ <= ε if λ_hat = 0 
    equivalent to [0 ] <= [1  1] [λ] <= [∞]
                  [-∞]    [1 -1] [ε]    [0]
    
    -ε <= λ <= ∞ if λ_hat > 0 
    equivalent to [0] <= [1 1] [λ] <= [∞]
                  [0]    [0 0] [ε]    [0]
  */
  for (int i = 0; i < N_; i++) {
    int idx = std::min(ic3_options_.N-1, ic3_timestep + i);
    VectorXd lambda_hat = force_data_.col(idx);

    MatrixXd E = lcs.E()[i];
    MatrixXd F = lcs.F()[i];
    MatrixXd H = lcs.H()[i];
    VectorXd c = lcs.c()[i];

    // row j and j+n_lambda correspond to the same component of lambda/eta
    VectorXd lambda_lb(2 * n_lambda_);
    VectorXd eta_lb(2 * n_lambda_);
    VectorXd lambda_ub(2 * n_lambda_);
    VectorXd eta_ub(2 * n_lambda_);
    MatrixXd A_lambda(MatrixXd::Zero(2 * n_lambda_, 2 * n_lambda_));
    MatrixXd A_eta(MatrixXd::Zero(2 * n_lambda_, n_x_ + n_lambda_ + n_u_ + n_lambda_));

    A_lambda.block(0, 0, n_lambda_, n_lambda_) = MatrixXd::Identity(n_lambda_, n_lambda_);
    A_lambda.block(0, n_lambda_, n_lambda_, n_lambda_) = MatrixXd::Identity(n_lambda_, n_lambda_);

    A_eta.block(0, 0, n_lambda_, n_x_) = E;
    A_eta.block(0, n_x_, n_lambda_, n_lambda_) = F;
    A_eta.block(0, n_x_+n_lambda_, n_lambda_, n_u_) = H;
    A_eta.block(0, n_x_+n_lambda_+n_u_, n_lambda_, n_lambda_) = MatrixXd::Identity(n_lambda_, n_lambda_);

    lambda_ub.segment(0, n_lambda_) = VectorXd::Constant(n_lambda_, std::numeric_limits<double>::infinity());
    lambda_ub.segment(n_lambda_, n_lambda_) = VectorXd::Zero(n_lambda_);

    eta_ub.segment(0, n_lambda_) = VectorXd::Constant(n_lambda_, std::numeric_limits<double>::infinity());

    double tolerance = 1e-5;
    for (int j = 0; j < n_lambda_; j++) {
      lambda_lb(j) = (lambda_hat(j) <= tolerance) ? 0 : lambda_threshold_(j);
      lambda_lb(n_lambda_+j) = (lambda_hat(j) <= tolerance) ? -std::numeric_limits<double>::infinity() : 0;

      eta_lb(j) = -c(j) + ((lambda_hat(j) > tolerance) ? 0 : eta_threshold_(j));
      eta_lb(n_lambda_+j) = (lambda_hat(j) > tolerance) ? -std::numeric_limits<double>::infinity() : 0;
      eta_ub(n_lambda_+j) = (lambda_hat(j) > tolerance) ? -c(j) : 0;

      A_lambda(n_lambda_+j, j) = (lambda_hat(j) <= tolerance) ? 1 : 0;
      A_lambda(n_lambda_+j, n_lambda_+j) = (lambda_hat(j) <= tolerance) ? -1 : 0;    
      
      A_eta.block(n_lambda_+j, 0, 1, n_x_) = 
          (lambda_hat(j) > tolerance) ? E.row(j) : RowVectorXd(RowVectorXd::Zero(n_x_));
      A_eta.block(n_lambda_+j, n_x_, 1, n_lambda_) = 
          (lambda_hat(j) > tolerance) ? F.row(j) : RowVectorXd(RowVectorXd::Zero(n_lambda_));
      A_eta.block(n_lambda_+j, n_x_+n_lambda_, 1, n_u_) = 
          (lambda_hat(j) > tolerance) ? H.row(j) : RowVectorXd(RowVectorXd::Zero(n_u_));

      RowVectorXd eta_row = RowVectorXd::Zero(n_lambda_);
      eta_row(j) = (lambda_hat(j) > tolerance) ? -1 : 0;    
      A_eta.block(n_lambda_+j, n_x_+n_lambda_+n_u_, 1, n_lambda_) = 
          (lambda_hat(j) > tolerance) ? eta_row : RowVectorXd(RowVectorXd::Zero(n_lambda_));
    }


    lambda_constraints_[i]->UpdateCoefficients(A_lambda, lambda_lb, lambda_ub);
    eta_constraints_[i]->UpdateCoefficients(A_eta, eta_lb, eta_ub);
  }

  std::cout << "after update lambda/eta constraints " << std::endl;

  // Update tracking and input targets to match plan 
  UpdateQuaternionCosts(x_lcs, x_des[N_]);

  std::cout << "after update quat costs " << std::endl;
  // Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver_Q(Q_);
  // Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver_R(R_);

  // std::cout << "Q min eigenvalue " << solver_Q.eigenvalues().minCoeff() << std::endl;
  // std::cout << "R min eigenvalue " << solver_R.eigenvalues().minCoeff() << std::endl;

  for (int i = 0; i < N_+1; i++) {
    int idx = std::min(ic3_options_.N-1, ic3_timestep + i);

    VectorXd xd = x_des[i];
    xd.segment(mpc_options_.quaternion_indices[0], 4) = 
        quat_norms[i] * xd.segment(mpc_options_.quaternion_indices[0], 4);
    target_costs_[i]->UpdateCoefficients(2 * Q_, -2 * Q_ * xd);

    if (i == N_) break;

    VectorXd ud = u_des[i];
    input_costs_[i]->UpdateCoefficients(2 * R_, -2 * R_ * ud);

    VectorXd lambda_des = force_data_.col(idx);
    force_costs_[i]->UpdateCoefficients(2 * S_, -2 * S_ * lambda_des);
  }

  std::cout << "Q_ " << Q_.rows() << " x " << Q_.cols() << std::endl;
  std::cout << "R_ " << R_.rows() << " x " << R_.cols() << std::endl;
  std::cout << "S_ " << S_.rows() << " x " << S_.cols() << std::endl;
  std::cout << "G_ " << G_.rows() << " x " << G_.cols() << std::endl;

  auto start_qp_solve = std::chrono::high_resolution_clock::now();

  // Solve QP and extract results
  drake::solvers::MathematicalProgramResult result = osqp_.Solve(prog_, std::nullopt, solver_options_);;

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
    while (true) {}
  }

  solve_time_ = solve_time_qp_solve;
  x_sol_.clear();
  u_sol_.clear();
  lambda_sol_.clear();
  for (int i = 0; i < N_; i++) {
    x_sol_.push_back(result.GetSolution(x_[i]));
    u_sol_.push_back(result.GetSolution(u_[i]));
    lambda_sol_.push_back(result.GetSolution(lambda_[i]));
  }

  if (example_idx_ == 0) {
    for (int i = 0; i < N_; i++) {
      std::cout << "epsilon " << result.GetSolution(epsilon_[i]).transpose() << std::endl;
    }
  } else if (example_idx_ == 1 || example_idx_ == 2) {
    // for (int i = 0; i < N_; i++) {
    //   VectorXd x_pred = result.GetSolution(x_[i]);
    //   VectorXd u_pred = result.GetSolution(u_[i]);
    //   VectorXd lambda_pred = result.GetSolution(lambda_[i]);
    //   VectorXd epsilon_pred = result.GetSolution(epsilon_[i]);
    //   VectorXd eta_pred = lcs.E()[i] * x_pred + lcs.F()[i] * lambda_pred + lcs.H()[i] * u_pred + lcs.c()[i];

    //   std::cout << "x " << i << ": " << x_pred.segment(0, 16).transpose() << std::endl;
    //   std::cout << "u " << i << ": " << u_pred.transpose() << std::endl;
    //   std::cout << "lambda " << i << ": " << lambda_pred.segment(0, 12).transpose() << std::endl;
    //   std::cout << "eta " << i << ": " << eta_pred.segment(0, 12).transpose() << std::endl;
    //   std::cout << "epsilon " << i << ": " << epsilon_pred.transpose() << std::endl;

    //   double quat_cost = x_pred.segment(9, 4).transpose() * Q_.block(9, 9, 4, 4) * x_pred.segment(9, 4);
    //   std::cout << "quat cost " << quat_cost << std::endl;

    //   double finger_cost = x_pred.segment(0, 9).transpose() * Q_.block(0, 0, 9, 9) * x_pred.segment(0, 9);
    //   std::cout << "finger cost " << finger_cost << std::endl;

    //   double cube_pos = x_pred.segment(13, 3).transpose() * Q_.block(13, 13, 3, 3) * x_pred.segment(13, 3);
    //   std::cout << "cube pos cost " << cube_pos << std::endl;

    //   double u_cost = u_pred.transpose() * R_ * u_pred;
    //   std::cout << "u cost " << u_cost << std::endl;

    //   double lambda_cost = lambda_pred.transpose() * S_ * lambda_pred;
    //   std::cout << "lambda cost " << lambda_cost << std::endl;

    //   double epsilon_cost = epsilon_pred.transpose() * G_ * epsilon_pred;
    //   std::cout << "epsilon cost " << epsilon_cost << std::endl;

    //   std::cout << std::endl;
    // }
  }


  return drake::systems::EventStatus::Succeeded();
}


void iC3HybridMpcTrackingController::OutputSolution(
    const drake::systems::Context<double>& context,
    c3::systems::C3Output::C3Solution* solution) const {

  if (x_sol_.size() == 0) return;

  double t = context.get_discrete_state(plan_start_time_index_)[0];
  std::cout << "output solution t " << t << std::endl;

  for (int i = 0; i < N_; i++) {
    solution->time_vector_(i) = solve_time_ + t + i * dt_;
    solution->x_sol_.col(i) = x_sol_[i].cast<float>();
    solution->lambda_sol_.col(i) =
        lambda_sol_[i].cast<float>();
    solution->u_sol_.col(i) =
        u_sol_[i].cast<float>();
  } 
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


LCS iC3HybridMpcTrackingController::GetLCSSegment(int start_idx, int size) const {
  DRAKE_DEMAND(lcs_.N() == ic3_options_.N);

  std::vector<Eigen::MatrixXd> A;
  std::vector<Eigen::MatrixXd> B;
  std::vector<Eigen::MatrixXd> D;
  std::vector<Eigen::VectorXd> d;
  std::vector<Eigen::MatrixXd> E;
  std::vector<Eigen::MatrixXd> F;
  std::vector<Eigen::MatrixXd> H;
  std::vector<Eigen::VectorXd> c;

  for (int i = 0; i < size; i++) {
    int idx = std::min(ic3_options_.N-1, start_idx + i);
    A.push_back(lcs_.A()[idx]);
    B.push_back(lcs_.B()[idx]);
    D.push_back(lcs_.D()[idx]);
    d.push_back(lcs_.d()[idx]);
    E.push_back(lcs_.E()[idx]);
    F.push_back(lcs_.F()[idx]);
    H.push_back(lcs_.H()[idx]);
    c.push_back(lcs_.c()[idx]);
  }

  return LCS(A, B, D, d, E, F, H, c, dt_);
}

}  // namespace systems
}  // namespace dairlib
