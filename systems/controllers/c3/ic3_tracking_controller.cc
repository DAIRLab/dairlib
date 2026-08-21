#include "ic3_tracking_controller.h"
#include <Eigen/Dense>

#include <c3/core/c3_miqp.h>
#include <c3/core/c3_qp.h>
#include <c3/core/c3_plus.h>
#include "c3/systems/common/quaternion_error_hessian.h"
#include <iostream>

#include <c3/multibody/geom_geom_collider.h>
#include <c3/core/lcs.h>

namespace dairlib {

using drake::multibody::ModelInstanceIndex;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Quaterniond;
using c3::C3;
using c3::C3MIQP;
using c3::C3QP;
using c3::C3Plus;
using c3::LCS;
using c3::LCSFactory;
using c3::C3Options;
using c3::systems::C3ControllerOptions;
using c3::systems::C3Output;
using c3::LCSFactoryOptions;
using c3::ContactPairConfig;

using std::vector;
using systems::TimestampedVector;

namespace systems {

iC3TrackingController::iC3TrackingController(
    const drake::multibody::MultibodyPlant<double>& plant, const LCSFactory lcs_factory,
      C3ControllerOptions controller_options, iC3Options ic3_options, VectorXd x_des, int example_idx, 
      MatrixXd A_x, VectorXd lb_x, VectorXd ub_x, MatrixXd A_u, VectorXd lb_u, VectorXd ub_u)
    : plant_(plant),
      lcs_factory_(lcs_factory),
      controller_options_(controller_options),
      c3_options_(controller_options.c3_options),
      lcs_factory_options_(controller_options.lcs_factory_options),
      ic3_options_(ic3_options),
      x_des_(x_des),
      N_(lcs_factory_options_.N), 
      dt_(lcs_factory_options_.dt),
      example_idx_(example_idx),
      A_x_(A_x),
      lb_x_(lb_x),
      ub_x_(ub_x), 
      A_u_(A_u),
      lb_u_(lb_u),
      ub_u_(ub_u) { 
  this->set_name("ic3_tracking_controller");

  double discount_factor = 1;
  for (int i = 0; i < N_; ++i) {
    Q_.push_back(discount_factor * c3_options_.Q);
    R_.push_back(discount_factor * c3_options_.R);
    G_.push_back(discount_factor * c3_options_.G);
    U_.push_back(discount_factor * c3_options_.U);

    discount_factor *= c3_options_.gamma;
  }
  Q_.push_back(discount_factor * c3_options_.Q);
  DRAKE_DEMAND(Q_.size() == N_ + 1);
  DRAKE_DEMAND(R_.size() == N_);
  DRAKE_DEMAND(G_.size() == N_);
  DRAKE_DEMAND(U_.size() == N_);

  n_q_ = plant_.num_positions();
  n_v_ = plant_.num_velocities();
  n_u_ = plant_.num_actuators();
  n_x_ = n_q_ + n_v_;

  std::cout << "n_q: " << n_q_ << std::endl;
  std::cout << "n_v: " << n_v_ << std::endl;
  std::cout << "n_u: " << n_u_ << std::endl;
  std::cout << "n_x: " << n_x_ << std::endl;


  solve_time_filter_constant_ = controller_options_.solve_time_filter_alpha;

  lcs_factory_.SetNewDt(dt_);
  n_lambda_ = c3::multibody::LCSFactory::GetNumContactVariables(lcs_factory_options_);
  VectorXd zeros = VectorXd::Zero(n_x_ + n_lambda_ + n_u_);
  n_u_ = plant_.num_actuators();

  // Get quaternion bodies
  for (const auto& body_idx : plant_.GetFloatingBaseBodies()) {
    const auto& body = plant_.get_body(body_idx);
    int start = body.floating_positions_start();
    quaternion_indices_.push_back(start);
  }

  // Creates placeholder lcs to construct base C3 problem
  // Placeholder LCS will have correct size as it's already determined by the
  // contact model
  auto lcs_placeholder = CreatePlaceholderLCS();
  auto x_desired_placeholder =
      std::vector<VectorXd>(N_ + 1, VectorXd::Zero(n_x_));
  if (controller_options_.projection_type == "MIQP") {
    c3_ = std::make_unique<C3MIQP>(lcs_placeholder,
                                   C3::CostMatrices(Q_, R_, G_, U_),
                                   x_desired_placeholder, c3_options_);

  } else if (controller_options_.projection_type == "QP") {
    c3_ = std::make_unique<C3QP>(lcs_placeholder,
                                 C3::CostMatrices(Q_, R_, G_, U_),
                                 x_desired_placeholder, c3_options_);

  } else if (controller_options_.projection_type == "C3+") {
    c3_ = std::make_unique<C3Plus>(lcs_placeholder,
                                 C3::CostMatrices(Q_, R_, G_, U_),
                                 x_desired_placeholder, c3_options_);

  } else {
    std::cerr << ("Unknown projection type") << std::endl;
    DRAKE_THROW_UNLESS(false);
  }

  if (ic3_options_.add_acceleration_cost) {
    c3_->AddAccelerationCost(n_q_, n_v_, ic3_options_.acceleration_cost_weight);
  }
  if (ic3_options_.add_position_constraints) {  
    c3_->AddLinearConstraint(A_x_, lb_x_, ub_x_, c3::ConstraintVariable::STATE);
  }
  
  if (ic3_options_.add_input_constraints) {
    c3_->AddLinearConstraint(A_u_, lb_u_, ub_u_, c3::ConstraintVariable::INPUT);  
  }

  lcs_state_input_port_ =
      this->DeclareVectorInputPort("x_lcs", TimestampedVector<double>(n_x_))
          .get_index();

  lqr_input_port_ =
      this->DeclareAbstractInputPort("lqr_input", drake::Value<lcmt_lqr_output>())
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
     
  auto c3_solution = c3::systems::C3Output::C3Solution();
  c3_solution.x_sol_ = MatrixXf::Zero(n_q_ + n_v_, N_);
  c3_solution.lambda_sol_ = MatrixXf::Zero(n_lambda_, N_);
  c3_solution.u_sol_ = MatrixXf::Zero(n_u_, N_);
  c3_solution.time_vector_ = VectorXf::Zero(N_);
  c3_solution_port_ =
      this->DeclareAbstractOutputPort("c3_solution", c3_solution,
                                      &iC3TrackingController::OutputC3Solution)
          .get_index();

  plan_start_time_index_ = DeclareDiscreteState(1);
  x_pred_index_ = DeclareDiscreteState(n_x_);
  filtered_solve_time_index_ = DeclareDiscreteState(1);

  if (controller_options_.publish_frequency > 0) {
    DeclarePeriodicDiscreteUpdateEvent(1 / controller_options_.publish_frequency, 0.0,
                                       &iC3TrackingController::ComputePlan);
  } else {
    DeclareForcedDiscreteUpdateEvent(&iC3TrackingController::ComputePlan);
  }

}

LCS iC3TrackingController::CreatePlaceholderLCS() const {
  MatrixXd A = MatrixXd::Ones(n_x_, n_x_);
  MatrixXd B = MatrixXd::Zero(n_x_, n_u_);
  VectorXd d = VectorXd::Zero(n_x_);
  MatrixXd D = MatrixXd::Ones(n_x_, n_lambda_);
  MatrixXd E = MatrixXd::Zero(n_lambda_, n_x_);
  MatrixXd F = MatrixXd::Zero(n_lambda_, n_lambda_);
  MatrixXd H = MatrixXd::Zero(n_lambda_, n_u_);
  VectorXd c = VectorXd::Zero(n_lambda_);
  return LCS(A, B, D, d, E, F, H, c, N_, dt_);
}

drake::systems::EventStatus iC3TrackingController::ComputePlan(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {

  const BasicVector<double>& timestep_vector =
      *this->template EvalVectorInput<BasicVector>(context, timestep_port_);
  int ic3_timestep = static_cast<int>(timestep_vector.get_value()(0));

  std::cout << "context time " << context.get_time() << std::endl;

  // If in teleop or waiting, don't solve
  if (ic3_timestep < 0 || ic3_timestep > ic3_options_.N) return drake::systems::EventStatus::Succeeded();

  auto start = std::chrono::high_resolution_clock::now();

  std::cout << "ic3 timestep " << ic3_timestep << std::endl;

  if (state_data_.cols() == 0 && input_data_.cols() == 0 && force_data_.cols() == 0) {
    const auto* lcm_all_x_trajectories = this->EvalAbstractInput(context, ic3_x_port_);
    const auto* lcm_all_u_trajectories = this->EvalAbstractInput(context, ic3_u_port_);
    const auto* lcm_all_lambda_trajectories = this->EvalAbstractInput(context, ic3_lambda_port_);

    const std::string trajectory_name = "iteration_" + std::to_string(ic3_options_.iter_to_use);

    LcmTrajectory x_trajectory = LcmTrajectory(
        lcm_all_x_trajectories->get_value<lcmt_timestamped_saved_traj>().saved_traj);
    LcmTrajectory u_trajectory = LcmTrajectory(
        lcm_all_u_trajectories->get_value<lcmt_timestamped_saved_traj>().saved_traj);
    LcmTrajectory lambda_trajectory = LcmTrajectory(
        lcm_all_lambda_trajectories->get_value<lcmt_timestamped_saved_traj>().saved_traj);
    LcmTrajectory::Trajectory state_trajectory = x_trajectory.GetTrajectory(trajectory_name);
    LcmTrajectory::Trajectory input_trajectory = u_trajectory.GetTrajectory(trajectory_name);
    LcmTrajectory::Trajectory force_trajectory = lambda_trajectory.GetTrajectory(trajectory_name);
    state_data_ = state_trajectory.datapoints;
    input_data_ = input_trajectory.datapoints;
    force_data_ = force_trajectory.datapoints;

    std::cout << "got trajectories " << std::endl;
  }

  // Get value function from lcm
  if (H_.size() == 0 && g_.size() == 0) {
    const auto* lqr_input = this->EvalAbstractInput(context, lqr_input_port_);
    const auto& lqr_all_inputs = lqr_input->get_value<lcmt_lqr_output>();

    const std::vector<std::vector<std::vector<double>>>& source_H = lqr_all_inputs.H[ic3_options_.iter_to_use];
    const std::vector<std::vector<double>>& source_g = lqr_all_inputs.g[ic3_options_.iter_to_use];

    for (int i = 0; i < source_H.size(); i++) {

      Eigen::MatrixXd mat(MatrixXd::Zero(source_H[i].size(), source_H[i][0].size()));
      for (int j = 0; j < source_H[i].size(); j++) {
          mat.row(j) = Eigen::Map<const Eigen::VectorXd>(source_H[i][j].data(), source_H[i][j].size());
      }
      H_.push_back(std::move(mat));
      g_.push_back(Eigen::VectorXd::Map(source_g[i].data(), source_g[i].size()));
    }
    std::cout << "got value function " << std::endl;
  }

  // Don't run C3 if can't get trajectory or value function
  if (state_data_.cols() == 0 || H_.size() == 0) return drake::systems::EventStatus::Succeeded();

  const TimestampedVector<double>* lcs_x =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        lcs_state_input_port_);
  drake::VectorX<double> x_lcs = lcs_x->get_data();

  if (example_idx_ == 0) {
    const BasicVector<double>* nominal_position =
      (BasicVector<double>*)this->EvalVectorInput(context, nominal_position_port_);
    x_lcs.segment(0, 3) -= nominal_position->get_value();
  }
  std::cout << "x lcs " << x_lcs.transpose() << std::endl;

  auto& x_pred = context.get_discrete_state(x_pred_index_).value();
  auto mutable_x_pred = discrete_state->get_mutable_value(x_pred_index_);
  auto mutable_solve_time =
      discrete_state->get_mutable_value(filtered_solve_time_index_);

  discrete_state->get_mutable_value(plan_start_time_index_)[0] =
      lcs_x->get_timestamp();

  lcs_factory_.UpdateStateAndInput(x_lcs, input_data_.col(std::min(ic3_options_.N-1, ic3_timestep)));
  LCS lcs = lcs_factory_.GenerateLCS();  

  // Assumes 1 quaternion body
  vector<VectorXd> x_desired;
  x_desired.push_back(x_des_);
  VectorXd x_curr = x_lcs;
  for (int i = 1; i < N_+1; i++) {
    int idx = std::min(ic3_options_.N-1, ic3_timestep+i);
    VectorXd u_nominal = input_data_.col(idx);
    x_curr = lcs.Simulate(x_curr, u_nominal);
    // std::cout << "x curr " << x_curr.segment(9, 4).transpose() << std::endl;
    double norm = x_curr.segment(quaternion_indices_[0], 4).norm(); 
    std::cout << "norm " << i << " " << norm << std::endl;

    VectorXd x_des_copy = x_des_;
    x_des_copy.segment(quaternion_indices_[0], 4) = 
        x_des_copy.segment(quaternion_indices_[0], 4) * norm;
    x_desired.push_back(x_des_copy);
  }

  UpdateQuaternionCosts(x_lcs, x_des_);
  C3::CostMatrices new_costs(Q_, R_, G_, U_);

  c3_->UpdateCostMatrices(new_costs);
  c3_->UpdateTarget(x_desired);
  c3_->UpdateLCS(lcs);

  int value_function_idx = std::min(ic3_options_.N-1, ic3_timestep);
  double trust_weight = ic3_options_.vf_trust_region_weight;
  MatrixXd Q_trust = UpdateQuaternionCosts(x_curr, {x_des_}, {Q_[value_function_idx]})[0];
  Q_trust *= trust_weight;

  VectorXd bias = g_[value_function_idx] - (H_[value_function_idx] + Q_trust) * state_data_.col(value_function_idx);
  c3_->UpdateFinalCost(H_[value_function_idx] + Q_trust, bias);

  vector<VectorXd> u_target;
  for (int i = 0; i < N_; i++) {
    VectorXd gravity(VectorXd::Zero(n_u_));
    // HARDCODED
    if (example_idx_ == 0) {
      gravity[2] = 8.5 * 9.81;
    } else if (example_idx_ == 1 || example_idx_ == 2) {
      gravity[2] = 0.02 * 9.81;
      gravity[5] = 0.196;
      gravity[8] = 0.196;
    }
    u_target.push_back(gravity);
  }
  c3_->UpdateInputTarget(u_target);
  

  auto c3_start = std::chrono::high_resolution_clock::now();
  c3_->Solve(x_lcs);  
  auto finish = std::chrono::high_resolution_clock::now();
  auto elapsed = finish - start;
  double solve_time =
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() /
      1e6;
  double c3_solve_time =
      std::chrono::duration_cast<std::chrono::microseconds>(finish - c3_start).count() /
      1e6;
  
  mutable_solve_time[0] = (1 - solve_time_filter_constant_) * solve_time +
                          solve_time_filter_constant_ * mutable_solve_time[0];
  if (controller_options_.publish_frequency > 0) {
    solve_time = 1.0 / controller_options_.publish_frequency;
    mutable_solve_time[0] = solve_time;
  }

  auto z_sol = c3_->GetFullSolution();

  // Print costs
  if (n_u_ == 9 && ic3_options_.print_costs) {
    VectorXd x_d = x_des_;
    for (int i = 0; i < z_sol.size(); i++) {
      double finger_pos_cost = (z_sol[i].segment(0, 9) - x_d.segment(0, 9)).transpose() * 
                                Q_[i].block(0, 0, 9, 9) * (z_sol[i].segment(0, 9) - x_d.segment(0, 9));
      double cube_rot_cost = (z_sol[i].segment(9, 4) - x_d.segment(9, 4)).transpose() *
                                Q_[i].block(9, 9, 4, 4) * (z_sol[i].segment(9, 4) - x_d.segment(9, 4));
      double cube_pos_cost = (z_sol[i].segment(13, 3) - x_d.segment(13, 3)).transpose() *
                                Q_[i].block(13, 13, 3, 3) * (z_sol[i].segment(13, 3) - x_d.segment(13, 3));
      std::cout << "c3 cost " << i << " finger cost " << finger_pos_cost << ", cube rot " << cube_rot_cost << ", cube pos " << cube_pos_cost << std::endl;
    }
    VectorXd x_final = c3_->GetFinalStateSolution();
    double final_finger_pos_cost = (x_final.segment(0, 9) - x_d.segment(0, 9)).transpose() * 
                                      H_[value_function_idx].block(0, 0, 9, 9) * (x_final.segment(0, 9) - x_d.segment(0, 9)); 
    double final_cube_rot_cost = (x_final.segment(9, 4) - x_d.segment(9, 4)).transpose() * 
                                      H_[value_function_idx].block(9, 9, 4, 4) * (x_final.segment(9, 4) - x_d.segment(9, 4));
    double final_cube_pos_cost =(x_final.segment(13, 3) - x_d.segment(13, 3)).transpose() * 
                                      H_[value_function_idx].block(13, 13, 3, 3) * (x_final.segment(13, 3) - x_d.segment(13, 3));

    std::cout << "c3 cost final "<< " finger cost " << final_finger_pos_cost << ", cube rot " 
            << final_cube_rot_cost << ", cube pos " << final_cube_pos_cost << std::endl;

    final_finger_pos_cost += g_[value_function_idx].segment(0, 9).dot(x_final.segment(0, 9) - x_d.segment(0, 9));
    final_cube_rot_cost += g_[value_function_idx].segment(9, 4).dot(x_final.segment(9, 4) - x_d.segment(9, 4));
    final_cube_pos_cost += g_[value_function_idx].segment(13, 3).dot(x_final.segment(13, 3) - x_d.segment(13, 3));

    std::cout << "c3 cost final with affine "<< " finger cost " << final_finger_pos_cost << ", cube rot " 
            << final_cube_rot_cost << ", cube pos " << final_cube_pos_cost << std::endl;
  }

  if (mutable_solve_time[0] < (N_ - 1) * dt_) {
    int index = mutable_solve_time[0] / dt_;
    double weight = (mutable_solve_time[0] - index * dt_) / dt_;
    mutable_x_pred = (1 - weight) * z_sol[index].segment(0, n_x_) +
                     weight * z_sol[index + 1].segment(0, n_x_);
  } else {
    mutable_x_pred = z_sol[N_ - 1].segment(0, n_x_);
  }

  std::cout << "c3 solve time: " << c3_solve_time << std::endl;

  if (ic3_timestep >= 0 && ic3_timestep <= ic3_options_.N) {
    //std::cout << "ic3_timestep: " << ic3_timestep << ": ";
    //std::cout << c3_->GetInputSolution()[0].transpose() << std::endl;    
  }
  return drake::systems::EventStatus::Succeeded();
}

void iC3TrackingController::UpdateQuaternionCosts(
    const Eigen::VectorXd& x_curr, const Eigen::VectorXd& x_des) const {
    
  // Early return if no quaternions or cost parameters not set
  if (quaternion_indices_.size() == 0 ||
      !controller_options_.quaternion_weight.has_value() ||
      !controller_options_.quaternion_regularizer_fraction.has_value()) {
    return;
  }

  for (int index : quaternion_indices_) {
    Eigen::VectorXd quat_curr_i = x_curr.segment(index, 4).normalized();
    Eigen::VectorXd quat_des_i = x_des.segment(index, 4).normalized();

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

    double quaternion_weight = controller_options_.quaternion_weight.value();
    double quaternion_regularizer_fraction =
        controller_options_.quaternion_regularizer_fraction.value();

    // Replace quaternion blocks in Q
    double discount_factor = 1;
    for (int i = 0; i < N_ + 1; i++) {
      Q_[i].block(index, index, 4, 4) =
          discount_factor * c3_options_.w_Q * quaternion_weight *
          (quat_hessian_i + quat_regularizer_1 +
           quaternion_regularizer_fraction * quat_regularizer_2 +
           quat_regularizer_3);
      discount_factor *= controller_options_.c3_options.gamma;
    }
  }
}


vector<MatrixXd> iC3TrackingController::UpdateQuaternionCosts(
    VectorXd x_curr, vector<VectorXd> x_des, vector<MatrixXd> Q_in) const {
  
  // std::cout << x_hat.rows() << ", " << x_hat.cols() << std::endl;
  // std::cout << "xd: " << x_des.transpose() << std::endl;
  // std::cout << c3_quat_norms.size() << std::endl;

  DRAKE_DEMAND(x_des.size() == Q_in.size());

  vector<MatrixXd> Q = Q_in;

  double discount_factor = 1;
  for (int i = 0; i < Q.size(); i++) {
    int j = 0;
    for (int index : quaternion_indices_) {

      // make quaternion costs time-varying based on x_hat
      Eigen::VectorXd quat_curr_i = x_curr.segment(index, 4).normalized();
      Eigen::VectorXd quat_des_i = x_des[i].segment(index, 4).normalized();

      //std::cout << "xhat q: " << quat_curr_i.transpose() << std::endl;

      Eigen::MatrixXd quat_hessian_i = c3::systems::common::hessian_of_squared_quaternion_angle_difference(quat_curr_i, quat_des_i);

      // Regularize hessian so Q is always PSD
      double min_eigenval = quat_hessian_i.eigenvalues().real().minCoeff();
      //std::cout << min_eigenval << std::endl;

      Eigen::MatrixXd Q_quat_regularizer_1 = std::max(0.0, -min_eigenval) * Eigen::MatrixXd::Identity(4, 4);
      Eigen::MatrixXd Q_quat_regularizer_2 = quat_des_i * quat_des_i.transpose();
      Eigen::MatrixXd Q_quat_regularizer_3 = 1e-4 * Eigen::MatrixXd::Identity(4, 4);

      double quaternion_regularizer_fraction =
          controller_options_.quaternion_regularizer_fraction.value();

      Q[i].block(index, index, 4, 4) = 
        discount_factor * 
        controller_options_.c3_options.w_Q * 
        controller_options_.quaternion_weight.value() * (quat_hessian_i + Q_quat_regularizer_1 + 
        quaternion_regularizer_fraction * Q_quat_regularizer_2 + Q_quat_regularizer_3);

      // double q_min_eigenval = Q_[i].eigenvalues().real().minCoeff();
      // std::cout << "Q_" << i << " min eigenvalue " <<  q_min_eigenval << std::endl;
      j++;
    }
    discount_factor *= controller_options_.c3_options.gamma;
  }
  return Q;
}


void iC3TrackingController::OutputC3Solution(
    const drake::systems::Context<double>& context,
    C3Output::C3Solution* c3_solution) const {
  double t = context.get_discrete_state(plan_start_time_index_)[0];
  double solve_time = context.get_discrete_state(filtered_solve_time_index_)[0];

  auto z_sol = c3_->GetFullSolution();
  for (int i = 0; i < N_; i++) {
    if (z_sol[i].array().isNaN().any()) {
      std::cout << "ic3 tracking controller z sol " << i << " has NAN" << std::endl;
    }
    c3_solution->time_vector_(i) = solve_time + t + i * dt_;
    c3_solution->x_sol_.col(i) = z_sol[i].segment(0, n_x_).cast<float>();
    c3_solution->lambda_sol_.col(i) =
        z_sol[i].segment(n_x_, n_lambda_).cast<float>();
    c3_solution->u_sol_.col(i) =
        z_sol[i].segment(n_x_ + n_lambda_, n_u_).cast<float>();

  } 
}

}  // namespace systems
}  // namespace dairlib
