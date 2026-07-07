#include "ic3_lqr_tracking_controller.h"
#include <Eigen/Dense>
#include "c3/systems/common/quaternion_error_hessian.h"

#include <iostream>
#include <c3/core/lcs.h>

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

iC3LqrTrackingController::iC3LqrTrackingController(
    const drake::multibody::MultibodyPlant<double>& plant, LCSFactory lcs_factory, 
    c3::systems::C3ControllerOptions c3_controller_options, 
    iC3Options ic3_options, VectorXd xd, VectorXd ud, int example_idx)
    : plant_(plant),
      lcs_factory_(lcs_factory),
      c3_controller_options_(c3_controller_options),
      c3_options_(c3_controller_options.c3_options),
      ic3_options_(ic3_options),
      Q_(c3_options_.Q),
      R_(c3_options_.R),
      xd_(xd),
      ud_(ud),
      N_(c3_controller_options_.lcs_factory_options.N),
      dt_(c3_controller_options_.lcs_factory_options.dt),
      example_idx_(example_idx) { 
  this->set_name("ic3_lqr_tracking_controller");

  n_q_ = plant_.num_positions();
  n_v_ = plant_.num_velocities();
  n_u_ = plant_.num_actuators();
  n_x_ = n_q_ + n_v_;

  std::cout << "n_q: " << n_q_ << std::endl;
  std::cout << "n_v: " << n_v_ << std::endl;
  std::cout << "n_u: " << n_u_ << std::endl;
  std::cout << "n_x: " << n_x_ << std::endl;

  // TODO: make this not bad
  if (example_idx_ == 0) {
    n_lambda_ = 4 * 4;
  } else if (example_idx_ == 1) {
    n_lambda_ = 7 * 4;
  } else if (example_idx_ == 2) {
    n_lambda_ = 11 * 4;
  }

  auto lcs_placeholder = CreatePlaceholderLCS();

  auto x_desired_placeholder =
      std::vector<VectorXd>(N_ + 1, VectorXd::Zero(n_x_));

  tracking_target_ = VectorXd::Zero(n_u_);
  u_out_ = VectorXd::Zero(n_u_);

  lcs_state_input_port_ =
      this->DeclareVectorInputPort("x_lcs", TimestampedVector<double>(n_x_))
          .get_index();

  target_input_port_ =
      this->DeclareVectorInputPort("x_lcs_des", n_x_).get_index();

  // Stuff specific to tracking ic3
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
      

  actor_port_ =
      this->DeclareVectorOutputPort("actor_input", BasicVector<double>(n_u_),
                                      &iC3LqrTrackingController::OutputActorInput)
          .get_index();

  tracking_target_port_ =
      this->DeclareVectorOutputPort("tracking_target_port", BasicVector<double>(n_u_),
                                  &iC3LqrTrackingController::OutputTrackingTarget)
          .get_index();

  // Get quaternion bodies
  for (const auto& body_idx : plant_.GetFloatingBaseBodies()) {
    const auto& body = plant_.get_body(body_idx);
    int start = body.floating_positions_start();
    quaternion_indices_.push_back(start);
  }

  DeclareForcedDiscreteUpdateEvent(&iC3LqrTrackingController::ComputePlan);

}

LCS iC3LqrTrackingController::CreatePlaceholderLCS() const {
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

drake::systems::EventStatus iC3LqrTrackingController::ComputePlan(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {

  const BasicVector<double>& timestep_vector =
      *this->template EvalVectorInput<BasicVector>(context, timestep_port_);
  int ic3_timestep = static_cast<int>(timestep_vector.get_value()(0));

  // If in teleop or waiting, don't solve
  if (ic3_timestep < 0 || ic3_timestep >= ic3_options_.N) return drake::systems::EventStatus::Succeeded();

  auto start = std::chrono::high_resolution_clock::now();

  std::cout << "ic3 timestep " << ic3_timestep << std::endl;

  const BasicVector<double>& x_des =
      *this->template EvalVectorInput<BasicVector>(context, target_input_port_);

  const TimestampedVector<double>* lcs_x =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        lcs_state_input_port_);
  drake::VectorX<double> x_lcs = lcs_x->get_data();

  // Get iC3 plan
  if (state_data_.cols() == 0 && input_data_.cols() == 0 && force_data_.cols() == 0) {
    auto start_ic3_traj = std::chrono::high_resolution_clock::now();

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

    auto finish_ic3_traj = std::chrono::high_resolution_clock::now();
    auto elapsed_ic3_traj = finish_ic3_traj - start_ic3_traj;
    double solve_time_ic3_traj =
        std::chrono::duration_cast<std::chrono::microseconds>(elapsed_ic3_traj).count() /
        1e6;
    std::cout << "ic3 traj time " << solve_time_ic3_traj << std::endl;
  }


  const auto* lqr_input = this->EvalAbstractInput(context, lqr_input_port_);
  if (lqr_input == nullptr) return drake::systems::EventStatus::Succeeded();    

  // Get H, K, k_ff
  if (H_.size() == 0 && K_hat_.size() == 0 && k_ff_hat_.size() == 0) {
    auto start_value_function = std::chrono::high_resolution_clock::now();

    const auto& lqr_all_inputs = lqr_input->get_value<dairlib::lcmt_lqr_output>();
    const std::vector<std::vector<std::vector<double>>>& source_H = lqr_all_inputs.H[ic3_options_.iter_to_use];
    const std::vector<std::vector<double>>& source_g = lqr_all_inputs.g[ic3_options_.iter_to_use];
    const std::vector<std::vector<std::vector<double>>>& source_K = lqr_all_inputs.K[ic3_options_.iter_to_use];
    const std::vector<std::vector<double>>& source_k_ff = lqr_all_inputs.k_ff[ic3_options_.iter_to_use];

    for (int i = 0; i < source_H.size(); i++) {
      Eigen::MatrixXd mat(MatrixXd::Zero(source_H[i].size(), source_H[i][0].size()));
      for (int j = 0; j < source_H[i].size(); j++) {
          mat.row(j) = Eigen::Map<const Eigen::VectorXd>(source_H[i][j].data(), source_H[i][j].size());
      }
      H_.push_back(std::move(mat));
      g_.push_back(Eigen::VectorXd::Map(source_g[i].data(), source_g[i].size()));
    }
    for (int i = 0; i < source_K.size(); i++) {
      Eigen::MatrixXd mat(MatrixXd::Zero(source_K[i].size(), source_K[i][0].size()));
      for (int j = 0; j < source_K[i].size(); j++) {
          mat.row(j) = Eigen::Map<const Eigen::VectorXd>(source_K[i][j].data(), source_K[i][j].size());
      }
      K_hat_.push_back(std::move(mat));
      k_ff_hat_.push_back(Eigen::VectorXd::Map(source_k_ff[i].data(), source_k_ff[i].size()));
    }
    
    auto finish_value_function = std::chrono::high_resolution_clock::now();
    auto elapsed_value_function = finish_value_function - start_value_function;
    double solve_time_value_function =
        std::chrono::duration_cast<std::chrono::microseconds>(elapsed_value_function).count() /
        1e6;
    std::cout << "ic3 value function time " << solve_time_value_function << std::endl;
  }


  LCS lcs_sim = GetLCSSegment(ic3_timestep, N_);

  UpdateQuaternionCosts(x_lcs, xd_);

  int idx = std::min(ic3_options_.N-1, ic3_timestep);
  auto start_simulate = std::chrono::high_resolution_clock::now();

  // Simulate to get lambda
  VectorXd u_nominal = input_data_.col(idx);
  c3::LCSSimulateConfig config; 
  config.regularized = true;
  config.min_exp = -16;
  config.max_exp = -6;
  auto [x_out, lambda] = lcs_sim.SimulateAndReturnForce(x_lcs, u_nominal, config);
  
  auto finish_simulate = std::chrono::high_resolution_clock::now();
  auto elapsed_simulate = finish_simulate - start_simulate;
  double solve_time_simulate =
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed_simulate).count() /
      1e6;
  std::cout << "lcs simulate time " << solve_time_simulate << std::endl;

  auto start_matrix_multiplication = std::chrono::high_resolution_clock::now();

  // Update feedforward gain
  MatrixXd B = lcs_sim.B()[0];
  MatrixXd D = lcs_sim.D()[0];
  VectorXd lambda_nominal = force_data_.col(idx);
  VectorXd x_nominal = state_data_.col(idx);

  VectorXd k_ff = UpdateFeedforwardGain(H_[idx+1], B, D, lambda_nominal, lambda, k_ff_hat_[idx]);
  MatrixXd K = K_hat_[idx];


  auto start_line_search = std::chrono::high_resolution_clock::now();
  
  int lookahead_horizon = 5;
  double alpha = DoBackTrackingLineSearch(x_lcs, ic3_timestep, lookahead_horizon);
  
  auto finish_line_search = std::chrono::high_resolution_clock::now();
  auto elapsed_line_search = finish_line_search - start_line_search;
  double solve_time_line_search =
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed_line_search).count() /
      1e6;
  std::cout << "line_search time " << solve_time_line_search << std::endl;

  u_out_ = u_nominal + K * (x_lcs - x_nominal) + alpha * k_ff;



  return drake::systems::EventStatus::Succeeded();
}


int iC3LqrTrackingController::GetNearestXForValueFunction(
    VectorXd x_curr, MatrixXd x_hat_slice) const {
  DRAKE_DEMAND(x_hat_slice.cols() > 0);

  int best_idx = 0;
  double best_cost = std::numeric_limits<double>::infinity();

  // HARDCODED INDICES
  int ee_idx;
  int ee_size;
  int object_idx;
  int object_size = 7;

  if (example_idx_ == 0) {
    ee_idx = 0;
    ee_size = 5;
    object_idx = 5;
  } else if (example_idx_ == 1 || example_idx_ == 2) {
    ee_idx = 0;
    ee_size = 9;
    object_idx = 9;
  }

  for (int i = 0; i < x_hat_slice.cols(); i++) {
    VectorXd x_diff = x_curr - x_hat_slice.col(i);
    double ee_cost = x_diff.segment(ee_idx, ee_size).transpose() * 
                        ic3_options_.value_function_ee_cost.asDiagonal() * x_diff.segment(ee_idx, ee_size);
                        
    double obj_pos_cost = x_diff.segment(object_idx + 4, 3).transpose() * 
                        ic3_options_.value_function_object_position_cost.asDiagonal() * x_diff.segment(object_idx + 4, 3);
                        
    double velo_cost = x_diff.segment(n_q_, n_v_).transpose() * 
                        ic3_options_.value_function_velocity_cost.asDiagonal() * x_diff.segment(n_q_, n_v_);

    Eigen::Quaterniond q_x_hat(x_hat_slice.col(i)(object_idx), x_hat_slice.col(i)(object_idx+1),
                              x_hat_slice.col(i)(object_idx+2),x_hat_slice.col(i)(object_idx+3));
    Eigen::Quaterniond q_curr(x_curr(object_idx), x_curr(object_idx+1), x_curr(object_idx+2), x_curr(object_idx+3));                     

    double obj_rot_cost = ic3_options_.value_function_object_orientation_cost * q_x_hat.angularDistance(q_curr);


    if (ee_cost + obj_pos_cost + velo_cost + obj_rot_cost < best_cost) {
      best_idx = i;
      best_cost = ee_cost + obj_pos_cost + velo_cost + obj_rot_cost;
    }
  }
  return best_idx;

}

VectorXd iC3LqrTrackingController::UpdateFeedforwardGain(MatrixXd H_ip1, MatrixXd B, 
    MatrixXd D, VectorXd lambda_nominal, VectorXd lambda_real, VectorXd k_ff_hat) const {
      
  VectorXd rhs = B.transpose() * H_ip1 * D * (lambda_nominal - lambda_real);
  MatrixXd Q_uu = R_ + B.transpose() * H_ip1 * B;
  Eigen::LDLT<Eigen::MatrixXd> solver(Q_uu);  
  VectorXd k_ff = k_ff_hat + solver.solve(rhs);

  return k_ff;
}

double iC3LqrTrackingController::DoBackTrackingLineSearch(
    Eigen::VectorXd x_curr, int idx, int lookahead_horizon) const {

  c3::LCSSimulateConfig config; 
  config.regularized = true;
  config.min_exp = -16;
  config.max_exp = -6;


  double alpha = 1.0;
  double beta = 0.5;
  int max_steps = 10;

  LCS lcs = GetLCSSegment(idx, lookahead_horizon);

  // Get lambdas from simulating with u nominal
  vector<VectorXd> lambdas;
  VectorXd x_sim = x_curr;
  for (int t = 0; t < lookahead_horizon; t++) {
    VectorXd u_nominal = input_data_.col(std::min(idx + t, ic3_options_.N-1));

    auto [x_out, lambda] = lcs.SimulateAndReturnForceAtTimestep(x_sim, u_nominal, config, t);
    x_sim = x_out;
    lambdas.push_back(lambda);
  }

  double best_alpha = 1.0;
  double best_cost = std::numeric_limits<double>::infinity();
  for (int i = 0; i < max_steps; i++) {

    double x_cost = 0;
    double u_cost = 0;
    double final_cost = 0;

    // Simulate forward lookahead_horizon timesteps to get cost
    VectorXd x_sim = x_curr;
    for (int t = 0; t < lookahead_horizon; t++) {

      int tracking_idx = std::min(idx + t, ic3_options_.N-1);  
      VectorXd x_nominal = state_data_.col(tracking_idx);
      VectorXd u_nominal = input_data_.col(tracking_idx);

      VectorXd k_ff = UpdateFeedforwardGain(H_[tracking_idx+1], lcs.B()[t], 
                        lcs.D()[t], force_data_.col(tracking_idx), lambdas[t], k_ff_hat_[tracking_idx]);
      VectorXd u_tracking = u_nominal + K_hat_[tracking_idx] * (x_sim - x_nominal)    
                              + alpha * k_ff;

      x_cost += (x_sim - xd_).transpose() * Q_ * (x_sim - xd_);
      u_cost += (u_tracking - ud_).transpose() * R_ * (u_tracking - ud_);

      auto [x_next, lambda] = lcs.SimulateAndReturnForceAtTimestep(x_sim, u_tracking, config, t);

      x_sim = x_next;
    }
    int final_idx = std::min(idx + lookahead_horizon, ic3_options_.N);
    VectorXd x_hat = state_data_.col(final_idx);
    final_cost += ((x_sim - x_hat).transpose() * H_[final_idx] * (x_sim - x_hat) + 2 * g_[final_idx].transpose() * (x_sim - x_hat)).value();

    // final_cost = 0;

    std::cout << "alpha " << alpha << std::endl;
    std::cout << "total cost " << x_cost + u_cost + final_cost << std::endl;

    // std::cout << "x_cost " << x_cost << std::endl;
    // std::cout << "u_cost " << u_cost << std::endl;
    // std::cout << "final_cost " << final_cost << std::endl;


    if (x_cost + u_cost + final_cost < best_cost) {
      best_cost = x_cost + u_cost + final_cost;
      best_alpha = alpha;
    }

    alpha *= beta;
  }

  // return previous alpha
  std::cout << "best alpha " << best_alpha << std::endl;
  return best_alpha;

}



void iC3LqrTrackingController::UpdateQuaternionCosts(
    const Eigen::VectorXd& x_curr, const Eigen::VectorXd& x_des) const {
    
  // Early return if no quaternions or cost parameters not set
  if (quaternion_indices_.size() == 0 ||
      !c3_controller_options_.quaternion_weight.has_value() ||
      !c3_controller_options_.quaternion_regularizer_fraction.has_value()) {
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

    double quaternion_weight = c3_controller_options_.quaternion_weight.value();
    double quaternion_regularizer_fraction =
        c3_controller_options_.quaternion_regularizer_fraction.value();

    // Replace quaternion blocks in Q
    Q_.block(index, index, 4, 4) =
        c3_options_.w_Q * quaternion_weight *
        (quat_hessian_i + quat_regularizer_1 +
          quaternion_regularizer_fraction * quat_regularizer_2 +
          quat_regularizer_3);
    
  }
}


LCS iC3LqrTrackingController::MakeTimeVaryingLCS(MatrixXd x_hat, MatrixXd u_hat) const {
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
    
    for (auto idx : quaternion_indices_) {
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

LCS iC3LqrTrackingController::GetLCSSegment(int start_idx, int size) const {
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


void iC3LqrTrackingController::OutputActorInput(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* actor_input) const {
  std::cout << "u_out output port " << u_out_.transpose() << std::endl;
	actor_input->get_mutable_value() = u_out_;
}


void iC3LqrTrackingController::OutputTrackingTarget(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* tracking_target) const {
	tracking_target->get_mutable_value() = tracking_target_;
}


}  // namespace systems
}  // namespace dairlib
