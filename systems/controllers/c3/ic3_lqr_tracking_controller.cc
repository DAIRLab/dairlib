#include "ic3_lqr_tracking_controller.h"
#include <Eigen/Dense>

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
    const drake::multibody::MultibodyPlant<double>& plant, 
      iC3Options ic3_options, MatrixXd R, int N, int example_idx)
    : plant_(plant),
      ic3_options_(ic3_options),
      R_(R),
      N_(N),
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
    n_lambda_ = 4;
  } else if (example_idx_ == 1) {
    n_lambda_ = 7;
  }

  auto lcs_placeholder = CreatePlaceholderLCS();

  auto x_desired_placeholder =
      std::vector<VectorXd>(N_ + 1, VectorXd::Zero(n_x_));

  tracking_target_ = VectorXd::Zero(n_u_);
  u_out_ = VectorXd::Zero(n_u_);

  lcs_state_input_port_ =
      this->DeclareVectorInputPort("x_lcs", TimestampedVector<double>(n_x_))
          .get_index();
  lcs_input_port_ =
      this->DeclareAbstractInputPort("lcs", drake::Value<LCS>(lcs_placeholder))
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
  if (ic3_timestep < 0) return drake::systems::EventStatus::Succeeded();

  auto start = std::chrono::high_resolution_clock::now();

  std::cout << "ic3 timestep " << ic3_timestep << std::endl;

  const BasicVector<double>& x_des =
      *this->template EvalVectorInput<BasicVector>(context, target_input_port_);
  const TimestampedVector<double>* lcs_x =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        lcs_state_input_port_);
  auto& lcs =
      this->EvalAbstractInput(context, lcs_input_port_)->get_value<LCS>();
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

  // Get H, K, k_ff
  vector<Eigen::MatrixXd> H;
  vector<Eigen::MatrixXd> K_hat;
  vector<Eigen::VectorXd> k_ff_hat;

  const auto* lqr_input = this->EvalAbstractInput(context, lqr_input_port_);
  const auto& lqr_all_inputs = lqr_input->get_value<dairlib::lcmt_lqr_output>();
  const std::vector<std::vector<std::vector<double>>>& source_H = lqr_all_inputs.H[ic3_options_.iter_to_use];
  const std::vector<std::vector<std::vector<double>>>& source_K = lqr_all_inputs.H[ic3_options_.iter_to_use];
  const std::vector<std::vector<double>>& source_k_ff = lqr_all_inputs.g[ic3_options_.iter_to_use];
  for (int i = 0; i < source_H.size(); i++) {
    Eigen::MatrixXd mat(MatrixXd::Zero(source_H[i].size(), source_H[i][0].size()));
    for (int j = 0; j < source_H[i].size(); j++) {
        mat.row(j) = Eigen::Map<const Eigen::VectorXd>(source_H[i][j].data(), source_H[i][j].size());
    }
    H.push_back(std::move(mat));
  }
  for (int i = 0; i < source_K.size(); i++) {
    Eigen::MatrixXd mat(MatrixXd::Zero(source_K[i].size(), source_K[i][0].size()));
    for (int j = 0; j < source_K[i].size(); j++) {
        mat.row(j) = Eigen::Map<const Eigen::VectorXd>(source_K[i][j].data(), source_K[i][j].size());
    }
    K_hat.push_back(std::move(mat));
    k_ff_hat.push_back(Eigen::VectorXd::Map(source_k_ff[i].data(), source_k_ff[i].size()));
  }

  int idx = std::min(ic3_options_.N-1, ic3_timestep);

  // Simulate to get lambda
  VectorXd u_nominal = input_data.col(idx);
  c3::LCSSimulateConfig config; 
  config.regularized = true;
  config.max_exp = -6;
  auto [x_out, lambda] = lcs.SimulateAndReturnForce(x_lcs, u_nominal, config);
  
  // Update feedforward gain
  MatrixXd B = lcs.B()[0];
  MatrixXd D = lcs.D()[0];
  VectorXd lambda_nominal = force_data.col(idx);
  VectorXd x_nominal = state_data.col(idx);

  VectorXd rhs = B.transpose() * H[idx+1] * D * (lambda_nominal - lambda);
  MatrixXd Q_uu = R_ + B.transpose() * H[idx+1] * B.transpose();
  Eigen::LDLT<Eigen::MatrixXd> solver(Q_uu);  

  VectorXd k_ff = k_ff_hat[idx] + solver.solve(rhs);
  MatrixXd K = K_hat[idx];

  double alpha = 1;
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

void iC3LqrTrackingController::OutputActorInput(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* actor_input) const {
	actor_input->get_mutable_value() = u_out_;
}


void iC3LqrTrackingController::OutputTrackingTarget(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* tracking_target) const {
	tracking_target->get_mutable_value() = tracking_target_;
}


}  // namespace systems
}  // namespace dairlib
