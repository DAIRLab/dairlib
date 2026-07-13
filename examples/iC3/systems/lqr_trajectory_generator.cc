#include "examples/iC3/systems/lqr_trajectory_generator.h"

#include <utility>
#include <iostream>
#include <chrono>

#include "common/find_resource.h"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "multibody/multibody_utils.h"
#include "c3/systems/framework/c3_output.h"

namespace dairlib {

using drake::multibody::ModelInstanceIndex;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::VectorXd;
using Eigen::Vector3d;

using c3::systems::C3Output;
using c3::systems::C3ControllerOptions;
using c3::LCSFactoryOptions;
using c3::LCS;
using c3::ContactPairConfig;
using c3::multibody::LCSFactory;

using std::vector;
using drake::systems::BasicVector;
using systems::TimestampedVector;

LqrTrajectoryGenerator::LqrTrajectoryGenerator(
        const drake::multibody::MultibodyPlant<double>& plant, LCSFactory lcs_factory, iC3Options ic3_options, int example_idx,
        MatrixXd A_x, VectorXd lb_x, VectorXd ub_x, MatrixXd A_u, VectorXd lb_u, VectorXd ub_u)
    : plant_(plant), 
      lcs_factory_(lcs_factory),
      ic3_options_(ic3_options),
      example_idx_(example_idx),
      A_x_(A_x),
      lb_x_(lb_x),
      ub_x_(ub_x), 
      A_u_(A_u),
      lb_u_(lb_u),
      ub_u_(ub_u) {
  this->set_name("lqr_trajectory_generator");

  n_q_ = plant_.num_positions();
  n_v_ = plant_.num_velocities();
  n_x_ = n_q_ + n_v_;
  n_u_ = plant_.num_actuators();

  actor_input_port_ =
      this->DeclareVectorInputPort("actor_input", BasicVector<double>(n_u_))
          .get_index();

  tracking_target_port_ =
      this->DeclareVectorInputPort("tracking target", BasicVector<double>(n_u_))
          .get_index();

  x_curr_port_ =
      this->DeclareVectorInputPort("x_lcs", TimestampedVector<double>(n_x_))
          .get_index();


  actor_trajectory_port_ =
      this->DeclareAbstractOutputPort(
              "lqr_actor_trajectory_output",
              dairlib::lcmt_timestamped_saved_traj(),
              &LqrTrajectoryGenerator::OutputActorTrajectory)
          .get_index();

}

void LqrTrajectoryGenerator::OutputActorTrajectory(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output_traj) const {
  
  const BasicVector<double>& actor_input =
      *this->template EvalVectorInput<BasicVector>(context, actor_input_port_);

  const TimestampedVector<double>* x_curr_vector =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        x_curr_port_);   
  drake::VectorX<double> x_curr = x_curr_vector->get_data();

  std::cout << "x curr " << x_curr.transpose() << std::endl;

  // HARDCODED HOW LONG TO SIMULATE FOR
  int num_sim_timesteps = 2;
  MatrixXd u_hat_input = actor_input.get_value().replicate(1, num_sim_timesteps);
  
  auto start_rollout = std::chrono::high_resolution_clock::now();

  auto [x_hat, u_hat] = SimulateLCS(x_curr, u_hat_input, context);
  x_hat = x_hat.rightCols(x_hat.cols() - 1); // Remove x0 for size consistency

  auto finish_rollout = std::chrono::high_resolution_clock::now();
  auto elapsed_rollout = finish_rollout - start_rollout;
  double solve_time_rollout =
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed_rollout).count() /
      1e6;
  std::cout << "Rollout time " << solve_time_rollout << std::endl;

  MatrixXd forces;
  MatrixXd torques;
  MatrixXd positions;
  MatrixXd orientations;

  if (example_idx_ == 0) {
    forces = u_hat.topRows(3);
    torques = MatrixXd::Zero(3, num_sim_timesteps);
    torques.topRows(2) = u_hat.bottomRows(2);

    positions = x_hat.topRows(3);

    orientations = MatrixXd::Zero(4, num_sim_timesteps);

    for (int i = 0; i < num_sim_timesteps; i++) {
      double roll = x_hat(3, i);
      double pitch = x_hat(4, i);

      Eigen::AngleAxisd rollAngle(roll, Vector3d::UnitX());
      Eigen::AngleAxisd pitchAngle(pitch, Vector3d::UnitY());
      Eigen::Quaterniond q = rollAngle * pitchAngle; 

      VectorXd q_vec(4); 
      q_vec << q.w(), q.x(), q.y(), q.z();
      orientations.col(i) = q_vec.normalized();
    }
  } else if (example_idx_ == 1 || example_idx_ == 2) {
    forces = u_hat;
    positions = x_hat.topRows(9);
  } 

  VectorXd time_vector(num_sim_timesteps);
  for (int i = 0; i < num_sim_timesteps; i++) {
    time_vector(i) = i * ic3_options_.dt;
  }

  LcmTrajectory::Trajectory end_effector_traj;
  end_effector_traj.traj_name = "end_effector_position_target";
  end_effector_traj.datatypes =
      std::vector<std::string>(positions.rows(), "double");
  end_effector_traj.datapoints = positions;
  end_effector_traj.time_vector = time_vector;
  LcmTrajectory lcm_traj({end_effector_traj}, {"end_effector_position_target"},
                         "end_effector_position_target",
                         "end_effector_position_target", false);

  LcmTrajectory::Trajectory force_traj;
  force_traj.traj_name = "end_effector_force_target";
  force_traj.datatypes =
      std::vector<std::string>(forces.rows(), "double");
  force_traj.datapoints = forces;
  force_traj.time_vector = time_vector;
  lcm_traj.AddTrajectory(force_traj.traj_name, force_traj);

  if (example_idx_ == 0) {
    LcmTrajectory::Trajectory torque_traj;
    torque_traj.traj_name = "end_effector_torque_target";
    torque_traj.datatypes =
        std::vector<std::string>(torques.rows(), "double");
    torque_traj.datapoints = torques;
    torque_traj.time_vector = time_vector;
    lcm_traj.AddTrajectory(torque_traj.traj_name, torque_traj);  
    
    LcmTrajectory::Trajectory orientation_traj;
    orientation_traj.traj_name = "end_effector_orientation_target";
    orientation_traj.datatypes =
        std::vector<std::string>(orientations.rows(), "double");
    orientation_traj.datapoints = orientations;
    orientation_traj.time_vector = time_vector;
    lcm_traj.AddTrajectory(orientation_traj.traj_name, orientation_traj);
  }
  
  output_traj->saved_traj = lcm_traj.GenerateLcmObject();
  output_traj->utime = context.get_time() * 1e6;

}


std::tuple<MatrixXd, MatrixXd> LqrTrajectoryGenerator::SimulateLCS(VectorXd x0, MatrixXd u_hat,
                                    const drake::systems::Context<double>& context) const {

  int N = u_hat.cols();

  if (x0.isZero()) {
    MatrixXd x_hat_fallback(MatrixXd::Zero(n_x_, N + 1));
    MatrixXd u_hat_fallback(MatrixXd::Zero(n_u_, N));

    return std::make_tuple(x_hat_fallback, u_hat_fallback);
  }
    
  double dt = ic3_options_.dt / ic3_options_.rollout_dt_scaling;
  lcs_factory_.SetNewDt(dt);


  MatrixXd x_hat(MatrixXd::Zero(n_x_, ic3_options_.rollout_dt_scaling * N + 1));
  MatrixXd u_hat_thresholded(MatrixXd::Zero(n_u_, ic3_options_.rollout_dt_scaling * N));

  x_hat.col(0) = x0;

  VectorXd x_curr = x0;
  for (int i = 0; i < ic3_options_.rollout_dt_scaling * N; i++) {
    
    VectorXd u = u_hat.col(i / ic3_options_.rollout_dt_scaling);

    if (!x_curr.allFinite()) {
      std::cout << "lqr traj gen x_curr not all finite " << x_curr.transpose() << std::endl;
    }

    lcs_factory_.UpdateStateAndInput(x_curr, u);
    LCS lcs = lcs_factory_.GenerateLCS();
    
    // TODO: make this if statement less hardcoded
    if ((example_idx_ == 1 || example_idx_ == 2) && ic3_options_.add_constraints_follow_plan) {

      const BasicVector<double>* tracking_target =
        (BasicVector<double>*)this->EvalVectorInput(context, tracking_target_port_);
                                        // Hardcoded indices
      for (int f = 0; f < 3; f++) {
        A_x_(3*f, 3*f) = 1;
        A_x_(3*f+1, 3*f+1) = 1;

        lb_x_(3*f) = tracking_target->get_value()(3*f) - 0.02;
        lb_x_(3*f+1) = tracking_target->get_value()(3*f+1) - 0.02;
        ub_x_(3*f) = tracking_target->get_value()(3*f) + 0.02;
        ub_x_(3*f+1) = tracking_target->get_value()(3*f+1) + 0.02;
      }
    } 

    // std::cout << "lqr traj gen A_x " << A_x_.diagonal().transpose() << std::endl;
    // std::cout << "lqr traj gen lb_x_ " << lb_x_.transpose() << std::endl;
    // std::cout << "lqr traj gen ub_x_ " << ub_x_.transpose() << std::endl;
    c3::LCSSimulateConfig config;
    config.regularized = true;
    config.min_exp = -16;
    config.max_exp = -6;
    VectorXd x_next = lcs.Simulate(x_curr, u, config);



    // Threshold positions and inputs, assume A_x, A_u are diagonal 0/1 matrices
    for (int j = 0; j < A_x_.rows(); j++) {
      if (A_x_(j, j) == 1) {
          x_curr(j) = std::min(std::max(x_curr(j), lb_x_(j)), ub_x_(j));
      }
    }
    for (int j = 0; j < A_u_.rows(); j++) {
      if (A_u_(j, j) == 1) {
          u(j) = std::min(std::max(u(j), lb_u_(j)), ub_u_(j));
      }
    }

    x_hat.col(i+1) = x_next;
    x_curr = x_next;
    u_hat_thresholded.col(i) = u;
  }

  // HARDCODED remove gravity comp
  for (int i = 0; i < u_hat_thresholded.cols(); i++) {
    if (example_idx_ == 0) {
      u_hat_thresholded.col(i)(2) = u_hat_thresholded.col(i)(2) - 0.85 * 9.81;
    } else if (example_idx_ == 1 || example_idx_ == 2) {
      for (int f = 0; f < 3; f++) {
        u_hat_thresholded.col(i)(3*f+2) = u_hat_thresholded.col(i)(3*f+2) - 0.02 * 9.81;
      }
    }
  }

  MatrixXd x_hat_downsampled(MatrixXd::Zero(n_x_, N + 1));
  MatrixXd u_hat_downsampled(MatrixXd::Zero(n_u_, N));
  for (int i = 0; i < N + 1; i++) {
    x_hat_downsampled.col(i) = x_hat.col(i * ic3_options_.rollout_dt_scaling);

    if (i < N) {
      u_hat_downsampled.col(i) = u_hat_thresholded.col(i * ic3_options_.rollout_dt_scaling);
    }
  }

  return std::make_tuple(x_hat_downsampled, u_hat_downsampled);
}


}  // namespace dairlib
