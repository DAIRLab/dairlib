#include "open_loop_trajectory_generator.h"

#include <iostream>
#include <utility>

#include "common/find_resource.h"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "multibody/multibody_utils.h"
#include "systems/framework/timestamped_vector.h"

namespace dairlib {

using drake::multibody::ModelInstanceIndex;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::VectorXd;
using systems::TimestampedVector;
using std::vector;
using c3::LCS;
using c3::multibody::LCSFactory;

OpenLoopTrajectoryGenerator::OpenLoopTrajectoryGenerator(
    const drake::multibody::MultibodyPlant<double>& plant, iC3Options ic3_options, LCSFactory lcs_factory, 
      bool track_dynamically_feasible, int example_idx)
    : plant_(plant), 
		ic3_options_(std::move(ic3_options)), 
    lcs_factory_(lcs_factory),
		example_idx_(example_idx),
    track_dynamically_feasible_(track_dynamically_feasible),
    iter_to_use_(ic3_options.iter_to_use),
		N_(ic3_options.N),
		dt_(ic3_options.dt) {
  this->set_name("open_loop_trajectory_generator");

  n_q_ = plant_.num_positions();
  n_v_ = plant_.num_velocities();
  n_x_ = n_q_ + n_v_;
  n_u_ = plant_.num_actuators();
	  
  x_curr_port_ =
      this->DeclareVectorInputPort("x_lcs", TimestampedVector<double>(n_x_))
          .get_index();

  ic3_x_port_ =
      this->DeclareAbstractInputPort("ic3_x_trajectory",
                                     drake::Value<lcmt_timestamped_saved_traj>())
          .get_index();

	ic3_u_port_ =
      this->DeclareAbstractInputPort("ic3_u_trajectory",
                                     drake::Value<lcmt_timestamped_saved_traj>())
          .get_index();

  // HARDCODED
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

  timestep_port_ =  
      this->DeclareVectorInputPort("timestep_port", 1).get_index();
      
  actor_trajectory_port_ =
      this->DeclareAbstractOutputPort(
              "ic3_actor_trajectory_output",
              dairlib::lcmt_timestamped_saved_traj(),
              &OpenLoopTrajectoryGenerator::OutputActorTrajectory)
          .get_index();

}

void OpenLoopTrajectoryGenerator::OutputActorTrajectory(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output_traj) const {

  const BasicVector<double>& timestep_vector =
      *this->template EvalVectorInput<BasicVector>(context, timestep_port_);
  int ic3_timestep = static_cast<int>(timestep_vector.get_value()(0));

  if (ic3_timestep < 0 || ic3_timestep > N_) return;

  // Get iC3 plan
  const std::string trajectory_name = "iteration_" + std::to_string(ic3_options_.iter_to_use);
  const auto& lcm_all_x_trajectories = 
    this->EvalAbstractInput(context, ic3_x_port_)->get_value<lcmt_timestamped_saved_traj>();
  const auto& lcm_all_u_trajectories = 
    this->EvalAbstractInput(context, ic3_u_port_)->get_value<lcmt_timestamped_saved_traj>();
  LcmTrajectory x_trajectory = LcmTrajectory(lcm_all_x_trajectories.saved_traj);
  LcmTrajectory u_trajectory = LcmTrajectory(lcm_all_u_trajectories.saved_traj);
  LcmTrajectory::Trajectory state_trajectory = x_trajectory.GetTrajectory(trajectory_name);
  LcmTrajectory::Trajectory input_trajectory = u_trajectory.GetTrajectory(trajectory_name);
  MatrixXd state_data = state_trajectory.datapoints;
  MatrixXd input_data = input_trajectory.datapoints;

  const BasicVector<double>* nominal_position =
    (BasicVector<double>*)this->EvalVectorInput(context, nominal_position_port_);

  const TimestampedVector<double>* lcs_x =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        x_curr_port_);
  drake::VectorX<double> x_lcs = lcs_x->get_data();
  std::cout << "x lcs " << x_lcs.transpose() << std::endl;

  MatrixXd positions;
  MatrixXd orientations;
  MatrixXd forces;
  MatrixXd torques;

  VectorXd timestamps(5);
  for (int t = 0; t < 5; t++) {
    timestamps(t) = t * dt_;
  }

  std::cout << "open loop ic3 timestep " << ic3_timestep << std::endl;
  

  if (example_idx_ == 1 || example_idx_ == 2) {



    if (ic3_timestep % 8 == 0) {
      VectorXd x_plan = state_data.col(ic3_timestep);
      Eigen::Quaterniond cube_rot_plan(x_plan(9), x_plan(10), x_plan(11), x_plan(12));
      Eigen::Vector3d cube_pos_plan(x_plan.segment(13, 3));
      drake::math::RigidTransform<double> X_W_Nom(cube_rot_plan, cube_pos_plan);

      Eigen::Quaterniond cube_rot_curr(x_lcs(9), x_lcs(10), x_lcs(11), x_lcs(12));
      Eigen::Vector3d cube_pos_curr(x_lcs.segment(13, 3));
      drake::math::RigidTransform<double> X_W_Curr(cube_rot_curr, cube_pos_curr);

      X_delta_ = X_W_Curr * X_W_Nom.inverse();
    }

    vector<VectorXd> ee_x_des;
    vector<VectorXd> ee_u_des;
    for (int i = 0; i < 5; i++) {
      int idx = std::min(ic3_timestep + i, ic3_options_.N); 
      VectorXd x_plan = state_data.col(idx);
      VectorXd u_plan = input_data.col(idx);
      // std::cout << "u plan " << u_plan.transpose() << std::endl;

      VectorXd ee_x(9);
      VectorXd ee_u(9);
      for (int f = 0; f < 3; f++) {
        ee_x.segment(3*f, 3) = X_delta_ * x_plan.segment(3*f, 3);
        ee_u.segment(3*f, 3) = X_delta_.rotation() * u_plan.segment(3*f, 3);
      }
      ee_x_des.push_back(ee_x);
      ee_u_des.push_back(ee_u);
      // std::cout << "u trans " << ee_u.transpose() << std::endl;
    }


    VectorXd x_lcs_new_ee = x_lcs;
    x_lcs_new_ee.segment(0, 9) = ee_x_des[0];


    MatrixXd u_hat_plan(9, 5);
    for (int i = 0; i < 5; i++) {
      int idx = std::min(ic3_options_.N, ic3_timestep + i);
      u_hat_plan.col(i) = ee_u_des.at(i);
    }

    auto [x_hat, u_hat] = SimulateLCS(x_lcs_new_ee, u_hat_plan);


    positions = MatrixXd::Zero(9, 5);
    forces = MatrixXd::Zero(9, 5);

    // Remove gravity
    for (int i = 0; i < 5; i++) {
      for (int f = 0; f < 3; f++) {
        ee_u_des[i](3*f + 2) -= 0.02 * 9.81;
      }
    }

    for (int i = 0; i < 5; i++) {
      int idx = std::min(N_, ic3_timestep + i);
      if (track_dynamically_feasible_) {
        positions.col(i) = x_hat.col(i).segment(0, 9);
      } else {
        positions.col(i) = ee_x_des[i].segment(0, 9);
      }
      forces.col(i) = ee_u_des.at(i);
    }

  } else if (example_idx_ == 0) {
    positions = MatrixXd::Zero(3, 5);
    orientations = MatrixXd::Zero(4, 5);
    forces = MatrixXd::Zero(3, 5);
    torques = MatrixXd::Zero(3, 5);

    for (int i = 0; i < 5; i++) {
      int idx = std::min(N_, ic3_timestep + i);
      positions.col(i) = state_data.col(idx).segment(0, 3) + nominal_position->get_value();
      forces.col(i) = input_data.col(idx).segment(0, 3);
      torques.col(i).segment(0, 2) = input_data.col(idx).segment(3, 2);

      double roll = state_data.col(idx)(3);
      double pitch = state_data.col(idx)(4);

      Eigen::AngleAxisd rollAngle(roll, Vector3d::UnitX());
      Eigen::AngleAxisd pitchAngle(pitch, Vector3d::UnitY());

      Eigen::Quaterniond q = rollAngle * pitchAngle; 
      VectorXd q_vec(4); 
      q_vec << q.w(), q.x(), q.y(), q.z();

      orientations.col(i) = q_vec.normalized();
    }

    // Remove gravity 
    for (int i = 0; i < 5; i++) {
      forces.col(i)(2) -= 0.85 * 9.81;
    }
  }


  LcmTrajectory::Trajectory position_traj;
  position_traj.traj_name = "end_effector_position_target";
  position_traj.datatypes = std::vector<std::string>(positions.rows(), "double"); 
  position_traj.datapoints = positions;
  position_traj.time_vector = timestamps;

  LcmTrajectory::Trajectory force_traj;
  force_traj.traj_name = "end_effector_force_target";
  force_traj.datatypes = std::vector<std::string>(forces.rows(), "double"); 
  force_traj.datapoints = forces;
  force_traj.time_vector = timestamps;

  LcmTrajectory::Trajectory orientation_traj;
  LcmTrajectory::Trajectory torque_traj;

  if (example_idx_ == 0) {
    orientation_traj.traj_name = "end_effector_orientation_target";
    orientation_traj.datatypes = std::vector<std::string>(orientations.rows(), "double"); 
    orientation_traj.datapoints = orientations;
    orientation_traj.time_vector = timestamps;

    torque_traj.traj_name = "end_effector_torque_target";
    torque_traj.datatypes = std::vector<std::string>(torques.rows(), "double"); 
    torque_traj.datapoints = torques;
    torque_traj.time_vector = timestamps;
  }

  LcmTrajectory lcm_trajectory({position_traj}, {"end_effector_position_target"},
                              "end_effector_position_target", "end_effector_position_target", false);
  lcm_trajectory.AddTrajectory("end_effector_force_target", force_traj);   

  if (example_idx_ == 0) {
    lcm_trajectory.AddTrajectory("end_effector_orientation_target", orientation_traj);   
    lcm_trajectory.AddTrajectory("end_effector_torque_target", torque_traj);   
  }


  output_traj->saved_traj = lcm_trajectory.GenerateLcmObject();
  output_traj->utime = context.get_time() * 1e6;

  
}


std::tuple<MatrixXd, MatrixXd> OpenLoopTrajectoryGenerator::SimulateLCS(VectorXd x0, MatrixXd u_hat) const {

  int N = u_hat.cols();

  // std::cout << "x0 " << x0.transpose() << std::endl;
  // First C3 solve not done yet
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

    // Enforce velocity limits on fingers
    if (example_idx_ == 1) {
      VectorXd q_dot = x_curr.segment(16, 9);
      for (int i = 0; i < 9; i++) {
        double q_ddot = std::clamp(u(i) / 0.02, (-0.075 - q_dot(i)) / (5.0 * dt), (0.075 - q_dot(i)) / (5.0 * dt));
        u(i) = q_ddot * 0.02;
      }
    }

    if (!x_curr.allFinite()) {
      std::cout << "open loop traj gen x_curr not all finite " << i  << " " << x_curr.transpose() << std::endl;
    }

    lcs_factory_.UpdateStateAndInput(x_curr, u);
    LCS lcs = lcs_factory_.GenerateLCS();

    c3::LCSSimulateConfig config;
    config.regularized = true;
    x_curr = lcs.Simulate(x_curr, u, config);
    x_hat.col(i+1) = x_curr;
    
    u_hat_thresholded.col(i) = u;
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
