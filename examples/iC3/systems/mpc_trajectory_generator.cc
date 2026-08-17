#include "examples/iC3/systems/mpc_trajectory_generator.h"

#include <utility>
#include <iostream>
#include <chrono>

#include "common/find_resource.h"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "multibody/multibody_utils.h"
#include "c3/systems/framework/c3_output.h"
#include "systems/framework/timestamped_vector.h"

namespace dairlib {

using drake::multibody::ModelInstanceIndex;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::VectorXd;
using Eigen::Vector3d;
using systems::TimestampedVector;

using c3::systems::C3Output;
using c3::systems::C3ControllerOptions;
using c3::LCSFactoryOptions;
using c3::LCS;
using c3::ContactPairConfig;
using c3::multibody::LCSFactory;
using c3::multibody::GeomGeomCollider;

using drake::geometry::GeometryId;
using drake::geometry::SceneGraph;
using drake::SortedPair;
using drake::geometry::SceneGraph;
using std::vector;
using drake::systems::BasicVector;

MPCTrajectoryGenerator::MPCTrajectoryGenerator(
    const drake::multibody::MultibodyPlant<double>& plant, drake::systems::Context<double>* plant_context,
    c3::multibody::LCSFactory lcs_factory, iC3Options ic3_options, c3::LCSFactoryOptions lcs_factory_options, 
    bool track_dynamically_feasible, int example_idx,
    MatrixXd A_x, VectorXd lb_x, VectorXd ub_x, MatrixXd A_u, VectorXd lb_u, VectorXd ub_u)
    : plant_(plant), 
      plant_context_(plant_context),
      lcs_factory_(lcs_factory),
      ic3_options_(ic3_options),
      lcs_factory_options_(lcs_factory_options),
      N_(lcs_factory_options_.N), 
      track_dynamically_feasible_(track_dynamically_feasible),
      example_idx_(example_idx),
      A_x_(A_x),
      lb_x_(lb_x),
      ub_x_(ub_x), 
      A_u_(A_u),
      lb_u_(lb_u),
      ub_u_(ub_u) {
  this->set_name("c3trajectory_generator");

  n_q_ = plant_.num_positions();
  n_v_ = plant_.num_velocities();
  n_x_ = n_q_ + n_v_;


	int n_lambda_with_tangential = 0;
  int num_contacts = lcs_factory_options_.contact_pair_configs.value().size();
  for (ContactPairConfig pair : lcs_factory_options_.contact_pair_configs.value()) {
    n_lambda_with_tangential += 2 * pair.num_friction_directions;
  }

  if (lcs_factory_options_.contact_model == "stewart_and_trinkle") {
    n_lambda_ =
        2 * num_contacts + n_lambda_with_tangential;
  } else if (lcs_factory_options_.contact_model == "anitescu") {
    n_lambda_ = n_lambda_with_tangential;
  } else {
    std::cerr << ("Unknown or unsupported contact model: " +
      lcs_factory_options_.contact_model) << std::endl;
    DRAKE_THROW_UNLESS(false);
  }
  n_u_ = plant_.num_actuators();

  solution_port_ =
      this->DeclareAbstractInputPort("solution",
                                     drake::Value<C3Output::C3Solution>())
          .get_index();

  int nominal_position_size;
  if (example_idx_ == 0) {
    nominal_position_size = 3;
  } else if (example_idx_ == 1 || example_idx_ == 2) {
    nominal_position_size = 9;
  }

  x_lcs_port_ =
      this->DeclareVectorInputPort("x_lcs", TimestampedVector<double>(n_x_))
          .get_index();

  nominal_position_port_ =
      this->DeclareVectorInputPort(
              "nominal_position", BasicVector<double>(nominal_position_size))
          .get_index();    

  actor_trajectory_port_ =
      this->DeclareAbstractOutputPort(
              "c3_actor_trajectory_output",
              dairlib::lcmt_timestamped_saved_traj(),
              &MPCTrajectoryGenerator::OutputActorTrajectory)
          .get_index();

  auto lcs_placeholder = CreatePlaceholderLCS();

}

// HARDCODED FOR PLATE EXAMPLE
void MPCTrajectoryGenerator::OutputActorTrajectory(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output_traj) const {
  const auto& c3_solution =
      this->EvalInputValue<C3Output::C3Solution>(context, solution_port_);

  DRAKE_DEMAND(c3_solution->x_sol_.rows() == n_q_ + n_v_);

  const TimestampedVector<double>* x_lcs_vector =
    (TimestampedVector<double>*)this->EvalVectorInput(context, x_lcs_port_);

  MatrixXd x_hat = c3_solution->x_sol_.cast<double>();
  MatrixXd u_hat = c3_solution->u_sol_.cast<double>();

  if (x_hat.isZero()) return;

  if (x_hat.array().isNaN().any()) {
    std::cout << "x_hat HAS NAN " << std::endl;
    std::cout << x_hat.rows() << " x " << x_hat.cols() << std::endl;
  }

  for (int i = 0; i < x_hat.cols(); i++) {
    if (example_idx_ == 0) {
      std::cout << "mpc x sol plate, cube " << i << " " << x_hat.col(i).segment(0, 12).transpose() << std::endl;
    } else if (example_idx_ == 1 || example_idx_ == 2) {
      std::cout << "mpc x sol cube " << i << " " << x_hat.col(i).segment(9, 7).transpose() << std::endl;
    }
  }

  for (int i = 0; i < u_hat.cols(); i++) {
    if (example_idx_ == 0) {
      std::cout << "mpc u sol " << i << " " << u_hat.col(i).transpose() << std::endl;
    } else if (example_idx_ == 1 || example_idx_ == 2) {
      std::cout << "mpc u sol " << i << " " << u_hat.col(i).transpose() << std::endl;
    }
  }

  if (track_dynamically_feasible_) {
    auto simulate_start = std::chrono::high_resolution_clock::now();
    auto [x_hat_out, u_hat_out] = SimulateLCS(x_hat.col(0), u_hat, context);
    auto simulate_end = std::chrono::high_resolution_clock::now();
    auto elapsed = simulate_end - simulate_start;
    double solve_time =
        std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() /
        1e6;
    std::cout << "rollout time " << solve_time << std::endl;

    x_hat = x_hat_out;
    u_hat = u_hat_out;

  } else {
    MatrixXd temp(x_hat.rows(), x_hat.cols() + 1);
    temp << x_hat, x_hat.col(x_hat.cols()-1);
    x_hat = temp; 

    // Threshold
    for (int i = 0; i < x_hat.cols(); i++) {
      for (int j = 0; j < A_x_.rows(); j++) {
        if (A_x_(j, j) == 1) {
          x_hat.col(i)(j) = std::clamp(x_hat.col(i)(j), lb_x_(j), ub_x_(j));
        }
      }
    }
  }

  // Stop double counting gravity
  if (example_idx_ == 0) {
    for (int i = 0; i < u_hat.cols(); i++) {
      u_hat.col(i)(2) -= (9.81 * 0.85);
    }
  } else if (example_idx_ == 1 || example_idx_ == 2) {
    for (int i = 0; i < u_hat.cols(); i++) {
      u_hat.col(i)(2) -= (9.81 * 0.02);
      u_hat.col(i)(5) -= (9.81 * 0.02);
      u_hat.col(i)(8) -= (9.81 * 0.02);
    }
  }

	// Make non-degenerate trajectory for N = 1
  MatrixXd positions;
	MatrixXd raw_orientations;
	MatrixXd forces;
  MatrixXd torques;

  VectorXd time_vector;

  // HARDCODED
  if (example_idx_ == 0) {
    int ee_pos_idx = 0;
    int ee_rot_idx = 3; 
    int force_idx = 0;
    int torque_idx = 3;

    const BasicVector<double>* nominal_position =
      (BasicVector<double>*)this->EvalVectorInput(context, nominal_position_port_);

    for (int i = 0; i < x_hat.cols(); i++) {
      x_hat.col(i).segment(0, 3) += nominal_position->get_value();
    }

    if (N_ == 1) {
      positions = MatrixXd::Zero(3, 2);
      positions.col(0) = x_hat.middleRows(ee_pos_idx, 3).col(0);
      positions.col(1) = x_hat.middleRows(ee_pos_idx, 3).col(1);

      raw_orientations = MatrixXd::Zero(2, 2);
      raw_orientations.col(0) = x_hat.middleRows(ee_rot_idx, 2).col(0);
      raw_orientations.col(1) = x_hat.middleRows(ee_rot_idx, 2).col(1);

      forces = MatrixXd::Zero(3, 2);
      forces.col(0) = u_hat.col(0).segment(force_idx, 3);
      forces.col(1) = u_hat.col(0).segment(force_idx, 3);

      torques = MatrixXd::Zero(3, 2);
      torques.col(0).segment(0, 2) = u_hat.col(0).segment(torque_idx, 2);
      torques.col(1).segment(0, 2) = u_hat.col(0).segment(torque_idx, 2);

      time_vector = VectorXd::Zero(2);
      time_vector[0] = c3_solution->time_vector_.cast<double>()(0);
      time_vector[1] = time_vector[0] + lcs_factory_options_.dt;

    } else {
      x_hat = x_hat.rightCols(x_hat.cols() - 1); // Remove x0 for size consistency

      // Reapply offset
      positions = x_hat.middleRows(ee_pos_idx, 3);
      for (int i = 0; i < positions.cols(); i++) {
        positions.col(i) = positions.col(i);
      }
      raw_orientations = x_hat.middleRows(ee_rot_idx, 2);

      forces = u_hat.topRows(3);
      torques = MatrixXd::Zero(3, u_hat.cols());
      torques.topRows(2) = u_hat.bottomRows(2);
      time_vector = c3_solution->time_vector_.cast<double>();
    }

    // Threshold plate positions for safety
    // for (int k = 0; k < positions.cols(); k++) {
    //   positions.col(k)(0) = std::min(std::max(positions.col(k)(0), nom_pos(0) - 0.15), nom_pos(0) + 0.15);
    //   positions.col(k)(1) = std::min(std::max(positions.col(k)(1), nom_pos(1) - 0.15), nom_pos(1) + 0.15);
    //   positions.col(k)(2) = std::min(std::max(positions.col(k)(2), nom_pos(2) - 0.15), nom_pos(2) + 0.15);
    // }

  } else if (example_idx_ == 1 || example_idx_ == 2) {
    int ee_pos_idx = 0;
    int force_idx = 0;

    std::cout << "x hat " << x_hat.rows() << " x " << x_hat.cols() << std::endl;
    std::cout << "u hat " << u_hat.rows() << " x " << u_hat.cols() << std::endl;

    if (N_ == 1) {
      positions = MatrixXd::Zero(9, 2);
      positions.col(0) = x_hat.middleRows(ee_pos_idx, 9).col(0);
      positions.col(1) = x_hat.middleRows(ee_pos_idx, 9).col(1);

      forces = MatrixXd::Zero(9, 2);
      forces.col(0) = u_hat.col(0).segment(force_idx, 9);
      forces.col(1) = u_hat.col(0).segment(force_idx, 9);

      time_vector = VectorXd::Zero(2);
      time_vector[0] = c3_solution->time_vector_.cast<double>()(0);
      time_vector[1] = time_vector[0] + lcs_factory_options_.dt;

    } else {
      x_hat = x_hat.rightCols(x_hat.cols() - 1); // Remove x0 for size consistency

      positions = x_hat.middleRows(ee_pos_idx, 9);
      forces = u_hat;
      time_vector = c3_solution->time_vector_.cast<double>();
    }

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

  // Don't need orientation/torque for trifinger
  if (example_idx_ == 0) {
    int ee_pos_idx = 0;
    int ee_rot_idx = 3; 
    int force_idx = 0;
    int torque_idx = 3;  

    LcmTrajectory::Trajectory torque_traj;
    torque_traj.traj_name = "end_effector_torque_target";
    torque_traj.datatypes =
        std::vector<std::string>(3, "double");
    torque_traj.datapoints = torques;
    torque_traj.time_vector = time_vector;
    lcm_traj.AddTrajectory(torque_traj.traj_name, torque_traj);

    LcmTrajectory::Trajectory end_effector_orientation_traj;
    MatrixXd orientations;
		if (N_ == 1) {
			orientations = MatrixXd::Zero(4, 2);
		} else {
			orientations = MatrixXd::Zero(4, N_);
		}

    MatrixXd raw_orientations = x_hat.middleRows(ee_rot_idx, 2);
		for (int i = 0; i < raw_orientations.cols(); i++) {
			  double roll = raw_orientations(0, i);
        double pitch = raw_orientations(1, i);

				Eigen::AngleAxisd rollAngle(roll, Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Vector3d::UnitY());

        Eigen::Quaterniond q = rollAngle * pitchAngle; 
        VectorXd q_vec(4); 
        q_vec << q.w(), q.x(), q.y(), q.z();

        orientations.col(i) = q_vec.normalized();
		}

    end_effector_orientation_traj.traj_name = "end_effector_orientation_target";
    end_effector_orientation_traj.datatypes =
        std::vector<std::string>(orientations.rows(), "double");
    end_effector_orientation_traj.datapoints = orientations;
    end_effector_orientation_traj.time_vector = time_vector;
    lcm_traj.AddTrajectory(end_effector_orientation_traj.traj_name,
                           end_effector_orientation_traj);

		// std::cout << "position size: " << positions.rows() << ", " << positions.cols() << std::endl;
		// std::cout << "orientations size: " << orientations.rows() << ", " << orientations.cols() << std::endl;
		// std::cout << "forces size: " << forces.rows() << ", " << forces.cols() << std::endl;
  }

  output_traj->saved_traj = lcm_traj.GenerateLcmObject();
  output_traj->utime = context.get_time() * 1e6;
}

std::tuple<MatrixXd, MatrixXd> MPCTrajectoryGenerator::SimulateLCS(VectorXd x0, MatrixXd u_hat,
                                    const drake::systems::Context<double>& context) const {

  int N = lcs_factory_options_.N;

  // std::cout << "x0 " << x0.transpose() << std::endl;
  // First C3 solve not done yet
  if (x0.isZero()) {
    MatrixXd x_hat_fallback(MatrixXd::Zero(n_x_, N + 1));
    MatrixXd u_hat_fallback(MatrixXd::Zero(n_u_, N));

    return std::make_tuple(x_hat_fallback, u_hat_fallback);
  }
    
  double dt = lcs_factory_options_.dt / ic3_options_.rollout_dt_scaling;
  lcs_factory_.SetNewDt(dt);


  MatrixXd x_hat(MatrixXd::Zero(n_x_, ic3_options_.rollout_dt_scaling * N + 1));
  MatrixXd u_hat_thresholded(MatrixXd::Zero(n_u_, ic3_options_.rollout_dt_scaling * N));

  x_hat.col(0) = x0;

  VectorXd x_curr = x0;
  for (int i = 0; i < ic3_options_.rollout_dt_scaling * N; i++) {
    
    VectorXd u = u_hat.col(i / ic3_options_.rollout_dt_scaling);

    if (!x_curr.allFinite()) {
      std::cout << "c3 traj gen x_curr not all finite " << x_curr.transpose() << std::endl;
    }

    lcs_factory_.UpdateStateAndInput(x_curr, u);
    LCS lcs = lcs_factory_.GenerateLCS();
    
    // Threshold inputs
    for (int j = 0; j < A_u_.rows(); j++) {
      if (A_u_(j, j) == 1) {
          u(j) = std::clamp(u(j), lb_u_(j), ub_u_(j));
      }
    }
    c3::LCSSimulateConfig config;
    config.regularized = true;
    config.min_exp = -16;
    config.max_exp = -6;
    VectorXd x_next = lcs.Simulate(x_curr, u, config);

    // Threshold states
    for (int j = 0; j < A_x_.rows(); j++) {
      if (A_x_(j, j) == 1) {
        x_next(j) = std::clamp(x_next(j), lb_x_(j), ub_x_(j));
      }
    }

    x_hat.col(i+1) = x_next;
    x_curr = x_next;
    
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

LCS MPCTrajectoryGenerator::CreatePlaceholderLCS() const {
  MatrixXd A = MatrixXd::Ones(n_x_, n_x_);
  MatrixXd B = MatrixXd::Zero(n_x_, n_u_);
  VectorXd d = VectorXd::Zero(n_x_);
  MatrixXd D = MatrixXd::Ones(n_x_, n_lambda_);
  MatrixXd E = MatrixXd::Zero(n_lambda_, n_x_);
  MatrixXd F = MatrixXd::Zero(n_lambda_, n_lambda_);
  MatrixXd H = MatrixXd::Zero(n_lambda_, n_u_);
  VectorXd c = VectorXd::Zero(n_lambda_);
  return LCS(A, B, D, d, E, F, H, c, lcs_factory_options_.N, lcs_factory_options_.dt);
}

}  // namespace dairlib
