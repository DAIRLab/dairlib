#include "examples/iC3/systems/c3_trajectory_generator.h"

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

C3TrajectoryGenerator::C3TrajectoryGenerator(
    const drake::multibody::MultibodyPlant<double>& plant, drake::systems::Context<double>* plant_context,
    LCSFactory lcs_factory, vector<SortedPair<GeometryId>> contact_pairs, iC3Options ic3_options, C3ControllerOptions c3_controller_options, 
      bool track_dynamically_feasible, int example_idx, MatrixXd A_x, VectorXd lb_x, VectorXd ub_x, MatrixXd A_u, VectorXd lb_u, VectorXd ub_u)
    : plant_(plant), 
      plant_context_(plant_context),
      lcs_factory_(lcs_factory),
      contact_pairs_(contact_pairs),
      ic3_options_(ic3_options),
      c3_controller_options_(c3_controller_options),
      lcs_factory_options_(c3_controller_options.lcs_factory_options),
      N_(lcs_factory_options_.N), 
      track_dynamically_feasible_(track_dynamically_feasible),
      example_idx_(example_idx),
      A_x_(A_x),
      lb_x_(lb_x),
      ub_x_(ub_x), 
      A_u_(A_u),
      lb_u_(lb_u),
      ub_u_(ub_u) {
  this->set_name("c3_trajectory_generator");

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

  c3_solution_port_ =
      this->DeclareAbstractInputPort("c3_solution",
                                     drake::Value<C3Output::C3Solution>())
          .get_index();

  // HARDCODED
  int nominal_position_size;
  if (example_idx_ == 0) {
    nominal_position_size = 3;
  } else if (example_idx_ == 1 || example_idx_ == 2) {
    nominal_position_size = 9;
  }

  x_lcs_port_ =
      this->DeclareVectorInputPort("x_lcs", TimestampedVector<double>(n_x_))
          .get_index();

  tracking_target_port_ =
      this->DeclareVectorInputPort("tracking target", BasicVector<double>(n_u_))
          .get_index();

  nominal_position_port_ =
      this->DeclareVectorInputPort(
              "nominal_position", BasicVector<double>(nominal_position_size))
          .get_index();    

  actor_trajectory_port_ =
      this->DeclareAbstractOutputPort(
              "c3_actor_trajectory_output",
              dairlib::lcmt_timestamped_saved_traj(),
              &C3TrajectoryGenerator::OutputActorTrajectory)
          .get_index();
  object_trajectory_port_ =
      this->DeclareAbstractOutputPort(
              "c3_object_trajectory_output",
              dairlib::lcmt_timestamped_saved_traj(),
              &C3TrajectoryGenerator::OutputObjectTrajectory)
          .get_index();

  auto lcs_placeholder = CreatePlaceholderLCS();

}

// HARDCODED FOR PLATE EXAMPLE
void C3TrajectoryGenerator::OutputActorTrajectory(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output_traj) const {
  const auto& c3_solution =
      this->EvalInputValue<C3Output::C3Solution>(context, c3_solution_port_);
  DRAKE_DEMAND(c3_solution->x_sol_.rows() == n_q_ + n_v_);
  const BasicVector<double>* nominal_position =
    (BasicVector<double>*)this->EvalVectorInput(context, nominal_position_port_);

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
    std::cout << "c3 x sol " << i << " " << x_hat.col(i).segment(9, 7).transpose() << std::endl;
  }


  // std::cout << "c3 u sol " << u_hat.col(0).transpose() << std::endl;
  // std::cout << "c3 u sol " << u_hat.col(1).transpose() << std::endl;

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


  VectorXd tau_g = plant_.CalcGravityGeneralizedForces(*plant_context_);
  if (example_idx_ == 0) {
    double gravity = tau_g(2); 

    // Gravity not accounted for in external force
    for (int i = 0; i < u_hat.cols(); i++) {
      u_hat.col(i)(2) += gravity;
    }

  } else if (example_idx_ == 1 || example_idx_ == 2) {

    for (int i = 0; i < u_hat.cols(); i++) {
      u_hat.col(i)(2) += (tau_g(2));
      u_hat.col(i)(5) += (tau_g(5));
      u_hat.col(i)(8) += (tau_g(8));
    }

  }

	// Make non-degenerate trajectory for N = 1
  MatrixXd positions;
	MatrixXd raw_orientations;
	MatrixXd forces;
  MatrixXd torques;

  VectorXd time_vector;

  VectorXd nom_pos = nominal_position->get_value();
  
  // HARDCODED
  if (example_idx_ == 0) {
    int ee_pos_idx = 0;
    int ee_rot_idx = 3; 
    int force_idx = 0;
    int torque_idx = 3;

    if (N_ == 1) {
      positions = MatrixXd::Zero(3, 2);
      positions.col(0) = x_hat.middleRows(ee_pos_idx, 3).col(0) + nom_pos;
      positions.col(1) = x_hat.middleRows(ee_pos_idx, 3).col(1) + nom_pos;

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
        positions.col(i) = positions.col(i) + nom_pos;
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

void C3TrajectoryGenerator::OutputObjectTrajectory(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output_traj) const {
  const auto& c3_solution =
      this->EvalInputValue<C3Output::C3Solution>(context, c3_solution_port_);
  DRAKE_DEMAND(c3_solution->x_sol_.rows() == n_q_ + n_v_);
	const BasicVector<double>* nominal_position =
    (BasicVector<double>*)this->EvalVectorInput(context, nominal_position_port_);

  MatrixXd knots = MatrixXd::Zero(6, N_);
  knots.topRows(3) = c3_solution->x_sol_.middleRows(n_q_ - 3, 3).cast<double>();
  knots.bottomRows(3) =
      c3_solution->x_sol_.middleRows(n_q_ + n_v_ - 3, 3).cast<double>();

	// Reapply offset
	for (int i = 0; i < knots.cols(); i++) {
		knots.col(i).segment(0, 3) = knots.col(i).segment(0, 3) + nominal_position->get_value();
	}

  LcmTrajectory::Trajectory object_traj;
  object_traj.traj_name = "object_position_target";
  object_traj.datatypes = std::vector<std::string>(knots.rows(), "double");
  object_traj.datapoints = knots;
  object_traj.time_vector = c3_solution->time_vector_.cast<double>();
  LcmTrajectory lcm_traj({object_traj}, {"object_position_target"},
                         "object_target", "object_target", false);

  LcmTrajectory::Trajectory object_orientation_traj;
  // first 3 rows are rpy, last 3 rows are angular velocity
  MatrixXd orientation_samples = MatrixXd::Zero(4, N_);
  orientation_samples =
      c3_solution->x_sol_.middleRows(n_q_ - 7, 4).cast<double>();
  object_orientation_traj.traj_name = "object_orientation_target";
  object_orientation_traj.datatypes =
      std::vector<std::string>(orientation_samples.rows(), "double");
  object_orientation_traj.datapoints = orientation_samples;
  object_orientation_traj.time_vector =
      c3_solution->time_vector_.cast<double>();
  lcm_traj.AddTrajectory(object_orientation_traj.traj_name,
                         object_orientation_traj);

  output_traj->saved_traj = lcm_traj.GenerateLcmObject();
  output_traj->utime = context.get_time() * 1e6;
}

std::tuple<MatrixXd, MatrixXd> C3TrajectoryGenerator::SimulateLCS(VectorXd x0, MatrixXd u_hat,
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
    
    // TODO: make this if statement less hardcoded
    // Threshold positions and inputs, assume A_x, A_u are diagonal 0/1 matrices
    if ((example_idx_ == 1 || example_idx_ == 2) && ic3_options_.add_constraints_follow_plan) {

      const BasicVector<double>* tracking_target =
        (BasicVector<double>*)this->EvalVectorInput(context, tracking_target_port_);
      
      // std::cout << tracking_target->get_value().transpose() << std::endl;

      // Hardcoded indices
      for (int f = 0; f < 3; f++) {
        A_x_(3*f, 3*f) = 1;
        A_x_(3*f+1, 3*f+1) = 1;

        lb_x_(3*f) = tracking_target->get_value()(3*f) - 0.01;
        lb_x_(3*f+1) = tracking_target->get_value()(3*f+1) - 0.01;
        ub_x_(3*f) = tracking_target->get_value()(3*f) + 0.01;
        ub_x_(3*f+1) = tracking_target->get_value()(3*f+1) + 0.01;
      }
    } 

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

LCS C3TrajectoryGenerator::CreatePlaceholderLCS() const {
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
