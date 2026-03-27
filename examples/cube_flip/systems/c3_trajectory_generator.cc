#include "examples/cube_flip/systems/c3_trajectory_generator.h"

#include <utility>
#include <iostream>

#include "common/find_resource.h"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "systems/franka_kinematics_vector.h"
#include "multibody/multibody_utils.h"
#include "solvers/c3_output.h"

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


C3TrajectoryGenerator::C3TrajectoryGenerator(
    const drake::multibody::MultibodyPlant<double>& plant, 
      C3Options c3_options, bool track_dynamically_feasible, int example_idx)
    : plant_(plant), 
    c3_options_(std::move(c3_options)), 
    N_(c3_options_.N), 
    track_dynamically_feasible_(track_dynamically_feasible),
    example_idx_(example_idx) {
  this->set_name("c3_trajectory_generator");

  std::cout << "HELLO INIT" << std::endl;

  n_q_ = plant_.num_positions();
  n_v_ = plant_.num_velocities();
  n_x_ = n_q_ + n_v_;
  if (c3_options_.contact_model == "stewart_and_trinkle") {
    n_lambda_ =
        2 * c3_options_.num_contacts +
        2 * c3_options_.num_friction_directions * c3_options_.num_contacts;
  } else if (c3_options_.contact_model == "anitescu") {
    n_lambda_ =
        2 * c3_options_.num_friction_directions * c3_options_.num_contacts;
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
  } else if (example_idx_ == 2) {
    nominal_position_size = 9;
  }

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
  lcs_port_ =
      this->DeclareAbstractInputPort("lcs", drake::Value<LCS>(lcs_placeholder))
          .get_index();
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
  auto& lcs =
    this->EvalAbstractInput(context, lcs_port_)->get_value<LCS>();


  MatrixXd x_hat = c3_solution->x_sol_.cast<double>();
  MatrixXd u_hat = c3_solution->u_sol_.cast<double>();

  if (track_dynamically_feasible_) {
    x_hat = SimulateLCS(x_hat.col(0), u_hat, lcs);
  } else {
    MatrixXd temp(x_hat.rows(), x_hat.cols() + 1);
    temp << x_hat, x_hat.col(x_hat.cols()-1);
    x_hat = temp; 
  }


  auto plant_context = plant_.CreateDefaultContext();		
  VectorXd tau_g = plant_.CalcGravityGeneralizedForces(*plant_context);
  if (example_idx_ == 0) {
    double gravity = tau_g(2); 

    // Gravity not accounted for in external force
    for (int i = 0; i < u_hat.cols(); i++) {
      u_hat.col(i)(2) += gravity;
    }

  } else if (example_idx_ == 2) {

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

  // HARDCODED
  if (example_idx_ == 0) {
    int ee_pos_idx = 0;
    int ee_rot_idx = 3; 
    int force_idx = 0;
    int torque_idx = 3;

    if (N_ == 1) {
      positions = MatrixXd::Zero(3, 2);
      positions.col(0) = x_hat.middleRows(ee_pos_idx, 3).col(0) + nominal_position->get_value();
      positions.col(1) = x_hat.middleRows(ee_pos_idx, 3).col(1) + nominal_position->get_value();

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
      time_vector[1] = time_vector[0] + c3_options_.dt;
      // time_vector[1] = c3_options_.dt;

    } else {
      x_hat = x_hat.rightCols(x_hat.cols() - 1); // Remove x0 for size consistency

      // Reapply offset
      positions = x_hat.middleRows(ee_pos_idx, 3);
      for (int i = 0; i < positions.cols(); i++) {
        positions.col(i) + nominal_position->get_value();
      }
      raw_orientations = x_hat.middleRows(ee_rot_idx, 2);

      forces = u_hat.topRows(3);
      torques = MatrixXd::Zero(3, u_hat.cols());
      torques.topRows(2) = u_hat.bottomRows(2);
      time_vector = c3_solution->time_vector_.cast<double>();
    }
  } else if (example_idx_ == 2) {
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
      time_vector[1] = time_vector[0] + c3_options_.dt;

    } else {
      x_hat = x_hat.rightCols(x_hat.cols() - 1); // Remove x0 for size consistency

      positions = x_hat.middleRows(ee_pos_idx, 9);
      forces = u_hat;
      time_vector = c3_solution->time_vector_.cast<double>();
    }

    // HARDCODED THRESHOLD POSITIONS TO JOINT LIMITS
    for (int i = 0; i < positions.cols(); i++) {
      positions.col(i)(0) = std::min(std::max(positions.col(i)(0), -0.1), 0.1);
      positions.col(i)(1) = std::min(std::max(positions.col(i)(1), -0.03), 0.17);
      positions.col(i)(2) = std::min(std::max(positions.col(i)(2), 0.07), 0.09);
      positions.col(i)(3) = std::min(std::max(positions.col(i)(3), -0.03), 0.17);
      positions.col(i)(4) = std::min(std::max(positions.col(i)(4), -0.16), 0.04);
      positions.col(i)(5) = std::min(std::max(positions.col(i)(5), 0.07), 0.09);
      positions.col(i)(6) = std::min(std::max(positions.col(i)(6), -0.17), 0.03);
      positions.col(i)(7) = std::min(std::max(positions.col(i)(7), -0.16), 0.04);
      positions.col(i)(8) = std::min(std::max(positions.col(i)(8), 0.07), 0.09);
      
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

MatrixXd C3TrajectoryGenerator::SimulateLCS(VectorXd x0, MatrixXd u_hat, LCS lcs) const {

  MatrixXd x_hat(MatrixXd::Zero(x0.size(), lcs.N_ + 1));
  x_hat.col(0) = x0;

  for (int i = 0; i < lcs.N_; i++) {
    x_hat.col(i+1) = lcs.Simulate(x_hat.col(i), u_hat.col(i));
  }
  return x_hat;
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
  return LCS(A, B, D, d, E, F, H, c, c3_options_.N, c3_options_.dt);
}

}  // namespace dairlib
