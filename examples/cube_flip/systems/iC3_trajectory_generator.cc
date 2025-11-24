#include "iC3_trajectory_generator.h"

#include <iostream>
#include <utility>

#include "common/find_resource.h"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "multibody/multibody_utils.h"
#include "solvers/c3_output.h"
#include "solvers/lcs.h"

namespace dairlib {

using drake::multibody::ModelInstanceIndex;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::VectorXd;
using solvers::LCS;

iC3TrajectoryGenerator::iC3TrajectoryGenerator(
    const drake::multibody::MultibodyPlant<double>& plant, iC3Options ic3_options)
    : plant_(plant), // Not used
		ic3_options_(std::move(ic3_options)), 
		num_iters_(ic3_options.num_iters),
		N_(ic3_options.N),
		dt_(ic3_options.dt) {
  this->set_name("iC3_trajectory_generator");

  n_q_ = plant_.num_positions();
  n_v_ = plant_.num_velocities();
  n_x_ = n_q_ + n_v_;
  n_u_ = plant_.num_actuators();
	
  nominal_input_port_ =
    this->DeclareVectorInputPort("nominal_input",
                                    BasicVector<double>(3))
        	.get_index();

  ic3_x_trajectory_port_ =
      this->DeclareAbstractInputPort("iC3_x_trajectory",
                                     drake::Value<lcmt_timestamped_saved_traj>())
          .get_index();

	ic3_u_trajectory_port_ =
      this->DeclareAbstractInputPort("iC3_u_trajectory",
                                     drake::Value<lcmt_timestamped_saved_traj>())
          .get_index();

  actor_trajectory_port_ =
      this->DeclareAbstractOutputPort(
              "iC3_actor_trajectory_output",
              dairlib::lcmt_timestamped_saved_traj(),
              &iC3TrajectoryGenerator::OutputActorTrajectory)
          .get_index();

  object_trajectory_port_ =
      this->DeclareAbstractOutputPort(
              "iC3_object_trajectory_output",
              dairlib::lcmt_timestamped_saved_traj(),
              &iC3TrajectoryGenerator::OutputObjectTrajectory)
          .get_index();

	this->DeclarePerStepDiscreteUpdateEvent(
			&iC3TrajectoryGenerator::SetFirstCallTime);

	t0_idx_ = this->DeclareDiscreteState(1); // first time output called
	called_ = false;

}

void iC3TrajectoryGenerator::OutputActorTrajectory(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output_traj) const {

	double t0 = context.get_discrete_state(t0_idx_).GetAtIndex(0);

  const auto* x_input = this->EvalAbstractInput(context, ic3_x_trajectory_port_);
	const auto* u_input = this->EvalAbstractInput(context, ic3_u_trajectory_port_);
	
	drake::VectorX<double> nominal_position = this->EvalVectorInput(context, nominal_input_port_)->get_value();

  if (x_input == nullptr || u_input == nullptr) return;

	int segment_idx = (int) std::max(((context.get_time() - t0) - 3) / (dt_ * 1), 0.0);

	// Extract trajectories from lcm
	const auto& lcm_all_x_trajectories = x_input->get_value<lcmt_timestamped_saved_traj>();
  const auto& lcm_all_u_trajectories = u_input->get_value<lcmt_timestamped_saved_traj>();
	LcmTrajectory x_trajectory = LcmTrajectory(lcm_all_x_trajectories.saved_traj);
  LcmTrajectory u_trajectory = LcmTrajectory(lcm_all_u_trajectories.saved_traj);

	const std::string final_trajectory_name = "iteration_" + std::to_string(num_iters_ - 1);
    
	const std::string orientation_trajectory_name = "end_effector_orientation_target";
	const std::string position_trajectory_name = "end_effector_position_target";
	const std::string force_trajectory_name = "end_effector_force_target";

  if ((context.get_time() - t0) <= 3 || (context.get_time() - t0) - 3 > N_ * dt_ * 1) {

		MatrixXd positions = nominal_position.replicate(1, 5);
	
		MatrixXd orientations = MatrixXd::Zero(4, 5);
		MatrixXd forces = MatrixXd::Zero(3, 5);

		VectorXd timestamps(5);
    for (int t = 0; t < 5; t++) {
		  orientations(0, t) = 1;
			// orientations(0, t) = 0.968718;
			// orientations(2, t) = -0.248163;


      timestamps(t) = t * dt_;
    }

		LcmTrajectory::Trajectory position_traj;
    position_traj.traj_name = position_trajectory_name;
    position_traj.datatypes = std::vector<std::string>(positions.rows(), "double"); 
    position_traj.datapoints = positions;
    position_traj.time_vector = timestamps;

		LcmTrajectory::Trajectory orientation_traj;
    orientation_traj.traj_name = orientation_trajectory_name;
    orientation_traj.datatypes = std::vector<std::string>(orientations.rows(), "double"); 
    orientation_traj.datapoints = orientations;
    orientation_traj.time_vector = timestamps;

		LcmTrajectory::Trajectory force_traj;
    force_traj.traj_name = force_trajectory_name;
    force_traj.datatypes = std::vector<std::string>(forces.rows(), "double"); 
    force_traj.datapoints = forces;
    force_traj.time_vector = timestamps;

    LcmTrajectory lcm_trajectory({position_traj}, {position_trajectory_name},
                                position_trajectory_name, position_trajectory_name, false);
		lcm_trajectory.AddTrajectory(orientation_trajectory_name, orientation_traj);   
	  lcm_trajectory.AddTrajectory(force_trajectory_name, force_traj);   

		output_traj->saved_traj = lcm_trajectory.GenerateLcmObject();
    output_traj->utime = context.get_time() * 1e6;

  } else if ((context.get_time() - t0) - 3 < N_ * dt_ * 1 && 
    x_trajectory.HasTrajectory(final_trajectory_name) && 
      u_trajectory.HasTrajectory(final_trajectory_name)) {

		std::cout << "tracking ic3" << std::endl;

    // Get ee position/orientation trajectories
    LcmTrajectory::Trajectory trajectory = x_trajectory.GetTrajectory(final_trajectory_name);
    MatrixXd data = trajectory.datapoints;

    LcmTrajectory::Trajectory force_trajectory = u_trajectory.GetTrajectory(final_trajectory_name);
    MatrixXd force_data = force_trajectory.datapoints;

    int ee_pos_idx = 0;
    int ee_rot_idx = 3; 

    MatrixXd raw_orientations = data.middleRows(ee_rot_idx, 2);
    MatrixXd full_positions = data.middleRows(ee_pos_idx, 3);
		MatrixXd full_forces = force_data.middleRows(0, 3);

    // std::cout << "raw orientation size: " << raw_orientations.rows() << " x " << raw_orientations.cols() << std::endl;
    // std::cout << "full position size: " << full_positions.rows() << " x " << full_positions.cols() << std::endl;

    MatrixXd orientations = MatrixXd::Zero(4, 5);
    MatrixXd positions = MatrixXd::Zero(3, 5);
    MatrixXd forces = MatrixXd::Zero(3, 5);
    for (int i = segment_idx; i < segment_idx + 5; ++i) {
				int idx;	
				if (i > N_) {
					idx = N_;
				} else {
					idx = i;
				}

        double roll = raw_orientations(0, idx);
        double pitch = raw_orientations(1, idx);

        Eigen::AngleAxisd rollAngle(roll, Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Vector3d::UnitY());

        Eigen::Quaterniond q = rollAngle * pitchAngle; 
        VectorXd q_vec(4); 
        q_vec << q.w(), q.x(), -q.y(), q.z();

        orientations.col(idx - segment_idx) = q_vec;

				// iC3 solves about origin so add offset
        positions.col(idx - segment_idx) = full_positions.col(idx) + nominal_position;
				forces.col(idx - segment_idx) = full_forces.col(idx);
    }

    VectorXd timestamps(5);
    for (int t = 0; t < 5; t++) {
      timestamps(t) = t * dt_ * 1;
    }

    LcmTrajectory::Trajectory orientation_traj;
    orientation_traj.traj_name = orientation_trajectory_name;
    orientation_traj.datatypes = std::vector<std::string>(orientations.rows(), "double"); 
    orientation_traj.datapoints = orientations;
    orientation_traj.time_vector = timestamps;

		std::cout << segment_idx << std::endl;
		std::cout << orientations.col(0).transpose() << std::endl << std::endl;

    LcmTrajectory::Trajectory position_traj;
    position_traj.traj_name = position_trajectory_name;
    position_traj.datatypes = std::vector<std::string>(positions.rows(), "double"); 
    position_traj.datapoints = positions;
    position_traj.time_vector = timestamps;

    LcmTrajectory::Trajectory force_traj;
    force_traj.traj_name = force_trajectory_name;
    force_traj.datatypes = std::vector<std::string>(forces.rows(), "double"); 
    force_traj.datapoints = forces;
    force_traj.time_vector = timestamps;


    LcmTrajectory lcm_trajectory({orientation_traj}, {orientation_trajectory_name},
                                orientation_trajectory_name, orientation_trajectory_name, false);
    lcm_trajectory.AddTrajectory(position_trajectory_name, position_traj);   
    lcm_trajectory.AddTrajectory(force_trajectory_name, force_traj);   

    output_traj->saved_traj = lcm_trajectory.GenerateLcmObject();
    output_traj->utime = context.get_time() * 1e6;
  }


}

void iC3TrajectoryGenerator::OutputObjectTrajectory(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output_traj) const {

	const auto* x_input = this->EvalAbstractInput(context, ic3_x_trajectory_port_);
  if (x_input == nullptr) return;

	// Extract trajectories from lcm
	const auto& lcm_all_x_trajectories = x_input->get_value<lcmt_timestamped_saved_traj>();
	LcmTrajectory x_trajectory = LcmTrajectory(lcm_all_x_trajectories.saved_traj);

	const std::string final_trajectory_name = "iteration_" + std::to_string(num_iters_ - 1);

	if (x_trajectory.HasTrajectory(final_trajectory_name)) {

		  const std::string orientation_trajectory_name = "object_orientation_target";
		  const std::string position_trajectory_name = "object_position_target";

			// Get ee position/orientation trajectories
			LcmTrajectory::Trajectory trajectory = x_trajectory.GetTrajectory(final_trajectory_name);
      MatrixXd data = trajectory.datapoints;

			int pos_idx = 9;
			int rot_idx = 5; 

      MatrixXd orientations = data.middleRows(rot_idx, 4);
      MatrixXd positions = data.middleRows(pos_idx, 3);

      VectorXd timestamps(N_+1);
      for (int t = 0; t < N_+1; t++) {
        timestamps(t) = t * dt_;
      }

			LcmTrajectory::Trajectory orientation_traj;
      orientation_traj.traj_name = orientation_trajectory_name;
      orientation_traj.datatypes = std::vector<std::string>(orientations.rows(), "double"); 
      orientation_traj.datapoints = orientations;
      orientation_traj.time_vector = timestamps;

      LcmTrajectory::Trajectory position_traj;
      position_traj.traj_name = position_trajectory_name;
      position_traj.datatypes = std::vector<std::string>(positions.rows(), "double"); 
      position_traj.datapoints = positions;
      position_traj.time_vector = timestamps;

      LcmTrajectory lcm_trajectory({orientation_traj}, {orientation_trajectory_name},
                                  orientation_trajectory_name, orientation_trajectory_name, false);
      lcm_trajectory.AddTrajectory(position_trajectory_name, position_traj);   

			output_traj->saved_traj = lcm_trajectory.GenerateLcmObject();
			output_traj->utime = context.get_time() * 1e6;
	}
}

drake::systems::EventStatus iC3TrajectoryGenerator::SetFirstCallTime(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const {
  auto& vec = discrete_state->get_mutable_vector(t0_idx_);
  if (!called_) {  
    vec.SetAtIndex(0, context.get_time());
		called_ = true;
  }
  return drake::systems::EventStatus::Succeeded();
}

}  // namespace dairlib
