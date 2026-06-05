#include "timed_gate.h"

#include <iostream>
#include <utility>

#include "common/find_resource.h"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "multibody/multibody_utils.h"


namespace dairlib {

TimedGate::TimedGate(iC3Options ic3_options, int example_idx) : 
  ic3_options_(ic3_options),
  hold_final_position_(ic3_options.hold_last_timestep_position),
  N_(ic3_options.N),
  dt_(ic3_options.dt),
  example_idx_(example_idx) { 

  this->set_name("timed_gate");

  c3_actor_port_ =
      this->DeclareAbstractInputPort("c3_actor",
                                     drake::Value<lcmt_timestamped_saved_traj>())
          .get_index();

  // HARDCODED
  int nominal_position_size;
  if (example_idx_ == 0) {
    nominal_position_size = 3;
  } else if (example_idx_ == 1) {
    nominal_position_size = 9;
  }          
  
  nominal_position_port_ =
      this->DeclareVectorInputPort(
              "nominal_position", BasicVector<double>(nominal_position_size))
          .get_index();

  ic3_x_trajectory_port_ =
      this->DeclareAbstractInputPort("iC3_x_trajectory",
                                     drake::Value<lcmt_timestamped_saved_traj>())
          .get_index();

  radio_port_ =
      this->DeclareAbstractInputPort("lcmt_radio_out",
                                     drake::Value<dairlib::lcmt_radio_out>{})
          .get_index();

  timestep_input_port_ =
      this->DeclareVectorInputPort(
              "timestep", BasicVector<double>(1))
          .get_index();

  actor_output_port_ =
      this->DeclareAbstractOutputPort(
              "actor_trajectory_output",
              dairlib::lcmt_timestamped_saved_traj(),
              &TimedGate::OutputActorTrajectory)
          .get_index();


}

void TimedGate::OutputActorTrajectory(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output_traj) const {

  const auto* radio_out =
    this->EvalInputValue<lcmt_radio_out>(context, radio_port_);

  // If in teleop, don't do anything
  if (radio_out->channel[14]) {
    std::cout << "IN TELEOP" << std::endl;
    return;
  }

  const auto* c3_actor_input_lcm = this->EvalAbstractInput(context, c3_actor_port_);
	const auto& c3_actor_input = c3_actor_input_lcm->get_value<lcmt_timestamped_saved_traj>();
  const auto* x_input = this->EvalAbstractInput(context, ic3_x_trajectory_port_);
	const auto& lcm_all_x_trajectories = x_input->get_value<lcmt_timestamped_saved_traj>();
	LcmTrajectory x_trajectory = LcmTrajectory(lcm_all_x_trajectories.saved_traj);

  const BasicVector<double>* nominal_position =
    (BasicVector<double>*)this->EvalVectorInput(context, nominal_position_port_);

  const BasicVector<double>* timestep_vector =
    (BasicVector<double>*)this->EvalVectorInput(context, timestep_input_port_);

  int timestep = static_cast<int>(timestep_vector->get_value()(0));

  std::string final_trajectory_name = "iteration_" + std::to_string(ic3_options_.iter_to_use);
	const std::string position_trajectory_name = "end_effector_position_target";
	const std::string force_trajectory_name = "end_effector_force_target";
  const std::string orientation_trajectory_name = "end_effector_orientation_target";
  const std::string torque_trajectory_name = "end_effector_torque_target";

  
  if (0 <= timestep && timestep < N_) {
    *output_traj = c3_actor_input;    

    // Debugging
	  LcmTrajectory c3_traj = LcmTrajectory(c3_actor_input.saved_traj);

    MatrixXd c3_position_data = c3_traj.GetTrajectory("end_effector_position_target").datapoints;
    MatrixXd c3_force_data = c3_traj.GetTrajectory("end_effector_force_target").datapoints;

    MatrixXd c3_orientation_data;
    MatrixXd c3_torque_data;

    if (example_idx_ == 0) {
      c3_orientation_data = c3_traj.GetTrajectory("end_effector_orientation_target").datapoints;
      c3_torque_data = c3_traj.GetTrajectory("end_effector_torque_target").datapoints;

      std::cout << "timed gate x " << c3_orientation_data.col(0).transpose() << "; " << c3_position_data.col(0).transpose() << std::endl;
      std::cout << "timed gate x " << c3_orientation_data.col(1).transpose() << "; " << c3_position_data.col(1).transpose() << std::endl;

      std::cout << "timed gate u " << c3_force_data.col(0).transpose() << "; " << c3_torque_data.col(0).transpose() << std::endl;
      std::cout << "timed gate u " << c3_force_data.col(1).transpose() << "; " << c3_torque_data.col(1).transpose() << std::endl;
    } else if (example_idx_ == 1) {
      std::cout << "timed gate x " << c3_position_data.col(0).transpose() << std::endl;
      std::cout << "timed gate x " << c3_position_data.col(1).transpose() << std::endl;

      std::cout << "timed gate u " << c3_force_data.col(0).transpose() << std::endl;
      std::cout << "timed gate u " << c3_force_data.col(1).transpose() << std::endl;
    }



  } else {
    // HARDCODED
    if (example_idx_ == 0) {
      int plate_position_idx = 0;
      VectorXd nominal_position_offset = nominal_position->get_value();
      if (timestep > N_) {
        if (hold_final_position_) {
          LcmTrajectory::Trajectory trajectory = x_trajectory.GetTrajectory(final_trajectory_name);
          VectorXd last_x = trajectory.datapoints.col(trajectory.datapoints.cols() - 1);
          nominal_position_offset = nominal_position->get_value() + last_x.segment(plate_position_idx, 3);
        } else {
          nominal_position_offset(2) += ic3_options_.vertical_offset;
        }
      }

      MatrixXd positions = nominal_position_offset.replicate(1, 2);
      MatrixXd orientations = MatrixXd::Zero(4, 2);
      MatrixXd forces = MatrixXd::Zero(3, 2);
      MatrixXd torques = MatrixXd::Zero(3, 2);

      VectorXd timestamps(2);
      for (int t = 0; t < 2; t++) {
        orientations(0, t) = 1;
        timestamps(t) = t * 1.0;
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

      LcmTrajectory::Trajectory torque_traj;
      torque_traj.traj_name = torque_trajectory_name;
      torque_traj.datatypes = std::vector<std::string>(torques.rows(), "double"); 
      torque_traj.datapoints = torques;
      torque_traj.time_vector = timestamps;

      LcmTrajectory lcm_trajectory({position_traj}, {position_trajectory_name},
                                  position_trajectory_name, position_trajectory_name, false);
      lcm_trajectory.AddTrajectory(orientation_trajectory_name, orientation_traj);   
      lcm_trajectory.AddTrajectory(force_trajectory_name, force_traj);   
      lcm_trajectory.AddTrajectory(torque_trajectory_name, torque_traj);   

      output_traj->saved_traj = lcm_trajectory.GenerateLcmObject();
      output_traj->utime = context.get_time() * 1e6;

    } else if (example_idx_ == 1) {
      VectorXd nominal_position_offset = nominal_position->get_value();

      if (timestep > N_) {
        if (hold_final_position_) {
          LcmTrajectory::Trajectory trajectory = x_trajectory.GetTrajectory(final_trajectory_name);
          VectorXd last_x = trajectory.datapoints.col(trajectory.datapoints.cols() - 1);
          nominal_position_offset = last_x.segment(0, 9);
        } 
      }

      MatrixXd positions = nominal_position_offset.replicate(1, 2);
      MatrixXd forces = MatrixXd::Zero(9, 2);

      VectorXd timestamps(2);
      for (int t = 0; t < 2; t++) {
        timestamps(t) = t * 1.0;
      }
      
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

      LcmTrajectory lcm_trajectory({position_traj}, {position_trajectory_name},
                                  position_trajectory_name, position_trajectory_name, false);
      lcm_trajectory.AddTrajectory(force_trajectory_name, force_traj);   

      output_traj->saved_traj = lcm_trajectory.GenerateLcmObject();
      output_traj->utime = context.get_time() * 1e6;
    }
    
  } 
}


} // namespace dairlib