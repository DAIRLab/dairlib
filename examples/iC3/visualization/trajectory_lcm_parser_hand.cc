#include <iostream>
#include "examples/iC3/visualization/trajectory_lcm_parser_hand.h"

#include "common/find_resource.h"

namespace dairlib {

TrajectoryLcmParserHand::TrajectoryLcmParserHand(CubeFlipVisualizerParams& vis_params, 
    int is_finger, std::string name)
: num_trajectories_(vis_params.ic3_num_iters),
  N_(vis_params.trajectory_length) {

    this->set_name(name);

    trajectory_input_port_ =
        this->DeclareAbstractInputPort(
            "all_trajectories",
              drake::Value<lcmt_timestamped_saved_traj>{})
        .get_index();
    
    int skip_factor = vis_params.iter_downsampling_factor;
    for (int i = skip_factor-1; i < num_trajectories_; i += skip_factor) {
      std::string name = "trajectory_" + std::to_string(i);
      
      auto* port = 
        &this->DeclareAbstractOutputPort(
          name,
          // Allocator: create the concrete OutputType (lcmt_timestamped_saved_traj) in a std::unique_ptr<AbstractValue>
          []() {
            return std::make_unique<drake::Value<lcmt_timestamped_saved_traj>>(lcmt_timestamped_saved_traj());
          },
          // Calculator: lambda capturing ‘this’ and ‘i’
          [this, i, vis_params, is_finger](const drake::systems::Context<double>& context,
                    drake::AbstractValue* output_abstract) {
            lcmt_timestamped_saved_traj& traj = output_abstract->get_mutable_value<lcmt_timestamped_saved_traj>();
            int step = vis_params.downsampling_factor;
            this->GetTrajectory(context, &traj, i, step, is_finger);
          });

      trajectory_output_ports_.push_back(port);
      trajectory_output_port_indices_.push_back(port->get_index());
    }
    std::cout << "num splitter ports: " << trajectory_output_ports_.size() << std::endl;

  }

  void TrajectoryLcmParserHand::GetTrajectory(
    const drake::systems::Context<double>& context, 
    lcmt_timestamped_saved_traj* traj, int i, int step, int is_finger) const {

    const auto* abstract_input = this->EvalAbstractInput(context, trajectory_input_port_);
    if (abstract_input == nullptr) return;
    
    const auto& lcm_all_trajectories = abstract_input->get_value<lcmt_timestamped_saved_traj>();

    LcmTrajectory trajectory = LcmTrajectory(lcm_all_trajectories.saved_traj);
    
    // Pull out trajectory corresponding to ith iteration
    const std::string trajectory_i_name = "iteration_" + std::to_string(i);
    //std::cout << trajectory_i_name << std::endl;

    if (trajectory.HasTrajectory(trajectory_i_name)) {
      const std::string orientation_trajectory_name = "orientations_" + std::to_string(i);
      const std::string position_trajectory_name = "positions_" + std::to_string(i);
      
      LcmTrajectory::Trajectory trajectory_i = trajectory.GetTrajectory(trajectory_i_name);
      MatrixXd data = trajectory_i.datapoints;

      MatrixXd orientations = MatrixXd::Zero(4, N_);
      for (int i = 0; i < N_; i++) {
        orientations.col(i)(0) = 1;
      }
      MatrixXd positions;
      if (is_finger == 0) {
        int cube_orientation_index = 16;
        int cube_position_index = 20;

        positions = MatrixXd::Zero(3, N_);
        orientations = data.block(cube_orientation_index, 0, 4, N_);
        positions = data.block(cube_position_index, 0, 3, N_);

      } else if (is_finger == 1) {
        int finger_position_index = 0;    
        positions = MatrixXd::Zero(16, N_);
        positions = data.block(finger_position_index, 0, 16, N_);

      } else {
        std::cout << "BAD OBJECT INDEX SIDOGHPOISJDGPWE" << std::endl;
      }

      int downsampled_cols =  (orientations.cols() + step - 2) / step;
      MatrixXd orientations_downsampled(orientations.rows(), downsampled_cols);
      MatrixXd positions_downsampled(positions.rows(), downsampled_cols);

      for (int i = 0; i < downsampled_cols; i++) {
          orientations_downsampled.col(i) = orientations.col(i * step);
          positions_downsampled.col(i) = positions.col(i * step);
      }

      VectorXd timestamps(downsampled_cols);
      for (int t = 0; t < downsampled_cols; t++) {
        timestamps(t) = t;
      }

      LcmTrajectory::Trajectory orientation_traj;
      orientation_traj.traj_name = orientation_trajectory_name;
      orientation_traj.datatypes = std::vector<std::string>(orientations_downsampled.rows(), "double"); 
      orientation_traj.datapoints = orientations_downsampled;
      orientation_traj.time_vector = timestamps;

      LcmTrajectory::Trajectory position_traj;
      position_traj.traj_name = position_trajectory_name;
      position_traj.datatypes = std::vector<std::string>(positions_downsampled.rows(), "double"); 
      position_traj.datapoints = positions_downsampled;
      position_traj.time_vector = timestamps;

      LcmTrajectory lcm_trajectory_i({orientation_traj}, {orientation_trajectory_name},
                                  orientation_trajectory_name, orientation_trajectory_name, false);
      lcm_trajectory_i.AddTrajectory(position_traj.traj_name, position_traj);                    

      traj->saved_traj = lcm_trajectory_i.GenerateLcmObject();
      traj->utime = context.get_time() * 1e6;
      
    }
    
  }




}