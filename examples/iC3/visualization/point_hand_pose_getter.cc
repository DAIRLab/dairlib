#include <iostream>
#include "examples/iC3/visualization/point_hand_pose_getter.h"

#include "common/find_resource.h"

namespace dairlib {

PointHandPoseGetter::PointHandPoseGetter(CubeFlipVisualizerParams& vis_params, 
    int is_finger, std::string name)
: num_trajectories_(vis_params.ic3_num_iters) {

    this->set_name(name);

    trajectory_input_port_ =
        this->DeclareAbstractInputPort(
            "all_trajectories",
              drake::Value<lcmt_timestamped_saved_traj>{})
        .get_index();

    timestep_input_port_ =
        this->DeclareVectorInputPort(
            "timestep",
              BasicVector<double>(1))
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
          [this, i, is_finger](const drake::systems::Context<double>& context,
                    drake::AbstractValue* output_abstract) {
            lcmt_timestamped_saved_traj& traj = output_abstract->get_mutable_value<lcmt_timestamped_saved_traj>();
            this->GetTrajectory(context, &traj, i, is_finger);
          });

      trajectory_output_ports_.push_back(port);
      trajectory_output_port_indices_.push_back(port->get_index());
    }
    std::cout << "num splitter ports: " << trajectory_output_ports_.size() << std::endl;

  }

PointHandPoseGetter::PointHandPoseGetter(TrajectoryVideoParams& video_params, 
  int is_finger, std::string name)
  : num_trajectories_(1) {

    this->set_name(name);

    trajectory_input_port_ =
        this->DeclareAbstractInputPort(
            "all_trajectories",
              drake::Value<lcmt_timestamped_saved_traj>{})
        .get_index();

    timestep_input_port_ =
        this->DeclareVectorInputPort(
            "timestep",
              BasicVector<double>(1))
        .get_index();
    
    int i = video_params.ic3_iter;

    std::string traj_name = "trajectory_" + std::to_string(i);
    
    auto* port = 
      &this->DeclareAbstractOutputPort(
        traj_name,
        // Allocator: create the concrete OutputType (lcmt_timestamped_saved_traj) in a std::unique_ptr<AbstractValue>
        []() {
          return std::make_unique<drake::Value<lcmt_timestamped_saved_traj>>(lcmt_timestamped_saved_traj());
        },
        // Calculator: lambda capturing ‘this’ and ‘i’
        [this, i, is_finger](const drake::systems::Context<double>& context,
                  drake::AbstractValue* output_abstract) {
          lcmt_timestamped_saved_traj& traj = output_abstract->get_mutable_value<lcmt_timestamped_saved_traj>();
          this->GetTrajectory(context, &traj, i, is_finger);
        });

    trajectory_output_ports_.push_back(port);
    trajectory_output_port_indices_.push_back(port->get_index());
    
    std::cout << "num splitter ports video : " << trajectory_output_ports_.size() << std::endl;

  }

  void PointHandPoseGetter::GetTrajectory(
    const drake::systems::Context<double>& context, 
    lcmt_timestamped_saved_traj* traj, int iter, int is_finger) const {
        
    const BasicVector<double>* timestep_vec =
      (BasicVector<double>*)this->EvalVectorInput(context, timestep_input_port_);
    int timestep = static_cast<int>(std::round(timestep_vec->get_value()(0)));
      

    const auto* abstract_input = this->EvalAbstractInput(context, trajectory_input_port_);
    if (abstract_input == nullptr) return;
    
    const auto& lcm_all_trajectories = abstract_input->get_value<lcmt_timestamped_saved_traj>();

    LcmTrajectory trajectory = LcmTrajectory(lcm_all_trajectories.saved_traj);
    
    // Pull out trajectory corresponding to ith iteration
    const std::string trajectory_i_name = "iteration_" + std::to_string(iter);
    //std::cout << trajectory_i_name << std::endl;

    if (trajectory.HasTrajectory(trajectory_i_name)) {
      LcmTrajectory::Trajectory trajectory_i = trajectory.GetTrajectory(trajectory_i_name);
      MatrixXd data = trajectory_i.datapoints;

      const std::string orientation_trajectory_name = "orientations_" + std::to_string(iter);
      const std::string position_trajectory_name = "positions_" + std::to_string(iter);

      MatrixXd orientations = MatrixXd::Zero(4, 3);
      orientations(0, 0) = 1;
      orientations(0, 1) = 1;
      orientations(0, 2) = 1;

      MatrixXd positions;
      if (is_finger == 0) {
        int cube_orientation_index = 9;
        int cube_position_index = 13;

        positions = MatrixXd::Zero(3, 3);
        orientations.col(0) = data.col(timestep).segment(cube_orientation_index, 4);
        orientations.col(1) = data.col(timestep).segment(cube_orientation_index, 4);
        orientations.col(2) = data.col(timestep).segment(cube_orientation_index, 4);

        positions.col(0) = data.col(timestep).segment(cube_position_index, 3);
        positions.col(1) = data.col(timestep).segment(cube_position_index, 3);
        positions.col(2) = data.col(timestep).segment(cube_position_index, 3);

      } else if (is_finger == 1) {
        int finger_position_index = 0;    
        positions = MatrixXd::Zero(9, 3);
        positions.col(0) = data.col(timestep).segment(finger_position_index, 9);
        positions.col(1) = data.col(timestep).segment(finger_position_index, 9);
        positions.col(2) = data.col(timestep).segment(finger_position_index, 9);

      } else {
        std::cout << "BAD OBJECT INDEX SIDOGHPOISJDGPWE" << std::endl;
      }

      VectorXd timestamps(VectorXd::Zero(3));
      timestamps(1) = 1.0;
      timestamps(2) = 2.0;

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

      LcmTrajectory lcm_trajectory_i({orientation_traj}, {orientation_trajectory_name},
                                  orientation_trajectory_name, orientation_trajectory_name, false);
      lcm_trajectory_i.AddTrajectory(position_traj.traj_name, position_traj);                    

      traj->saved_traj = lcm_trajectory_i.GenerateLcmObject();
      traj->utime = context.get_time() * 1e6;
      
    }
    
  }




}