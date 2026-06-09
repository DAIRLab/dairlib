#include <iostream>
#include "examples/iC3/visualization/lcm_pose_getter.h"

#include "common/find_resource.h"

namespace dairlib {

LcmPoseGetter::LcmPoseGetter(CubeFlipVisualizerParams& vis_params, std::string name, int example_idx)
: num_trajectories_(vis_params.ic3_num_iters),
  example_idx_(example_idx) {

    this->set_name(name);

    // size hardcoded
    if (example_idx_ == 0) {
      n_x_ = 23;
    } else if (example_idx_ == 1 || example_idx_ == 2) {
      n_x_ = 31;
    }

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
      std::string traj_name = "trajectory_" + std::to_string(i);
      
      auto* port = 
        &this->DeclareVectorOutputPort(
          traj_name,
          n_x_,
          // Calculator: lambda capturing ‘this’ and ‘i’
          [this, i](const drake::systems::Context<double>& context,
                    drake::systems::BasicVector<double>* output) {
            this->GetPose(context, output, i);
          });
        trajectory_output_ports_.push_back(port);
        trajectory_output_port_indices_.push_back(port->get_index());
    }
    std::cout << "num splitter ports: " << trajectory_output_ports_.size() << std::endl;

  }

LcmPoseGetter::LcmPoseGetter(TrajectoryVideoParams& video_params, std::string name, int example_idx)
  : num_trajectories_(1),
    example_idx_(example_idx) {

    this->set_name(name);

    // size hardcoded
    if (example_idx_ == 0) {
      n_x_ = 23;
    } else if (example_idx_ == 1 || example_idx_ == 2) {
      n_x_ = 31;
    }

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
    
    // size hardcoded
    auto* port = 
      &this->DeclareVectorOutputPort(
        traj_name,
        n_x_,
        // Calculator: lambda capturing ‘this’ and ‘i’
        [this, i](const drake::systems::Context<double>& context,
                  drake::systems::BasicVector<double>* output) {
          this->GetPose(context, output, i);
        });

    trajectory_output_ports_.push_back(port);
    trajectory_output_port_indices_.push_back(port->get_index());
    
    std::cout << "num splitter ports video : " << trajectory_output_ports_.size() << std::endl;

  }

  void LcmPoseGetter::GetPose(
    const drake::systems::Context<double>& context, 
    drake::systems::BasicVector<double>* output, int iter) const {
        
    const BasicVector<double>* timestep_vec =
      (BasicVector<double>*)this->EvalVectorInput(context, timestep_input_port_);
    int timestep = static_cast<int>(std::round(timestep_vec->get_value()(0)));
    //std::cout << "timestep " << timestep << std::endl;

    const auto* abstract_input = this->EvalAbstractInput(context, trajectory_input_port_);
    if (abstract_input == nullptr) return;
    
    const auto& lcm_all_trajectories = abstract_input->get_value<lcmt_timestamped_saved_traj>();

    LcmTrajectory trajectory = LcmTrajectory(lcm_all_trajectories.saved_traj);
    
    // Pull out trajectory corresponding to ith iteration
    const std::string trajectory_i_name = "iteration_" + std::to_string(iter);

    if (trajectory.HasTrajectory(trajectory_i_name)) {
      LcmTrajectory::Trajectory trajectory_i = trajectory.GetTrajectory(trajectory_i_name);
      MatrixXd data = trajectory_i.datapoints;

      // std::cout << data.col(timestep).transpose() << std::endl;

      VectorXd x_timestep = data.col(timestep);
      output->SetFromVector(x_timestep); 
    }
    
  }




}