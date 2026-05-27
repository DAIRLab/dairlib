#pragma once

#include <string>
#include <vector>

#include "lcm/lcm_trajectory.h"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "examples/iC3/visualization/parameter_headers/trajectory_visualizer_params.h"

#include <drake/multibody/plant/multibody_plant.h>
#include "drake/systems/framework/leaf_system.h"
#include <Eigen/Geometry>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::AngleAxisd;
using Eigen::Vector3d;
using Eigen::Quaterniond;

using drake::systems::InputPort;
using drake::systems::OutputPort; 
using drake::systems::InputPortIndex;
using drake::systems::OutputPortIndex; 
using drake::systems::Context;

using dairlib::LcmTrajectory;
using dairlib::lcmt_timestamped_saved_traj;

namespace dairlib {

class TrajectoryLcmParserPlate : public drake::systems::LeafSystem<double> {
  public:
    // object: 0 = cube, 1 = plate
    explicit TrajectoryLcmParserPlate(CubeFlipVisualizerParams& vis_params, int object, std::string name);

    const InputPort<double>& get_input_port_trajectory() const {
      return this->get_input_port(trajectory_input_port_);
    }

    const OutputPort<double>& get_output_port(int index) const {
      //return this->get_output_port(trajectory_output_ports_.at(index));
      return *trajectory_output_ports_.at(index);
    }

  private: 
    int num_trajectories_;
    int N_;
    
    InputPortIndex trajectory_input_port_;
    std::vector<OutputPort<double>*> trajectory_output_ports_;
    std::vector<OutputPortIndex> trajectory_output_port_indices_;

    // Parse raw iC3 output into cube position/orientation trajectories
    void GetTrajectory(
      const drake::systems::Context<double>& context, 
      lcmt_timestamped_saved_traj* traj, int i, int step, int object) const;
};

} // namespace dairlib