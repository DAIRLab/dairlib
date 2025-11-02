#pragma once

#include <string>
#include <vector>

#include "lcm/lcm_trajectory.h"

#include "dairlib/lcmt_timestamped_saved_traj.hpp"

#include <drake/multibody/plant/multibody_plant.h>
#include "drake/systems/framework/leaf_system.h"

using Eigen::VectorXd;
using drake::systems::InputPort;
using drake::systems::OutputPort; 
using drake::systems::InputPortIndex;
using drake::systems::OutputPortIndex; 
using drake::systems::Context;

using dairlib::LcmTrajectory;

namespace dairlib {


class TrajectoryLcmParser : public drake::systems::LeafSystem<double> {
  public:
    explicit TrajectoryLcmParser(int num_trajectories, int N);

    const InputPort<double>& get_input_port_trajectory() const {
      return this->get_input_port(trajectory_input_port_);
    }

    const OutputPort<double>& get_output_port(int index) const {
      return this->get_output_port(trajectory_output_ports_.at(index));
    }

  private: 
    int num_trajectories_;
    int N_;
    
    InputPortIndex trajectory_input_port_;
    std::vector<OutputPortIndex> trajectory_output_ports_;

    void GetTrajectory(
      const drake::systems::Context<double>& context, 
      lcmt_timestamped_saved_traj* traj, int i);
};

} // namespace dairlib