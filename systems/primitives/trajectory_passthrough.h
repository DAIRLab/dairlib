#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

namespace dairlib::systems {

class TrajectoryPassthrough : public drake::systems::LeafSystem<double> {
 public:
  TrajectoryPassthrough(
      const std::string& traj_name, int start_index, int length);

  const drake::systems::InputPort<double>& get_trajectory_input_port() const {
    return this->get_input_port(trajectory_port_);
  }

 private:
  void CalcTraj(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  drake::systems::InputPortIndex trajectory_port_;
  int start_index_;
  int length_;

};

}  // namespace dairlib::systems
