#pragma once

#include <drake/multibody/plant/multibody_plant.h>

#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {

class EndEffectorForceTrajectoryGenerator
    : public drake::systems::LeafSystem<double> {
 public:
  EndEffectorForceTrajectoryGenerator();

  const drake::systems::InputPort<double>& get_input_port_trajectory() const {
    return this->get_input_port(trajectory_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_radio() const {
    return this->get_input_port(radio_port_);
  }

 private:
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void CalcTraj(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  drake::systems::DiscreteStateIndex controller_switch_index_;

  drake::systems::InputPortIndex trajectory_port_;
  drake::systems::InputPortIndex radio_port_;
};

}  // namespace dairlib
