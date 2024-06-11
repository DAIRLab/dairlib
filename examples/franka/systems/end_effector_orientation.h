#pragma once

#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {

class EndEffectorOrientationGenerator
    : public drake::systems::LeafSystem<double> {
 public:
  EndEffectorOrientationGenerator();

  const drake::systems::InputPort<double>& get_input_port_trajectory() const {
    return this->get_input_port(trajectory_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_radio() const {
    return this->get_input_port(radio_port_);
  }

  void SetTrackOrientation(bool track_orientation) {
    track_orientation_ = track_orientation;
  }

 private:
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void CalcTraj(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  bool track_orientation_;

  drake::systems::InputPortIndex trajectory_port_;
  drake::systems::InputPortIndex radio_port_;
};

}  // namespace dairlib
