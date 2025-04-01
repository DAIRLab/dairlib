#pragma once

#include <drake/multibody/plant/multibody_plant.h>

#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {

class EndEffectorOrientationGenerator : public drake::systems::LeafSystem<double> {
 public:
  EndEffectorOrientationGenerator(const drake::multibody::MultibodyPlant<double>& plant,
                                 drake::systems::Context<double>* context);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_trajectory() const {
    return this->get_input_port(trajectory_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_radio() const {
    return this->get_input_port(radio_port_);
  }

  void SetTrackOrientation(bool track_orientation){
    track_orientation_ = track_orientation;
  }

 private:
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;
  drake::trajectories::PiecewiseQuaternionSlerp<double> GeneratePose(
      const drake::systems::Context<double>& context) const;

  void CalcTraj(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  const drake::multibody::Frame<double>& world_;

  bool track_orientation_;

  drake::systems::InputPortIndex state_port_;
  drake::systems::InputPortIndex trajectory_port_;
  drake::systems::InputPortIndex radio_port_;

};

}  // namespace dairlib::examples::osc_run
