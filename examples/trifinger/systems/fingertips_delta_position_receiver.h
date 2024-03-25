#pragma once

#include <drake/common/trajectories/trajectory.h>
#include "dairlib/lcmt_fingertips_delta_position.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::systems {

class FingertipDeltaPositionReceiver
    : public drake::systems::LeafSystem<double> {
 public:
  FingertipDeltaPositionReceiver(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context,
      const Eigen::Vector3d& min_fingertips_delta_position,
      const Eigen::Vector3d& max_fingertips_delta_position,
      const std::string& fingertip_0_name,
      const std::string& fingertip_120_name,
      const std::string& fingertip_240_name);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>&
  get_input_port_fingertips_delta_position() const {
    return this->get_input_port(fingertips_delta_position_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_fingertips_target()
      const {
    return this->get_output_port(fingertips_target_port_);
  }

 private:
  void CopyToOutputTraj(
      const drake::systems::Context<double>& context,
      drake::trajectories::Trajectory<double> *target_traj) const;

  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;

  drake::systems::InputPortIndex state_port_;
  drake::systems::InputPortIndex fingertips_delta_position_port_;
  drake::systems::OutputPortIndex fingertips_target_port_;
  drake::systems::DiscreteStateIndex fingertips_target_idx_;
  drake::systems::DiscreteStateIndex prev_target_timestamp_idx_;

  Eigen::Vector3d min_fingertips_delta_position_;
  Eigen::Vector3d max_fingertips_delta_position_;

  std::string fingertip_0_name_;
  std::string fingertip_120_name_;
  std::string fingertip_240_name_;
};

}  // namespace dairlib::systems
