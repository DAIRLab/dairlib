#pragma once

#include "dairlib/lcmt_fingertips_delta_position.hpp"
#include "dairlib/lcmt_fingertips_position.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/trajectory.h"
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
      const std::string& fingertip_240_name,
      const unsigned int& delta_pos_update_frequency);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>&
  get_input_port_fingertips_delta_position() const {
    return this->get_input_port(fingertips_delta_position_port_);
  }
  const drake::systems::OutputPort<double>&
  get_output_port_fingertips_target_traj() const {
    return this->get_output_port(fingertips_target_traj_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_fingertips_target()
      const {
    return this->get_output_port(fingertips_target_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_cur_fingertips_pos()
      const {
    return this->get_output_port(cur_fingertips_pos_port_);
  }
  const drake::systems::OutputPort<double>&
  get_output_port_lcm_cur_fingertips_pos() const {
    return this->get_output_port(lcm_cur_fingertips_pos_port_);
  }

 private:
  void CopyToOutputFingertipsTargetTraj(
      const drake::systems::Context<double>& context,
      drake::trajectories::Trajectory<double>* target_traj) const;

  void CopyToOutputFingertipsTarget(
      const drake::systems::Context<double>& context,
      systems::TimestampedVector<double>* target) const;

  void CopyToOutputCurrentFingertipPositions(
      const drake::systems::Context<double>& context,
      systems::TimestampedVector<double>* cur_fingertips_pos) const;

  void CopytoLCMCurrentFingertipPositions(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_fingertips_position* lcm_cur_fingertips_pos) const;

  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;

  drake::systems::InputPortIndex state_port_;
  drake::systems::InputPortIndex fingertips_delta_position_port_;
  drake::systems::OutputPortIndex fingertips_target_traj_port_;
  drake::systems::OutputPortIndex fingertips_target_port_;
  drake::systems::OutputPortIndex cur_fingertips_pos_port_;
  drake::systems::OutputPortIndex lcm_cur_fingertips_pos_port_;
  drake::systems::DiscreteStateIndex fingertips_target_idx_;
  drake::systems::DiscreteStateIndex start_fingertips_idx_;
  drake::systems::DiscreteStateIndex prev_target_timestamp_idx_;
  drake::systems::DiscreteStateIndex start_time_traj_idx_;
  drake::systems::DiscreteStateIndex cur_fingertips_pos_idx_;

  Eigen::Vector3d min_fingertips_delta_position_;
  Eigen::Vector3d max_fingertips_delta_position_;

  std::string fingertip_0_name_;
  std::string fingertip_120_name_;
  std::string fingertip_240_name_;

  unsigned int delta_pos_update_frequency_;
};

}  // namespace dairlib::systems
