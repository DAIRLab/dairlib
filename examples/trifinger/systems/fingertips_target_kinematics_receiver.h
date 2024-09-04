#pragma once

#include "dairlib/lcmt_fingertips_position.hpp"
#include "dairlib/lcmt_fingertips_target_kinematics.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::systems {

class FingertipTargetKinematicsReceiver
    : public drake::systems::LeafSystem<double> {
 public:
  FingertipTargetKinematicsReceiver(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context,
      const std::string& fingertip_0_name,
      const std::string& fingertip_120_name,
      const std::string& fingertip_240_name,
      const unsigned int& target_kinematics_update_frequency);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>&
  get_input_port_fingertips_target_kinematics() const {
    return this->get_input_port(fingertips_target_kinematics_port_);
  }
  const drake::systems::OutputPort<double>&
  get_output_port_fingertips_target_traj() const {
    return this->get_output_port(fingertips_target_traj_port_);
  }
  const drake::systems::OutputPort<double>&
  get_output_port_lcm_cur_fingertips_pos() const {
    return this->get_output_port(lcm_cur_fingertips_pos_port_);
  }
  const drake::systems::OutputPort<double>&
  get_output_port_lcm_target_fingertips_pos() const {
    return this->get_output_port(lcm_target_fingertips_pos_port_);
  }

 private:
  void CopyToOutputFingertipsTargetTraj(
      const drake::systems::Context<double>& context,
      drake::trajectories::Trajectory<double>* target_traj) const;

  void CopyToOutputFingertipsTargetKinematics(
      const drake::systems::Context<double>& context,
      TimestampedVector<double>* fingertips_target) const;

  void CopytoLCMCurrentFingertipPositions(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_fingertips_position* lcm_cur_fingertips_pos) const;

  void CopytoLCMTargetFingertipPositions(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_fingertips_position* lcm_target_fingertips_pos) const;

  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;

  drake::systems::InputPortIndex state_port_;
  drake::systems::InputPortIndex fingertips_target_kinematics_port_;
  drake::systems::OutputPortIndex fingertips_target_traj_port_;
  drake::systems::OutputPortIndex fingertips_target_kinematics_out_port_;
  drake::systems::OutputPortIndex lcm_cur_fingertips_pos_port_;
  drake::systems::OutputPortIndex lcm_target_fingertips_pos_port_;
  drake::systems::DiscreteStateIndex fingertips_target_pos_idx_;
  drake::systems::DiscreteStateIndex fingertips_target_vel_idx_;
  drake::systems::DiscreteStateIndex start_fingertips_pos_traj_idx_;
  drake::systems::DiscreteStateIndex start_fingertips_vel_traj_idx_;
  drake::systems::DiscreteStateIndex start_time_traj_idx_;
  drake::systems::DiscreteStateIndex prev_target_timestamp_idx_;
  drake::systems::DiscreteStateIndex cur_fingertips_pos_idx_;
  drake::systems::DiscreteStateIndex cur_fingertips_vel_idx_;

  std::string fingertip_0_name_;
  std::string fingertip_120_name_;
  std::string fingertip_240_name_;

  unsigned int target_kinematics_update_frequency_;
};

}  // namespace dairlib::systems
