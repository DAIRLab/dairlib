#pragma once

#include "multibody/multibody_utils.h"
#include "drake/systems/framework/leaf_system.h"

/// System to translate an incoming lcmt_saved_traj message containing an ankle
/// torque to a vector of desired robot ankle torques
namespace dairlib::perceptive_locomotion {
class CassieAnkleTorqueReceiver : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CassieAnkleTorqueReceiver);
  CassieAnkleTorqueReceiver(const drake::multibody::MultibodyPlant<double>& plant,
                            std::vector<int> left_right_fsm_states,
                            std::vector<std::string> left_right_ankle_motor_names);

  const drake::systems::InputPort<double>& get_input_port_fsm() {
    return this->get_input_port(fsm_input_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_u() {
    return this->get_input_port(input_traj_input_port_);
  }

 private:

  void CopyInput(const drake::systems::Context<double>& context,
                 drake::systems::BasicVector<double>* out) const;

  const int nu_;
  drake::systems::InputPortIndex fsm_input_port_;
  drake::systems::InputPortIndex input_traj_input_port_;
  const std::vector<int> left_right_fsm_states_;
  std::unordered_map<int, int> fsm_to_stance_ankle_map_;
};
}