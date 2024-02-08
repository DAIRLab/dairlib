#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "systems/framework/impact_info_vector.h"


namespace dairlib {
namespace systems {
namespace controllers {

/// This class consumes fsm data for a walking gait with double stance periods,
/// and outputs the ImpactInfoVector to be consumed by the OSC for imapct
/// invariant control
class BipedalWalkingImpactInfo : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BipedalWalkingImpactInfo);

  BipedalWalkingImpactInfo(const drake::multibody::MultibodyPlant<double>& plant,
                           std::vector<int> left_right_fsm_states,
                           std::vector<int> post_left_right_fsm_states);

  [[nodiscard]] const drake::systems::InputPort<double>& get_input_port_state()
  const {
    return get_input_port(input_port_state_);
  }
  [[nodiscard]] const drake::systems::InputPort<double>& get_input_port_fsm()
  const {
    return get_input_port(input_port_fsm_);
  }
  [[nodiscard]] const drake::systems::InputPort<double>& get_input_port_next_switch_time()
  const {
    return get_input_port(input_port_next_switch_time_);
  }
  [[nodiscard]] const drake::systems::InputPort<double>& get_input_port_prev_switch_time()
  const {
    return get_input_port(input_port_prev_switch_time_);
  }

  void CalcImpactInfo(const drake::systems::Context<double>& context,
                      systems::ImpactInfoVector<double>* impact_info) const ;

 private:
  drake::systems::InputPortIndex input_port_state_;
  drake::systems::InputPortIndex input_port_fsm_;
  drake::systems::InputPortIndex input_port_next_switch_time_;
  drake::systems::InputPortIndex input_port_prev_switch_time_;

  std::unordered_map<int, bool> state_ends_with_impact_{};
  std::unordered_map<int, bool> state_begins_with_impact_{};
  std::unordered_map<int, int> post_impact_state_{};

  double window_ = 40 * 1e-3; // 50 ms impact window
  double tau_ = 10 * 1e-3; // 5 ms blending time constant

};

}
}
}