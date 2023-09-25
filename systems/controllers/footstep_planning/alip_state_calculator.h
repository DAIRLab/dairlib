#pragma once

#include "alip_utils.h"
#include "systems/framework/output_vector.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::systems::controllers::alip_utils{
class AlipStateCalculator : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AlipStateCalculator)
  AlipStateCalculator(const drake::multibody::MultibodyPlant<double> &plant,
                      drake::systems::Context<double> *context,
                      std::vector<int> left_right_support_fsm_states,
                      std::vector<int> post_left_post_right_fsm_states,
                      std::vector<PointOnFramed> left_right_foot,
                      const std::string& expressed_in_frame="");

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return get_input_port(input_port_state_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm() const {
    return get_input_port(input_port_fsm_);
  }

 private:

  void CalcOutput(const drake::systems::Context<double>& context,
                  drake::systems::BasicVector<double>* alip_state) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  const std::string expressed_in_frame_;
  std::map<int, PointOnFramed> stance_foot_map_;

  drake::systems::InputPortIndex input_port_state_;
  drake::systems::InputPortIndex input_port_fsm_;
};
}