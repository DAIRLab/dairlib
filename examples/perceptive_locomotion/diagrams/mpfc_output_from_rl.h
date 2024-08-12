#pragma once

#include "dairlib/lcmt_alip_mpc_output.hpp"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"


namespace dairlib::perceptive_locomotion {

class MpfcOutputFromRL : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MpfcOutputFromRL)

  explicit MpfcOutputFromRL();

  /*!
   * Input port with RL system info:
   * footstep xyz, fsm state, prev_switch_time, next_switch_time
   */
  const drake::systems::InputPort<double>& get_input_port_rl() const {
    return get_input_port(input_port_rl_);
  }

 private:

  void CalcOutput(const drake::systems::Context<double>& context,
                  lcmt_alip_mpc_output* output) const;

  drake::systems::InputPortIndex input_port_rl_;
  drake::systems::OutputPortIndex output_port_mpc_output_;
};

}
