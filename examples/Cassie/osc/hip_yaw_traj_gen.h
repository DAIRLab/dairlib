#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

namespace dairlib::cassie {

class HipYawTrajGen : public drake::systems::LeafSystem<double>{
 public:
  HipYawTrajGen(int left_stance_state);

  const drake::systems::InputPort<double>& get_fsm_input_port() {
    return this->get_input_port(fsm_port_);
  }

  const drake::systems::InputPort<double>& get_radio_input_port() {
    return this->get_input_port(radio_port_);
  }

  const drake::systems::OutputPort<double>& get_hip_yaw_output_port() {
    return this->get_output_port(hip_yaw_traj_port_);
  }

 private:
  void CalcYawTraj(const drake::systems::Context<double>& context,
                       drake::trajectories::Trajectory<double>* traj) const;

  drake::systems::InputPortIndex radio_port_;
  drake::systems::InputPortIndex fsm_port_;
  drake::systems::OutputPortIndex hip_yaw_traj_port_;
  const int left_stance_state_;
};
}

