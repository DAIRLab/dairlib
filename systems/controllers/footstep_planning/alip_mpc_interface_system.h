#pragma once

#include "alip_utils.h"
#include "swing_foot_trajectory_generator.h"
#include "systems/controllers/alip_com_trajectory_generator.h"


namespace dairlib {
namespace systems {
namespace controllers {

class AlipMPCInterfaceSystem : public drake::systems::Diagram<double> {
 public:
  AlipMPCInterfaceSystem(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context,
      AlipComTrajGeneratorParams com_params,
      SwingFootTrajectoryGeneratorParams swing_params);

  const drake::systems::InputPort<double>& get_input_port_slope_parameters()
  const {
    return this->get_input_port(slope_parameter_input_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm() const {
    return this->get_input_port(fsm_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm_switch_time()
  const {
    return this->get_input_port(prev_liftoff_time_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_next_fsm_switch_time()
  const {
    return this->get_input_port(next_touchdown_time_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_footstep_target()
  const {
    return this->get_input_port(footstep_target_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_com_traj() const {
    return this->get_output_port(com_traj_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_swing_foot_traj()
  const {
    return this->get_output_port(swing_traj_port_);
  }
 private:

  drake::systems::InputPortIndex ExportSharedInput(
      drake::systems::DiagramBuilder<double>& builder,
      const drake::systems::InputPort<double>& p1,
      const drake::systems::InputPort<double>& p2, std::string name);

  drake::systems::InputPortIndex slope_parameter_input_port_;
  drake::systems::InputPortIndex state_port_;
  drake::systems::InputPortIndex fsm_port_;
  drake::systems::InputPortIndex next_touchdown_time_port_;
  drake::systems::InputPortIndex prev_liftoff_time_port_;
  drake::systems::InputPortIndex footstep_target_port_;
  drake::systems::OutputPortIndex com_traj_port_;
  drake::systems::OutputPortIndex swing_traj_port_;

};


}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
