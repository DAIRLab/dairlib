#include "alip_mpc_interface_system.h"

#include <string>
#include "multibody/multibody_utils.h"

#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/pass_through.h"


namespace dairlib {
namespace systems {
namespace controllers {

AlipMPCInterfaceSystem::AlipMPCInterfaceSystem(
    const drake::multibody::MultibodyPlant<double> &plant,
    drake::systems::Context<double> *context,
    AlipComTrajGeneratorParams com_params,
    SwingFootTrajectoryGeneratorParams swing_params) {
  set_name("alip_mpc_interface_system");
  drake::systems::DiagramBuilder<double> builder;
  auto swing_interface =
      builder.AddSystem<SwingFootTrajectoryGenerator>(plant, context, swing_params);
  auto com_interface =
      builder.AddSystem<AlipComTrajectoryGenerator>(plant, context, com_params);


  // Export ports
  state_port_ = ExportSharedInput(
      builder,
      swing_interface->get_input_port_state(),
      com_interface->get_input_port_state(),
      "x, u, t");

  fsm_port_ = ExportSharedInput(
      builder,
      swing_interface->get_input_port_fsm(),
      com_interface->get_input_port_fsm(),
      "fsm");

  next_touchdown_time_port_ = ExportSharedInput(
      builder,
      swing_interface->get_input_port_next_fsm_switch_time(),
      com_interface->get_input_port_next_fsm_switch_time(),
      "tnext");

  prev_liftoff_time_port_ = ExportSharedInput(
      builder,
      swing_interface->get_input_port_fsm_switch_time(),
      com_interface->get_input_port_fsm_switch_time(),
      "tprev");

  footstep_target_port_ =
      builder.ExportInput(swing_interface->get_input_port_footstep_target());
  slope_parameter_input_port_ =
      builder.ExportInput(com_interface->get_input_port_slope_params());
  com_traj_port_ = builder.ExportOutput(com_interface->get_output_port_com());
  swing_traj_port_ =
      builder.ExportOutput(swing_interface->get_output_port_swing_foot_traj());

  builder.BuildInto(this);

}

drake::systems::InputPortIndex AlipMPCInterfaceSystem::ExportSharedInput(
    drake::systems::DiagramBuilder<double>& builder,
    const drake::systems::InputPort<double> &p1,
    const drake::systems::InputPort<double> &p2, std::string name) {

  const drake::systems::InputPortIndex idx = builder.ExportInput(p1, name);
  builder.ConnectInput(name, p2);
  return idx;
}

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
