#include "mpfc_output_from_footstep.h"
#include "systems/framework/output_vector.h"

namespace dairlib::perceptive_locomotion{

MpfcOutputFromFootstep::MpfcOutputFromFootstep(
    double T_ss, double T_ds,
    const drake::multibody::MultibodyPlant<double>& plant) : Tss_(T_ss), Tds_(T_ds) {

  input_port_state_ = DeclareVectorInputPort(
      "x, u, t", systems::OutputVector<double>(plant.num_positions(),
                                               plant.num_velocities(),
                                               plant.num_actuators())
       ).get_index();
  input_port_footstep_ = DeclareVectorInputPort("next_footstep", 3).get_index();

  output_port_mpc_output_ = DeclareAbstractOutputPort(
      "mpc", &MpfcOutputFromFootstep::CalcOutput).get_index();
}

lcmt_fsm_info MpfcOutputFromFootstep::MakeFsmInfo(double t) const {
  lcmt_fsm_info fsm{};
  // TODO (@Brian-Acosta) implement simple time based fsm

  return fsm;
}

void MpfcOutputFromFootstep::CalcOutput(
    const drake::systems::Context<double> &context,
    lcmt_alip_mpc_output *output) const {
  auto robot_output = dynamic_cast<const systems::OutputVector<double>*>(
      EvalVectorInput(context, input_port_state_));
  auto fsm = MakeFsmInfo(robot_output->get_timestamp());

  output->fsm = fsm;

  // TODO (@Brian-Acosta) Fill in footstep and u_traj
}

}