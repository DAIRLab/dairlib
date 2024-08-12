#include "mpfc_output_from_rl.h"
#include "systems/framework/output_vector.h"
#include "lcm/lcm_trajectory.h"
#include <iostream>

namespace dairlib::perceptive_locomotion{

MpfcOutputFromRL::MpfcOutputFromRL() {

  input_port_rl_ = DeclareVectorInputPort("rl_info", 6).get_index();

  output_port_mpc_output_ = DeclareAbstractOutputPort(
      "mpc", &MpfcOutputFromRL::CalcOutput).get_index();
}

void MpfcOutputFromRL::CalcOutput(
    const drake::systems::Context<double> &context,
    lcmt_alip_mpc_output *output) const {

  Eigen::VectorXd rl_cmd = EvalVectorInput(
      context, input_port_rl_
  )->value();

  // hack for simulation
  if (context.get_time() - rl_cmd(5) > 0.4) {
      rl_cmd(3) = 1;
      rl_cmd(4) = 0.5;
      rl_cmd(5) = 0.8;
  }

  // set fsm output
  output->fsm.timestamp_us = 1e6 * context.get_time();
  output->fsm.fsm_state = rl_cmd(3);
  output->fsm.prev_switch_time_us = 1e6 * rl_cmd(4);
  output->fsm.next_switch_time_us = 1e6 * std::max(rl_cmd(5), context.get_time() + 0.0005);



  // set footstep target from input port
  std::copy(
      rl_cmd.data(),
      rl_cmd.data() + 3,
      output->next_footstep_in_stance_frame
  );

  // set input to zero for all time
  LcmTrajectory::Trajectory input_traj;
  input_traj.traj_name = "input_traj";
  input_traj.datatypes = std::vector<std::string>(1, "double");
  input_traj.datapoints = Eigen::Matrix<double, 1, 2>::Zero();
  input_traj.time_vector = Eigen::Vector2d(0, std::numeric_limits<double>::infinity());
  LcmTrajectory lcm_traj(
      {input_traj}, {"input_traj"}, "input_traj", "input_traj", false);
  output->u_traj = lcm_traj.GenerateLcmObject();


}

}