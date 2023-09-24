#include "mpfc_output_from_footstep.h"
#include "systems/framework/output_vector.h"
#include "lcm/lcm_trajectory.h"

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
  const double two_stride_period = 2 * (Tds_ + Tss_);
  const double one_stride_period = Tds_ + Tss_;

  double periods = t / two_stride_period;
  double r = periods - static_cast<int>(periods);
  double phase = r * two_stride_period;
  double period_start = (periods - r) * two_stride_period;

  if (phase < Tds_) { // Post right double stance
    fsm.fsm_state = post_right_double_support_state_;
    fsm.prev_switch_time_us = 1e6 * period_start;
    fsm.next_switch_time_us = 1e6 * (period_start + Tds_);
  } else if (phase < Tss_ + Tds_) { // left stance
    fsm.fsm_state = left_stance_state_;
    fsm.prev_switch_time_us = 1e6 * (period_start + Tds_);
    fsm.next_switch_time_us = 1e6 * (period_start + one_stride_period);
  } else if (phase < 2 * Tds_ + Tss_) { // post left double stance
    fsm.fsm_state = post_left_double_support_state_;
    fsm.prev_switch_time_us = 1e6 * (period_start + one_stride_period);
    fsm.next_switch_time_us = 1e6 * (period_start + one_stride_period + Tds_);
  } else { // right stance
    fsm.fsm_state = right_stance_state_;
    fsm.prev_switch_time_us = 1e6 * (period_start + one_stride_period + Tds_);
    fsm.next_switch_time_us = 1e6 * (period_start + two_stride_period);
  }
  fsm.timestamp_us = 1e6 * t;
  return fsm;
}

void MpfcOutputFromFootstep::CalcOutput(
    const drake::systems::Context<double> &context,
    lcmt_alip_mpc_output *output) const {

  // Get Timestamp
  auto robot_output = dynamic_cast<const systems::OutputVector<double>*>(
      EvalVectorInput(context, input_port_state_));

  // set fsm
  auto fsm = MakeFsmInfo(robot_output->get_timestamp());
  output->fsm = fsm;

  // set input to zero for all time
  LcmTrajectory::Trajectory input_traj;
  input_traj.traj_name = "input_traj";
  input_traj.datatypes = std::vector<std::string>(1, "double");
  input_traj.datapoints = Eigen::Matrix<double, 1, 2>::Zero();
  input_traj.time_vector = Eigen::Vector2d(0, std::numeric_limits<double>::infinity());
  LcmTrajectory lcm_traj(
      {input_traj}, {"input_traj"}, "input_traj", "input_traj", false);
  output->u_traj = lcm_traj.GenerateLcmObject();

  // set footstep target from input port
  Eigen::Vector3d footstep_cmd = EvalVectorInput(context, input_port_footstep_)->value();
  std::copy(footstep_cmd.data(),
            footstep_cmd.data() + footstep_cmd.size(),
            output->next_footstep_in_stance_frame);
}

}