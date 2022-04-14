#include "swing_foot_reward_calculator.h"
#include "dairlib/lcmt_osc_output.hpp"
#include <algorithm>

using Eigen::Matrix3d;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::RowVector3d;
using Eigen::MatrixXd;

using drake::systems::BasicVector;

namespace dairlib::learning {

SwingFootRewardCalculator::SwingFootRewardCalculator(
    const Matrix3d& W_swing_foot_tracking,
    const Matrix3d& W_swing_foot_smoothness,
    const std::vector<int>& single_support_fsm_states,
    const std::string& swing_foot_traj_name) :
    W_swing_foot_tracking_(W_swing_foot_tracking),
    W_swing_foot_smoothness_(W_swing_foot_smoothness),
    single_support_fsm_states_(single_support_fsm_states),
    traj_name_(swing_foot_traj_name){

  osc_debug_input_port_ = this->DeclareAbstractInputPort(
      "osc_debug",drake::Value<lcmt_osc_output>()).get_index();
  fsm_input_port_ = this->DeclareVectorInputPort(
      "fsm", BasicVector<double>(1)).get_index();
  reward_output_port_ = this->DeclareVectorOutputPort(
      "reward out",1,
      &SwingFootRewardCalculator::CopyReward).get_index();

  running_reward_idx_ = this->DeclareDiscreteState(BasicVector<double>(1));
  prev_fsm_state_idx_ = this->DeclareDiscreteState(BasicVector<double>(1));
  prev_time_idx_ = this->DeclareDiscreteState(BasicVector<double>(1));

  this->DeclarePerStepUnrestrictedUpdateEvent(
      &SwingFootRewardCalculator::StateUpdate);
}


drake::systems::EventStatus SwingFootRewardCalculator::StateUpdate(
    const drake::systems::Context<double> &context,
    drake::systems::State<double> *state) const {

  double prev_time = state->get_discrete_state(prev_time_idx_).value()(0);
  lcmt_osc_output osc_debug = this->EvalAbstractInput(
      context, osc_debug_input_port_)->get_value<lcmt_osc_output>();
  double new_time = osc_debug.utime * 1e-6;

  int prev_fsm = state->get_discrete_state(prev_fsm_state_idx_).value()(0);
  int curr_fsm = this->EvalVectorInput(context, fsm_input_port_)->value()(0);

  // return if no new messages or double stance
  if (prev_time == new_time ||
      std::find(single_support_fsm_states_.begin(),
                single_support_fsm_states_.end(), curr_fsm) ==
                single_support_fsm_states_.end()) {
    return drake::systems::EventStatus::Succeeded();}

  // calculate reward
  double dt = new_time - prev_time;
  int idx = std::find(
      osc_debug.tracking_data_names.begin(),
      osc_debug.tracking_data_names.end(),traj_name_) -
          osc_debug.tracking_data_names.begin();

  auto& swing_ft_data = osc_debug.tracking_data.at(idx);
  Vector3d y = Eigen::Map<Vector3d>(swing_ft_data.y.data());
  Vector3d y_des = Eigen::Map<Vector3d>(swing_ft_data.y_des.data());
  Vector3d y_dd = Eigen::Map<Vector3d>(swing_ft_data.yddot_command.data());
  Vector3d y_dd_cmd  = Eigen::Map<Vector3d>(swing_ft_data.yddot_command_sol.data());

  state->get_mutable_discrete_state(running_reward_idx_).get_mutable_value() +=
      dt * (y.transpose() * W_swing_foot_tracking_ * y_des / (y.norm() * y_des.norm())
      + y_dd.transpose() * W_swing_foot_smoothness_ * y_dd_cmd / (y_dd.norm() * y_dd_cmd.norm()));

  state->get_mutable_discrete_state(prev_fsm_state_idx_).get_mutable_value()(0) = curr_fsm;
  state->get_mutable_discrete_state(prev_time_idx_).get_mutable_value()(0) = new_time;
  return drake::systems::EventStatus::Succeeded();
}

void SwingFootRewardCalculator::CopyReward(
    const drake::systems::Context<double> &context,
    drake::systems::BasicVector<double> *reward) const {
  reward->get_mutable_value() =
      context.get_discrete_state(running_reward_idx_).value();
}

}