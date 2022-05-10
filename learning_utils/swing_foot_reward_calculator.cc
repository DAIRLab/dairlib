#include "swing_foot_reward_calculator.h"
#include "dairlib/lcmt_osc_output.hpp"
#include <algorithm>

using Eigen::Matrix3d;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::RowVector3d;
using Eigen::MatrixXd;

using drake::systems::BasicVector;

namespace dairlib::learning {

SwingFootRewardCalculator::SwingFootRewardCalculator(
    const Matrix3d& W_swing_foot_tracking,
    const Matrix3d& W_swing_foot_smoothness,
    const std::vector<int>& single_support_fsm_states,
    const double single_support_duration,
    const std::string& swing_foot_traj_name) :
    W_swing_foot_tracking_(W_swing_foot_tracking),
    W_swing_foot_smoothness_(W_swing_foot_smoothness),
    single_support_fsm_states_(single_support_fsm_states),
    single_support_duration_(single_support_duration),
    traj_name_(swing_foot_traj_name){

  osc_debug_input_port_ = this->DeclareAbstractInputPort(
      "osc_debug",drake::Value<lcmt_osc_output>()).get_index();
  fsm_input_port_ = this->DeclareVectorInputPort(
      "fsm", BasicVector<double>(1)).get_index();
  switch_time_port_ = this->DeclareVectorInputPort(
      "fsm_switch", BasicVector<double>(1)).get_index();
  reward_output_port_ = this->DeclareVectorOutputPort(
      "reward out",1,
      &SwingFootRewardCalculator::CopyReward).get_index();

  running_reward_idx_ = this->DeclareDiscreteState(BasicVector<double>(VectorXd::Zero(1)));
  prev_fsm_state_idx_ = this->DeclareDiscreteState(BasicVector<double>(1));
  prev_time_idx_ = this->DeclareDiscreteState(BasicVector<double>(-1*VectorXd::Ones(1)));

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
  double switch_time = this->EvalVectorInput(
      context, switch_time_port_)->value()(0);

  int curr_fsm = osc_debug.fsm_state;
  int prev_fsm =
      state->get_discrete_state(prev_fsm_state_idx_).value()(0);

  if (curr_fsm != prev_fsm ) {
    if (std::find(single_support_fsm_states_.begin(),
              single_support_fsm_states_.end(), curr_fsm) !=
        single_support_fsm_states_.end()) {
      state->get_mutable_discrete_state(running_reward_idx_).SetZero();
    } else {
      // Calculate impact related reward terms here

    }
  }
  // return if no new messages or double stance or controller just started
  if (prev_time == new_time || prev_time == -1 ||
      std::find(single_support_fsm_states_.begin(),
                single_support_fsm_states_.end(), curr_fsm) ==
          single_support_fsm_states_.end()){
      state->get_mutable_discrete_state(prev_time_idx_).get_mutable_value()(0) = new_time;
    return drake::systems::EventStatus::Succeeded();
  }

  // calculate reward
  double dt = new_time - prev_time;
  auto it = std::find(
      osc_debug.tracking_data_names.begin(),
      osc_debug.tracking_data_names.end(),traj_name_);

  int idx = it - osc_debug.tracking_data_names.begin();
  auto& swing_ft_data = osc_debug.tracking_data.at(idx);

  Vector3d y_err = Eigen::Map<Vector3d>(swing_ft_data.error_y.data());
  Vector3d y_des = Eigen::Map<Vector3d>(swing_ft_data.y_des.data());
  Vector3d y_dd = Eigen::Map<Vector3d>(swing_ft_data.yddot_command.data());

  // Inner product between swing foot trajectory and desired swing foot traj
  // scaled to be large near end of stance
  VectorXd tracking_reward = - dt * y_err.head(2).transpose() * y_err.head(2);
  tracking_reward *= 1000 * pow((new_time - switch_time) / single_support_duration_, 2);
  // Inner product between desired and realized swing foot acceleration
  VectorXd smooth_reward =  - dt * y_dd.transpose() * y_dd;

  state->get_mutable_discrete_state(running_reward_idx_).get_mutable_value() <<
      state->get_discrete_state(running_reward_idx_).value() + tracking_reward + smooth_reward;

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