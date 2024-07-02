#include "relative_translation_tracking_data.h"

using Eigen::Isometry3d;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;

using std::string;
using std::vector;

using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;

namespace dairlib::systems::controllers {

/**** RelativeTranslationTrackingData ****/
RelativeTranslationTrackingData::RelativeTranslationTrackingData(
    const std::string& name, const Eigen::MatrixXd& K_p,
    const Eigen::MatrixXd& K_d, const Eigen::MatrixXd& W,
    const drake::multibody::MultibodyPlant<double>& plant,
    OptionsTrackingData* to_frame_data, OptionsTrackingData* from_frame_data)
    : OptionsTrackingData(name, kSpaceDim, kSpaceDim, K_p, K_d, W, plant),
      to_frame_data_(to_frame_data),
      from_frame_data_(from_frame_data) {
  auto states1 = to_frame_data->GetActiveStates();
  auto states2 = from_frame_data->GetActiveStates();
  active_fsm_states_ = states1;
  DRAKE_DEMAND(states1 == states2);

  to_state_ = to_frame_data_->AllocateState();
  from_state_ = from_frame_data_->AllocateState();
}

void RelativeTranslationTrackingData::Update(
    const VectorXd& x, const Context<double>& context,
    const drake::trajectories::Trajectory<double>& traj, double t,
    double t_gait_cycle, const int fsm_state, const VectorXd& v_proj,
    OscTrackingDataState& td_state) const {

  // Currently there are redundant calculation here. For both to_frame_data_,
  // and from_frame_data_, we don't nee to run UpdateDesired, UpdateYError,
  // UpdateYdotError and UpdateYddotCmd inside Update().
  // TODO: improve this to make it slightly more efficient.
  to_frame_data_->Update(x, context, traj,t, t_gait_cycle, fsm_state, v_proj, to_state_);
  from_frame_data_->Update(x, context, traj, t, t_gait_cycle, fsm_state, v_proj, from_state_);
  OptionsTrackingData::Update(x, context, traj, t, t_gait_cycle, fsm_state, v_proj, td_state);
}

void RelativeTranslationTrackingData::UpdateY(
    const VectorXd& x, const Context<double>& context,
    OscTrackingDataState& td_state) const {
  td_state.y_ = to_state_.y_ - from_state_.y_;
}

void RelativeTranslationTrackingData::UpdateYdot(
    const VectorXd& x, const Context<double>& context,
    OscTrackingDataState& td_state) const {
  td_state.ydot_ = to_state_.ydot_ - from_state_.ydot_;
}

void RelativeTranslationTrackingData::UpdateJ(
    const VectorXd& x, const Context<double>& context,
    OscTrackingDataState& td_state) const {
  td_state.J_ = to_state_.J_ - from_state_.J_;
}

void RelativeTranslationTrackingData::UpdateJdotV(
    const VectorXd& x, const Context<double>& context,
    OscTrackingDataState& td_state) const {
  td_state.JdotV_ = to_state_.JdotV_ - from_state_.JdotV_;
}

void RelativeTranslationTrackingData::CheckDerivedOscTrackingData() {
  to_frame_data_->CheckOscTrackingData();
  from_frame_data_->CheckOscTrackingData();
}

}  // namespace dairlib::systems::controllers
