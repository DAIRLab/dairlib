#include "distance_tracking_data.h"

namespace dairlib::systems::controllers {

using drake::systems::Context;

using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::MatrixXd;

/**** DistanceTrackingData ****/
DistanceTrackingData::DistanceTrackingData(
    const std::string& name, const Eigen::MatrixXd& K_p,
    const Eigen::MatrixXd& K_d, const Eigen::MatrixXd& W,
    const drake::multibody::MultibodyPlant<double>& plant,
    OptionsTrackingData* to_frame_data, OptionsTrackingData* from_frame_data)
    : OptionsTrackingData(name, 1, 1, K_p, K_d, W, plant),
      to_frame_data_(to_frame_data),
      from_frame_data_(from_frame_data) {
  auto states1 = to_frame_data->GetActiveStates();
  auto states2 = from_frame_data->GetActiveStates();
  active_fsm_states_ = states1;
  DRAKE_DEMAND(states1 == states2);
  DRAKE_DEMAND(to_frame_data->GetYDim() == 3);
  DRAKE_DEMAND(from_frame_data->GetYDim() == 3);

  to_state_ = to_frame_data_->AllocateState();
  from_state_ = from_frame_data_->AllocateState();
}

void DistanceTrackingData::Update(
    const VectorXd& x, const Context<double>& context,
    const drake::trajectories::Trajectory<double>& traj, double t,
    double t_gait_cycle, const int fsm_state, const VectorXd& v_proj,
    OscTrackingDataState& td_state) const {

  // Currently there are redundant calculation here. For both to_frame_data_,
  // and from_frame_data_, we don't nee to run UpdateDesired, UpdateYError,
  // UpdateYdotError and UpdateYddotCmd inside Update().
  // TODO: improve this to make it slightly more efficient.
  to_frame_data_->Update(x, context, dummy_traj_, t, t_gait_cycle, fsm_state, v_proj, to_state_);
  from_frame_data_->Update(x, context, dummy_traj_, t, t_gait_cycle, fsm_state, v_proj, from_state_);
  OptionsTrackingData::Update(x, context, traj, t, t_gait_cycle, fsm_state, v_proj, td_state);
}

void DistanceTrackingData::UpdateY(
    const VectorXd& x, const Context<double>& context,
    OscTrackingDataState& td_state) const {
  td_state.y_(0) = (to_state_.y_ - from_state_.y_).norm();
}

void DistanceTrackingData::UpdateYdot(
    const VectorXd& x, const Context<double>& context,
    OscTrackingDataState& td_state) const {
  Vector3d p = to_state_.y_ - from_state_.y_;
  Vector3d v = to_state_.ydot_ - from_state_.ydot_;
  td_state.ydot_(0) = v.dot(p) / p.norm();
}

void DistanceTrackingData::UpdateJ(
    const VectorXd& x, const Context<double>& context,
    OscTrackingDataState& td_state) const {
  Vector3d p = to_state_.y_ - from_state_.y_;
  td_state.J_ = p.transpose() * (to_state_.J_ - from_state_.J_) / p.norm();
}

void DistanceTrackingData::UpdateJdotV(
    const VectorXd& x, const Context<double>& context,
    OscTrackingDataState& td_state) const {
  Vector3d p_rel = to_state_.y_ - from_state_.y_;
  Vector3d v_rel = to_state_.ydot_ - from_state_.ydot_;
  MatrixXd J_rel = to_state_.J_ - from_state_.J_;
  Vector3d bias_rel = to_state_.JdotV_ - from_state_.JdotV_;
  double phi = p_rel.norm();
  double phidot =  v_rel.dot(p_rel) / p_rel.norm();
  td_state.JdotV_ = (
      v_rel.transpose() * v_rel +
      p_rel.transpose() * bias_rel -
      phidot * p_rel.transpose() * v_rel / phi
  ) / phi;
}

void DistanceTrackingData::CheckDerivedOscTrackingData() {
  to_frame_data_->CheckOscTrackingData();
  from_frame_data_->CheckOscTrackingData();
  DRAKE_DEMAND(K_p_.rows() == 1);
  DRAKE_DEMAND(K_p_.cols() == 1);
  DRAKE_DEMAND(K_d_.rows() == 1);
  DRAKE_DEMAND(K_d_.cols() == 1);
}

}