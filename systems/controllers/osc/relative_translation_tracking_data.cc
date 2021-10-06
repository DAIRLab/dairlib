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
    const drake::multibody::MultibodyPlant<double>& plant_w_spr,
    const drake::multibody::MultibodyPlant<double>& plant_wo_spr,
    OptionsTrackingData& to_frame_data, OptionsTrackingData& from_frame_data)
    : OptionsTrackingData(name, kSpaceDim, kSpaceDim, K_p, K_d, W, plant_w_spr,
                          plant_wo_spr),
      to_frame_data_(to_frame_data),
      from_frame_data_(from_frame_data) {
  auto states1 = to_frame_data.GetActiveStates();
  auto states2 = from_frame_data.GetActiveStates();
  DRAKE_DEMAND(states1 == states2);
//  DRAKE_DEMAND(states1.size() == states2.size());
//  for (int i = 0; i < states1.size(); i++) {
//    DRAKE_DEMAND(states1.at(i) == states2.at(i));
//    AddState(states1.at(i));
//  }
}

void RelativeTranslationTrackingData::PreUpdate(
    const VectorXd& x_w_spr, const Context<double>& context_w_spr,
    const VectorXd& x_wo_spr, const Context<double>& context_wo_spr,
    const drake::trajectories::Trajectory<double>& traj, double t,
    double t_since_last_state_switch, int fsm_state,
    const Eigen::VectorXd& v_proj, bool no_desired_traj) {
  to_frame_data_.Update(x_w_spr, context_w_spr, x_wo_spr, context_wo_spr, traj,
                        t, fsm_state, v_proj);
  from_frame_data_.Update(x_w_spr, context_w_spr, x_wo_spr, context_wo_spr,
                          traj, t, fsm_state, v_proj);
}

void RelativeTranslationTrackingData::UpdateY(
    const VectorXd& x_w_spr, const Context<double>& context_w_spr) {
  y_ = to_frame_data_.GetY() - from_frame_data_.GetY();
}

void RelativeTranslationTrackingData::UpdateYdot(
    const VectorXd& x_w_spr, const Context<double>& context_w_spr) {
  ydot_ = to_frame_data_.GetYdot() - from_frame_data_.GetYdot();
}

void RelativeTranslationTrackingData::UpdateJ(
    const VectorXd& x_wo_spr, const Context<double>& context_wo_spr) {
  J_ = to_frame_data_.GetJ() - from_frame_data_.GetJ();
}

void RelativeTranslationTrackingData::UpdateJdotV(
    const VectorXd& x_wo_spr, const Context<double>& context_wo_spr) {
  JdotV_ = to_frame_data_.GetJdotTimesV() - from_frame_data_.GetJdotTimesV();
}

void RelativeTranslationTrackingData::CheckDerivedOscTrackingData() {
  to_frame_data_.CheckOscTrackingData();
  from_frame_data_.CheckOscTrackingData();
}

}  // namespace dairlib::systems::controllers