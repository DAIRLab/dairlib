#include "trans_space_tracking_data.h"

#include <iostream>

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::string;
using std::vector;

using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;

namespace dairlib::systems::controllers {

/**** TransTaskSpaceTrackingData ****/
TransTaskSpaceTrackingData::TransTaskSpaceTrackingData(
    const string& name, const MatrixXd& K_p, const MatrixXd& K_d,
    const MatrixXd& W, const MultibodyPlant<double>& plant_w_spr,
    const MultibodyPlant<double>& plant_wo_spr)
    : OptionsTrackingData(name, kSpaceDim, kSpaceDim, K_p, K_d, W, plant_w_spr,
                          plant_wo_spr) {}

void TransTaskSpaceTrackingData::AddPointToTrack(const std::string& body_name,
                                                 const Vector3d& pt_on_body) {
  AddStateAndPointToTrack(-1, body_name, pt_on_body);
}
void TransTaskSpaceTrackingData::AddStateAndPointToTrack(
    int fsm_state, const std::string& body_name, const Vector3d& pt_on_body) {
  AddFiniteStateToTrack(fsm_state);
  DRAKE_DEMAND(plant_w_spr_.HasBodyNamed(body_name));
  DRAKE_DEMAND(plant_wo_spr_.HasBodyNamed(body_name));
  body_frames_w_spr_[fsm_state] =
      &plant_w_spr_.GetBodyByName(body_name).body_frame();
  body_frames_wo_spr_[fsm_state] =
      &plant_wo_spr_.GetBodyByName(body_name).body_frame();
  pts_on_body_[fsm_state] = pt_on_body;
}

void TransTaskSpaceTrackingData::UpdateY(const VectorXd& x_w_spr,
                                         const Context<double>& context_w_spr) {
  y_ = Vector3d::Zero();
  plant_w_spr_.CalcPointsPositions(context_w_spr,
                                   *body_frames_wo_spr_.at(fsm_state_),
                                   pts_on_body_[fsm_state_], world_w_spr_, &y_);
}

void TransTaskSpaceTrackingData::UpdateYdot(
    const VectorXd& x_w_spr, const Context<double>& context_w_spr) {
  MatrixXd J(kSpaceDim, plant_w_spr_.num_velocities());
  plant_w_spr_.CalcJacobianTranslationalVelocity(
      context_w_spr, JacobianWrtVariable::kV,
      *body_frames_w_spr_.at(fsm_state_), pts_on_body_[fsm_state_],
      world_w_spr_, world_w_spr_, &J);
  ydot_ = J * x_w_spr.tail(plant_w_spr_.num_velocities());
}

void TransTaskSpaceTrackingData::UpdateJ(
    const VectorXd& x_wo_spr, const Context<double>& context_wo_spr) {
  J_ = MatrixXd::Zero(kSpaceDim, plant_wo_spr_.num_velocities());
  plant_wo_spr_.CalcJacobianTranslationalVelocity(
      context_wo_spr, JacobianWrtVariable::kV,
      *body_frames_wo_spr_.at(fsm_state_), pts_on_body_[fsm_state_],
      world_wo_spr_, world_wo_spr_, &J_);
}

void TransTaskSpaceTrackingData::UpdateJdotV(
    const VectorXd& x_wo_spr, const Context<double>& context_wo_spr) {
  JdotV_ = plant_wo_spr_.CalcBiasTranslationalAcceleration(
      context_wo_spr, drake::multibody::JacobianWrtVariable::kV,
      *body_frames_wo_spr_.at(fsm_state_), pts_on_body_[fsm_state_],
      world_wo_spr_, world_wo_spr_);
}

void TransTaskSpaceTrackingData::CheckDerivedOscTrackingData() {
  if (!body_frames_w_spr_.empty()) {
    body_frames_w_spr_ = body_frames_wo_spr_;
  }
}
}  // namespace dairlib::systems::controllers
