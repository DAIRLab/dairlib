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
    const MatrixXd& W, const MultibodyPlant<double>& plant)
    : OptionsTrackingData(name, kSpaceDim, kSpaceDim, K_p, K_d, W, plant) {}

void TransTaskSpaceTrackingData::AddPointToTrack(const std::string& body_name,
                                                 const Vector3d& pt_on_body) {
  AddStateAndPointToTrack(-1, body_name, pt_on_body);
}
void TransTaskSpaceTrackingData::AddStateAndPointToTrack(
    int fsm_state, const std::string& body_name, const Vector3d& pt_on_body) {
  AddFiniteStateToTrack(fsm_state);
  DRAKE_DEMAND(plant_.HasBodyNamed(body_name));
  DRAKE_DEMAND(plant_.HasBodyNamed(body_name));
  body_frames_[fsm_state] =
      &plant_.GetBodyByName(body_name).body_frame();
  pts_on_body_[fsm_state] = pt_on_body;
}

void TransTaskSpaceTrackingData::UpdateY(
    const VectorXd& x, const Context<double>& context,
    OscTrackingDataState& td_state) const {
  td_state.y_ = Vector3d::Zero();
  plant_.CalcPointsPositions(
      context, *body_frames_.at(td_state.fsm_state_),
      pts_on_body_.at(td_state.fsm_state_), world_, &td_state.y_);
}

void TransTaskSpaceTrackingData::UpdateYdot(
    const VectorXd& x, const Context<double>& context,
    OscTrackingDataState& td_state) const {
  MatrixXd J(kSpaceDim, plant_.num_velocities());
  plant_.CalcJacobianTranslationalVelocity(
      context, JacobianWrtVariable::kV,
      *body_frames_.at(td_state.fsm_state_), pts_on_body_.at(td_state.fsm_state_),
      world_, world_, &J);
  td_state.ydot_ = J * x.tail(plant_.num_velocities());
}

void TransTaskSpaceTrackingData::UpdateJ(
    const VectorXd& x, const Context<double>& context,
    OscTrackingDataState& td_state) const {
  td_state.J_ = MatrixXd::Zero(kSpaceDim, plant_.num_velocities());
  plant_.CalcJacobianTranslationalVelocity(
      context, JacobianWrtVariable::kV,
      *body_frames_.at(td_state.fsm_state_), pts_on_body_.at(td_state.fsm_state_),
      world_, world_, &td_state.J_);
}

void TransTaskSpaceTrackingData::UpdateJdotV(
    const VectorXd& x, const Context<double>& context,
    OscTrackingDataState& td_state) const {
  td_state.JdotV_ = plant_.CalcBiasTranslationalAcceleration(
      context, drake::multibody::JacobianWrtVariable::kV,
      *body_frames_.at(td_state.fsm_state_), pts_on_body_.at(td_state.fsm_state_),
      world_, world_);
}

void TransTaskSpaceTrackingData::CheckDerivedOscTrackingData() {}
}  // namespace dairlib::systems::controllers
