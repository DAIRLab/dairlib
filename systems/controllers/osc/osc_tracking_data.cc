#include "systems/controllers/osc/osc_tracking_data.h"

#include <algorithm>
#include <cmath>
#include <iostream>

#include <drake/multibody/plant/multibody_plant.h>

#include "multibody/multibody_utils.h"

using std::cout;
using std::endl;

using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using Eigen::Isometry3d;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::string;
using std::vector;

namespace dairlib::systems::controllers {

using multibody::MakeNameToPositionsMap;
using multibody::MakeNameToVelocitiesMap;

/**** OscTrackingData ****/
OscTrackingData::OscTrackingData(const string& name, int n_y, int n_ydot,
                                 const MatrixXd& K_p, const MatrixXd& K_d,
                                 const MatrixXd& W,
                                 const MultibodyPlant<double>& plant)
    : plant_(plant),
      world_(plant_.world_frame()),
      name_(name),
      n_y_(n_y),
      n_ydot_(n_ydot),
      K_p_(K_p),
      K_d_(K_d),
      W_(W) {}

OscTrackingDataState OscTrackingData::AllocateState() const {
  OscTrackingDataState state;
  state.y_ = VectorXd::Zero(n_y_);
  state.y_des_ = VectorXd::Zero(n_y_);
  state.error_y_ = VectorXd::Zero(n_ydot_);
  state.ydot_ = VectorXd::Zero(n_ydot_);
  state.ydot_des_ = VectorXd::Zero(n_ydot_);
  state.error_ydot_ = VectorXd::Zero(n_ydot_);
  state.yddot_des_ = VectorXd::Zero(n_ydot_);
  state.yddot_des_converted_ = VectorXd::Zero(n_ydot_);
  state.yddot_command_ = VectorXd::Zero(n_ydot_);

  state.J_ = MatrixXd::Zero(n_ydot_, plant_.num_velocities());
  state.JdotV_ = VectorXd::Zero(n_ydot_);
  state.view_frame_rot_T_ = Eigen::Matrix3d::Identity();
  state.name_ = name_;
  state.fsm_state_ = -1;

  // Members of low-pass filter
  state.filtered_y_ = VectorXd::Zero(n_y_);
  state.filtered_ydot_ = VectorXd::Zero(n_ydot_);
  state.time_varying_weight_ = W_;

  return state;
}

// Update
void OscTrackingData::Update(
    const VectorXd& x, const Context<double>& context,
    const drake::trajectories::Trajectory<double>& traj, double t,
    double t_since_state_switch, const int fsm_state, const VectorXd& v_proj,
    OscTrackingDataState& tracking_data_state) const {

  tracking_data_state.fsm_state_ = fsm_state;
  // If the set of active states contains -1, the tracking data is always active
  if (active_fsm_states_.count(-1)) {
    tracking_data_state.fsm_state_ = -1;
  }
  DRAKE_ASSERT(IsActive(fsm_state));

  UpdateActual(x, context, t, tracking_data_state);
  UpdateDesired(traj, t, t_since_state_switch, tracking_data_state);
  // 3. Update error
  // Careful: must update y and y_des before calling UpdateYError()
  UpdateYError(tracking_data_state);
  UpdateYdotError(v_proj, tracking_data_state);
  UpdateYddotCmd(t, t_since_state_switch, tracking_data_state);
}

void OscTrackingData::UpdateActual(
    const Eigen::VectorXd& x,
    const drake::systems::Context<double>& context,
    double t, OscTrackingDataState& tracking_data_state) const {
  // 1. Update actual output
  UpdateY(x, context, tracking_data_state);
  UpdateYdot(x, context, tracking_data_state);
  UpdateJ(x, context, tracking_data_state);
  UpdateJdotV(x, context, tracking_data_state);
}

void OscTrackingData::UpdateDesired(
    const drake::trajectories::Trajectory<double>& traj, double t,
    double t_since_state_switch, OscTrackingDataState& tracking_data_state) const {
  // 2. Update desired output
  if (traj.has_derivative()) {
    if (traj.rows() == 2 * n_ydot_) {
      tracking_data_state.y_des_ = traj.value(t).topRows(n_y_);
      tracking_data_state.ydot_des_ = traj.EvalDerivative(t, 1).topRows(n_ydot_);
      tracking_data_state.yddot_des_ = traj.EvalDerivative(t, 1).bottomRows(n_ydot_);
    } else {
      tracking_data_state.y_des_ = traj.value(t);
      tracking_data_state.ydot_des_ = traj.EvalDerivative(t, 1);
      tracking_data_state.yddot_des_ = traj.EvalDerivative(t, 2);
    }
  }
  // TODO (yangwill): Remove this edge case after EvalDerivative has been
  // implemented for ExponentialPlusPiecewisePolynomial
  else {
    tracking_data_state.y_des_ = traj.value(t);
    tracking_data_state.ydot_des_ = traj.MakeDerivative(1)->value(t);
    tracking_data_state.yddot_des_ = traj.MakeDerivative(2)->value(t);
  }
  UpdateYddotDes(t, t_since_state_switch, tracking_data_state);
  tracking_data_state.time_through_trajectory_ = t - traj.start_time();
}

void OscTrackingData::UpdateYddotCmd(
    double t, double t_since_state_switch,
    OscTrackingDataState& tracking_data_state) const {
  tracking_data_state.yddot_command_ =
      tracking_data_state.yddot_des_converted_ +
      (K_p_ * (tracking_data_state.error_y_) + K_d_ * (tracking_data_state.error_ydot_));
}


void OscTrackingData::AddFiniteStateToTrack(int state) {
  DRAKE_DEMAND(!active_fsm_states_.count(state));
  active_fsm_states_.insert(state);
}

// Run this function in OSC constructor to make sure that users constructed
// OscTrackingData correctly.
void OscTrackingData::CheckOscTrackingData() {
  //  cout << "Checking " << name_ << endl;
  CheckDerivedOscTrackingData();

  DRAKE_DEMAND((K_p_.rows() == n_ydot_) && (K_p_.cols() == n_ydot_));
  DRAKE_DEMAND((K_d_.rows() == n_ydot_) && (K_d_.cols() == n_ydot_));
  DRAKE_DEMAND((W_.rows() == n_ydot_) && (W_.cols() == n_ydot_));
}

}  // namespace dairlib::systems::controllers
