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
                                 const MultibodyPlant<double>& plant_w_spr,
                                 const MultibodyPlant<double>& plant_wo_spr)
    : plant_w_spr_(plant_w_spr),
      plant_wo_spr_(plant_wo_spr),
      world_w_spr_(plant_w_spr_.world_frame()),
      world_wo_spr_(plant_wo_spr_.world_frame()),
      name_(name),
      n_y_(n_y),
      n_ydot_(n_ydot),
      K_p_(K_p),
      K_d_(K_d),
      W_(W) {}

// Update
void OscTrackingData::Update(
    const VectorXd& x_w_spr, const Context<double>& context_w_spr,
    const VectorXd& x_wo_spr, const Context<double>& context_wo_spr,
    const drake::trajectories::Trajectory<double>& traj, double t,
    double t_since_state_switch, const int fsm_state, const VectorXd& v_proj) {
  fsm_state_ = fsm_state;
  // If the set of active states contains -1, the tracking data is always active
  if (active_fsm_states_.count(-1)) {
    fsm_state_ = -1;
  }
  DRAKE_ASSERT(IsActive(fsm_state));

  UpdateActual(x_w_spr, context_w_spr, x_wo_spr, context_wo_spr, t);
  UpdateDesired(traj, t, t_since_state_switch);
  // 3. Update error
  // Careful: must update y and y_des before calling UpdateYError()
  UpdateYError();
  UpdateYdotError(v_proj);
  UpdateYddotCmd(t, t_since_state_switch);
}

void OscTrackingData::UpdateActual(
    const Eigen::VectorXd& x_w_spr,
    const drake::systems::Context<double>& context_w_spr,
    const Eigen::VectorXd& x_wo_spr,
    const drake::systems::Context<double>& context_wo_spr, double t) {
  // 1. Update actual output
  if (use_springs_in_eval_) {
    UpdateY(x_w_spr, context_w_spr);
    UpdateYdot(x_w_spr, context_w_spr);
  } else {
    UpdateY(x_wo_spr, context_wo_spr);
    UpdateYdot(x_wo_spr, context_wo_spr);
  }

  UpdateJ(x_wo_spr, context_wo_spr);
  UpdateJdotV(x_wo_spr, context_wo_spr);
}

void OscTrackingData::UpdateDesired(
    const drake::trajectories::Trajectory<double>& traj, double t,
    double t_since_state_switch) {
  // 2. Update desired output
  if (traj.has_derivative()) {
    if (traj.rows() == 2 * n_ydot_) {
      y_des_ = traj.value(t).topRows(n_y_);
      ydot_des_ = traj.EvalDerivative(t, 1).topRows(n_ydot_);
      yddot_des_ = traj.EvalDerivative(t, 1).bottomRows(n_ydot_);
    } else {
      y_des_ = traj.value(t);
      ydot_des_ = traj.EvalDerivative(t, 1);
      yddot_des_ = traj.EvalDerivative(t, 2);
    }
  }
  // TODO (yangwill): Remove this edge case after EvalDerivative has been
  // implemented for ExponentialPlusPiecewisePolynomial
  else {
    y_des_ = traj.value(t);
    ydot_des_ = traj.MakeDerivative(1)->value(t);
    yddot_des_ = traj.MakeDerivative(2)->value(t);
  }
  UpdateYddotDes(t, t_since_state_switch);
  time_through_trajectory_ = t - traj.start_time();
}

void OscTrackingData::UpdateYddotCmd(double t, double t_since_state_switch) {
  yddot_command_ =
      yddot_des_converted_ + (K_p_ * (error_y_) + K_d_ * (error_ydot_));
}

void OscTrackingData::StoreYddotCommandSol(const VectorXd& dv) {
  yddot_command_sol_ = J_ * dv + JdotV_;
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
