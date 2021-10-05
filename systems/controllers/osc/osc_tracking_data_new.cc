#include "systems/controllers/osc/osc_tracking_data_new.h"

#include <algorithm>
#include <cmath>

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

using multibody::JwrtqdotToJwrtv;
using multibody::makeNameToPositionsMap;
using multibody::makeNameToVelocitiesMap;
using multibody::WToQuatDotMap;

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
    const drake::trajectories::Trajectory<double>& traj, double t) {
  UpdateActual(x_wo_spr, context_wo_spr);
  UpdateDesired(traj, t);
  // 3. Update error
  // Careful: must update y and y_des before calling UpdateYError()
  UpdateYError();
  UpdateYdotError();

  yddot_command_ =
      yddot_des_converted_ + (K_p_ * (error_y_) + K_d_ * (error_ydot_));
}

void OscTrackingData::UpdateActual(
    const Eigen::VectorXd& x_wo_spr,
    const drake::systems::Context<double>& context_wo_spr) {
  // 1. Update actual output
  UpdateY(x_wo_spr, context_wo_spr);
  UpdateYdot(x_wo_spr, context_wo_spr);
  UpdateJ(x_wo_spr, context_wo_spr);
  UpdateJdotV(x_wo_spr, context_wo_spr);
}

void OscTrackingData::UpdateDesired(
    const drake::trajectories::Trajectory<double>& traj, double t) {
  // 2. Update desired output
  y_des_ = traj.value(t);
  if (traj.has_derivative()) {
    ydot_des_ = traj.EvalDerivative(t, 1);
    yddot_des_ = traj.EvalDerivative(t, 2);
  }
  // TODO (yangwill): Remove this edge case after EvalDerivative has been
  // implemented for ExponentialPlusPiecewisePolynomial
  else {
    ydot_des_ = traj.MakeDerivative(1)->value(t);
    yddot_des_ = traj.MakeDerivative(2)->value(t);
  }
  UpdateYddotDes(t);
}

void OscTrackingData::StoreYddotCommandSol(const VectorXd& dv) {
  yddot_command_sol_ = J_ * dv + JdotV_;
}

void OscTrackingData::AddFiniteStateToTrack(int state) {
  // Avoid repeated states
  for (auto const& element : state_) {
    DRAKE_DEMAND(element != state);
  }
  state_.insert(state);
}

}  // namespace dairlib::systems::controllers
