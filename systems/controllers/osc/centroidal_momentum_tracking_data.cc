//
// Created by brian on 11/17/21.
//

#include "centroidal_momentum_tracking_data.h"

namespace dairlib::systems::controllers {

using multibody::PinocchioPlant;

using Eigen::MatrixXd;
using Eigen::VectorXd;

using drake::multibody::MultibodyPlant;
using drake::systems::Context;

CentroidalMomentumTrackingData::CentroidalMomentumTrackingData(
    const std::string& name, const MatrixXd& K_p,
    const MatrixXd& K_d, const MatrixXd& W,
    const drake::multibody::MultibodyPlant<double>& plant_w_spr,
    const drake::multibody::MultibodyPlant<double>& plant_wo_sp,
    const std::string& urdf_w_spr, const std::string& urdf_wo_spr,
    bool angular_only=true) :
    OptionsTrackingData(name,
                        angular_only ? 3 : 6, angular_only ? 3 : 6,
                        K_p, K_d, W, plant_w_spr, plant_wo_sp) {

  angular_only_ = angular_only;

  pinocchio_plant_w_spings_ = PinocchioPlant<double>(0.0, urdf_w_spr);
  pinocchio_plant_wo_springs_ = PinocchioPlant<double>(0.0, urdf_wo_spr);
  pinocchio_plant_w_spings_.Finalize();
  pinocchio_plant_wo_springs_.Finalize();
  pin_context_w_springs_ = pinocchio_plant_w_spings_.CreateDefaultContext();
  pin_context_wo_springs_ = pinocchio_plant_wo_springs_.CreateDefaultContext();
}

void CentroidalMomentumTrackingData::UpdateY(const int &x_w_spr,
                                             const int &context_w_spr) {
  y_ = VectorXd::Zero()
}
void CentroidalMomentumTrackingData::UpdateDesired(const drake::trajectories::Trajectory<
    double> &traj, double t, double t_since_state_switch) {
  // 2. Update desired output
  y_des_ = traj.value(t);
  if (traj.has_derivative()) {
    ydot_des_ = traj.EvalDerivative(t, 1);
  }
    // TODO (yangwill): Remove this edge case after EvalDerivative has been
    // implemented for ExponentialPlusPiecewisePolynomial
  else {
    ydot_des_ = traj.MakeDerivative(1)->value(t);
  }
}

}