#include "options_tracking_data.h"

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::string;
using std::vector;

using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;

namespace dairlib::systems::controllers {

OptionsTrackingData::OptionsTrackingData(
    const string& name, int n_y, int n_ydot, const MatrixXd& K_p,
    const MatrixXd& K_d, const MatrixXd& W,
    const MultibodyPlant<double>& plant_w_spr,
    const MultibodyPlant<double>& plant_wo_spr)
    : ImpactInvariantTrackingData(name, n_y, n_ydot_, K_p, K_d, W, plant_w_spr,
                      plant_wo_spr) {}

void OptionsTrackingData::UpdateActual(
    const Eigen::VectorXd& x_wo_spr,
    const drake::systems::Context<double>& context_wo_spr) {
  ImpactInvariantTrackingData::UpdateActual(x_wo_spr, context_wo_spr);

  view_frame_rot_T_ =
      view_frame_->CalcRotationalMatrix(plant_w_spr_, context_wo_spr)
          .transpose();
}

void OptionsTrackingData::UpdateYError() {
  error_y_ = view_frame_rot_T_ * error_y_;
}

void OptionsTrackingData::UpdateYdotError() {
  ImpactInvariantTrackingData::UpdateYdotError();
  error_ydot_ = view_frame_rot_T_ * error_ydot_;
}

void OptionsTrackingData::UpdateYddotDes(double t) {
  for (auto idx : idx_zero_feedforward_accel_) {
    yddot_des_converted_(idx) = 0;
  }
  if (ff_accel_multiplier_ != nullptr) {
    yddot_des_converted_ =
        ff_accel_multiplier_->value(t) *
            yddot_des_converted_;
  }
}


}  // namespace dairlib::systems::controllers