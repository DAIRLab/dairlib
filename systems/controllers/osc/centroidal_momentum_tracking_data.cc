#include "centroidal_momentum_tracking_data.h"
#include "multibody/multibody_utils.h"
#include "drake/math/roll_pitch_yaw.h"

namespace dairlib::systems::controllers {

using multibody::PinocchioPlant;
using multibody::SetPositionsAndVelocitiesIfNew;

using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::VectorXd;
using Eigen::Vector3d;

using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::math::RollPitchYaw;

CentroidalMomentumTrackingData::CentroidalMomentumTrackingData(
    const std::string &name, const MatrixXd &K_p, const MatrixXd &W,
    const drake::multibody::MultibodyPlant<double> &plant_w_spr,
    const drake::multibody::MultibodyPlant<double> &plant_wo_sp,
    const multibody::PinocchioPlant<double>& pin_plant_w_spr,
    const multibody::PinocchioPlant<double>& pin_plant_wo_spr,
    bool angular_only = true) :
    OptionsTrackingData(name,
                        angular_only ? 3 : 6, angular_only ? 3 : 6,
                        K_p,
                        MatrixXd::Zero(angular_only ? 3 : 6, angular_only ? 3 : 6),
                        W, plant_w_spr, plant_wo_sp),
    pinocchio_plant_w_spings_(pin_plant_w_spr),
    pinocchio_plant_wo_springs_(pin_plant_wo_spr) {

  angular_only_ = angular_only;
  pin_context_w_springs_ = pinocchio_plant_w_spings_.CreateDefaultContext();
  pin_context_wo_springs_ = pinocchio_plant_wo_springs_.CreateDefaultContext();
  ydot_ = VectorXd::Zero(n_y_);
  error_ydot_ = VectorXd::Zero(n_ydot_);
  yddot_des_ = VectorXd::Zero(n_ydot_);
  yddot_des_converted_ = VectorXd::Zero(n_ydot_);
}

void CentroidalMomentumTrackingData::UpdateActual(
    const Eigen::VectorXd& x_w_spr,
    const drake::systems::Context<double>& context_w_spr,
    const Eigen::VectorXd& x_wo_spr,
    const drake::systems::Context<double>& context_wo_spr, double t) {
  // 1. Update actual output
  if (use_springs_in_eval_) {
    UpdateY(x_w_spr, context_w_spr);
  } else {
    UpdateY(x_wo_spr, context_wo_spr);
  }
    if (with_view_frame_) {
    view_frame_rot_T_ =
        view_frame_->CalcWorldToFrameRotation(plant_w_spr_, context_wo_spr);
    y_ = view_frame_rot_T_ * y_;
    J_ = view_frame_rot_T_ * J_;
    JdotV_ = view_frame_rot_T_ * JdotV_;
  }
}


void CentroidalMomentumTrackingData::UpdateYddotDes(double, double) {}

void CentroidalMomentumTrackingData::UpdateYddotCmd(
    double t, double t_since_state_switch) {
  yddot_command_ = ydot_des_ + K_p_ * error_y_;
}

void CentroidalMomentumTrackingData::UpdateY(const VectorXd &x_w_spr,
                                             const Context<double> &context_w_spr) {
  const MultibodyPlant<double>* plant =
      use_springs_in_eval_ ? &plant_w_spr_ : &plant_wo_spr_;
  const PinocchioPlant<double>* pin =
      use_springs_in_eval_ ?
      &pinocchio_plant_w_spings_ : &pinocchio_plant_wo_springs_;
  Context<double>* pin_context =
      use_springs_in_eval_ ?
      pin_context_w_springs_.get() : pin_context_wo_springs_.get();

  Eigen::VectorXd qv = plant->GetPositionsAndVelocities(context_w_spr);
  SetPositionsAndVelocitiesIfNew<double>(*pin, qv, pin_context);

  VectorXd h = VectorXd::Zero(6);
  MatrixXd A = MatrixXd::Zero(6, plant->num_velocities());
  MatrixXd Adot = MatrixXd::Zero(A.rows(), A.cols());

  pin->CalcCentroidalMomentumAndDerivatives(
      *pin_context, &h, &A, &Adot);

  if (angular_only_) {
    y_ = h.tail(3);
    J_ = A.block(3, 0, 3, A.cols());
    JdotV_ = Adot.block(3, 0, 3, A.cols()) *
        qv.tail(plant->num_velocities());
  } else {
    y_ = h;
    J_ = A;
    JdotV_ = Adot * qv.tail(plant->num_velocities());
  }
}

void CentroidalMomentumTrackingData::UpdateYError() { error_y_ = y_des_ - y_; }

void CentroidalMomentumTrackingData::UpdateYdot(const Eigen::VectorXd &x_w_spr,
                                                const drake::systems::Context<
                                                    double> &context_w_spr) {}

void CentroidalMomentumTrackingData::UpdateYdotError(
    const Eigen::VectorXd &v_proj) {}

void CentroidalMomentumTrackingData::UpdateJ(const VectorXd &x_wo_spr,
                                             const Context<double> &context_wo_spr) {}

void CentroidalMomentumTrackingData::UpdateJdotV(
    const VectorXd &x_wo_spr, const Context<double> &context_wo_spr) {}

void CentroidalMomentumTrackingData::CheckDerivedOscTrackingData(){}
}

