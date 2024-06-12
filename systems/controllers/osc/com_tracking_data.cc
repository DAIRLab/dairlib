#include "com_tracking_data.h"

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::string;
using std::vector;

using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;

namespace dairlib::systems::controllers {

/**** ComTrackingData ****/
ComTrackingData::ComTrackingData(const string& name, const MatrixXd& K_p,
                                 const MatrixXd& K_d, const MatrixXd& W,
                                 const MultibodyPlant<double>& plant)
    : OptionsTrackingData(name, kSpaceDim, kSpaceDim, K_p, K_d, W, plant) {
}

void ComTrackingData::UpdateY(const VectorXd& x_w_spr,
                              const Context<double>& context_w_spr) {
  y_ = plant_.CalcCenterOfMassPositionInWorld(context_w_spr);
}

void ComTrackingData::UpdateYdot(const VectorXd& x_w_spr,
                                 const Context<double>& context_w_spr) {
  ydot_ = plant_.CalcCenterOfMassTranslationalVelocityInWorld(
      context_w_spr);
}

void ComTrackingData::UpdateJ(const VectorXd& x_wo_spr,
                              const Context<double>& context_wo_spr) {
  J_ = MatrixXd::Zero(kSpaceDim, plant_.num_velocities());
  plant_.CalcJacobianCenterOfMassTranslationalVelocity(
      context_wo_spr, JacobianWrtVariable::kV, world_, world_, &J_);
}

void ComTrackingData::UpdateJdotV(const VectorXd& x_wo_spr,
                                  const Context<double>& context_wo_spr) {
  JdotV_ = plant_.CalcBiasCenterOfMassTranslationalAcceleration(
      context_wo_spr, JacobianWrtVariable::kV, world_, world_);
}

}