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
                                 const MultibodyPlant<double>& plant_w_spr,
                                 const MultibodyPlant<double>& plant_wo_spr)
    : OptionsTrackingData(name, kSpaceDim, kSpaceDim, K_p, K_d, W, plant_w_spr,
                      plant_wo_spr) {
}

void ComTrackingData::UpdateY(const VectorXd& x_w_spr,
                              const Context<double>& context_w_spr) {
  y_ = plant_w_spr_.CalcCenterOfMassPositionInWorld(context_w_spr);
}

void ComTrackingData::UpdateYdot(const VectorXd& x_w_spr,
                                 const Context<double>& context_w_spr) {
  ydot_ = plant_w_spr_.CalcCenterOfMassTranslationalVelocityInWorld(
      context_w_spr);
}

void ComTrackingData::UpdateJ(const VectorXd& x_wo_spr,
                              const Context<double>& context_wo_spr) {
  J_ = MatrixXd::Zero(kSpaceDim, plant_wo_spr_.num_velocities());
  plant_wo_spr_.CalcJacobianCenterOfMassTranslationalVelocity(
      context_wo_spr, JacobianWrtVariable::kV, world_w_spr_, world_w_spr_, &J_);
}

void ComTrackingData::UpdateJdotV(const VectorXd& x_wo_spr,
                                  const Context<double>& context_wo_spr) {
  JdotV_ = plant_wo_spr_.CalcBiasCenterOfMassTranslationalAcceleration(
      context_wo_spr, JacobianWrtVariable::kV, world_wo_spr_, world_wo_spr_);
}

}