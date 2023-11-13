#include "external_force_tracking_data.h"

using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::string;
using std::vector;

using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;

namespace dairlib::systems::controllers {

ExternalForceTrackingData::ExternalForceTrackingData(
    const string& name, const MatrixXd& W,
    const MultibodyPlant<double>& plant_w_spr,
    const MultibodyPlant<double>& plant_wo_spr,
    const std::string& body_name, const Vector3d& pt_on_body)
    : name_(name),
      plant_w_spr_(plant_w_spr),
      plant_wo_spr_(plant_wo_spr),
      world_w_spr_(plant_w_spr_.world_frame()),
      world_wo_spr_(plant_wo_spr_.world_frame()),
      body_frame_w_spr_(&plant_w_spr_.GetBodyByName(body_name).body_frame()),
      body_frame_wo_spr_(&plant_wo_spr_.GetBodyByName(body_name).body_frame()),
      pt_on_body_(pt_on_body),
      W_(W) {
  J_ = MatrixXd::Zero(3, plant_wo_spr_.num_velocities());
  lambda_des_ = Vector3d::Zero();
}

void ExternalForceTrackingData::Update(
    const Eigen::VectorXd& x_w_spr,
    const drake::systems::Context<double>& context_w_spr,
    const Eigen::VectorXd& x_wo_spr,
    const drake::systems::Context<double>& context_wo_spr,
    const drake::trajectories::Trajectory<double>& traj,
    double t) {
  DRAKE_DEMAND(traj.rows() == 3);
  lambda_des_ = -traj.value(t);
  J_ = MatrixXd::Zero(3, plant_wo_spr_.num_velocities());
  plant_wo_spr_.CalcJacobianTranslationalVelocity(
      context_wo_spr, JacobianWrtVariable::kV, *body_frame_wo_spr_, pt_on_body_,
      world_wo_spr_, world_wo_spr_, &J_);
}

}  // namespace dairlib::systems::controllers
