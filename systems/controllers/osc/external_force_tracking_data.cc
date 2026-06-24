#include "external_force_tracking_data.h"

#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/common/trajectories/piecewise_quaternion.h>

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
namespace {
bool IsEmptyTrajectory(const drake::trajectories::Trajectory<double>& traj) {
  if (const auto* pp =
          dynamic_cast<const drake::trajectories::PiecewisePolynomial<double>*>(
              &traj)) {
    return pp->get_number_of_segments() == 0;
  }
  if (const auto* slerp = dynamic_cast<
          const drake::trajectories::PiecewiseQuaternionSlerp<double>*>(&traj)) {
    return slerp->get_number_of_segments() == 0;
  }
  return false;
}
}  // namespace

ExternalForceTrackingData::ExternalForceTrackingData(
    const string& name, const MatrixXd& W,
    const MultibodyPlant<double>& plant_w_spr,
    const MultibodyPlant<double>& plant_wo_spr, const std::string& body_name,
    const Vector3d& pt_on_body)
    : name_(name),
      plant_w_spr_(plant_w_spr),
      plant_wo_spr_(plant_wo_spr),
      world_w_spr_(plant_w_spr_.world_frame()),
      world_wo_spr_(plant_wo_spr_.world_frame()),
      body_frame_w_spr_(&plant_w_spr_.GetBodyByName(body_name).body_frame()),
      body_frame_wo_spr_(&plant_wo_spr_.GetBodyByName(body_name).body_frame()),
      pt_on_body_(pt_on_body),
      W_(W) {
  lambda_des_ = Vector3d::Zero();
}

void ExternalForceTrackingData::Update(
    const Eigen::VectorXd& x_w_spr,
    const drake::systems::Context<double>& context_w_spr,
    const Eigen::VectorXd& x_wo_spr,
    const drake::systems::Context<double>& context_wo_spr,
    const drake::trajectories::Trajectory<double>& traj, double t) {
  if (IsEmptyTrajectory(traj)) {
    lambda_des_ = Vector3d::Zero();
  } else {
    DRAKE_DEMAND(traj.rows() == 3);
    lambda_des_ = traj.value(t);
  }
}

}  // namespace dairlib::systems::controllers