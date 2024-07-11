#include "external_force_tracking_data.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using std::string;

using drake::multibody::MultibodyPlant;

namespace dairlib {

using multibody::KinematicEvaluator;
using multibody::WorldPointEvaluator;

namespace systems::controllers {

ExternalForceTrackingData::ExternalForceTrackingData(
    const string& name, const MatrixXd& W, const MultibodyPlant<double>& plant,
    const std::string& body_name, const Vector3d& pt_on_body)
    : name_(name),
      plant_(plant),
      frame_evaluator_(std::make_shared<multibody::WorldPointEvaluator<double>>(
          plant_, pt_on_body, plant_.GetBodyByName(body_name).body_frame())),
      W_(W) {
  lambda_des_ = Vector3d::Zero();
}

void ExternalForceTrackingData::Update(
    const drake::trajectories::Trajectory<double>& traj, double t) {
  DRAKE_DEMAND(traj.rows() == 3);
  lambda_des_ = -traj.value(t);
}

}  // namespace systems::controllers
}  // namespace dairlib