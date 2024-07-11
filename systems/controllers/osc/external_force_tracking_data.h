#pragma once

#include <drake/common/trajectories/trajectory.h>
#include <drake/multibody/plant/multibody_plant.h>

#include "multibody/kinematic/world_point_evaluator.h"

namespace dairlib {
namespace systems {
namespace controllers {

const static int kLambdaDim = 3;

/// ExternalForceTrackingData
/// Force tracking objective. Used to track desired external forces. Requires
/// contact points on the MultibodyPlant where contact forces enter the dynamics
class ExternalForceTrackingData {
 public:
  ExternalForceTrackingData(
      const std::string& name, const Eigen::MatrixXd& W,
      const drake::multibody::MultibodyPlant<double>& plant,
      const std::string& body_name, const Eigen::Vector3d& pt_on_body);

  const Eigen::MatrixXd& GetWeight() const { return W_; }
  const Eigen::VectorXd& GetLambdaDes() const { return lambda_des_; }
  std::shared_ptr<const multibody::WorldPointEvaluator<double>> GetFrameEvaluator() const {
    return frame_evaluator_;
  }
  const std::string& GetName() const { return name_; };
  int GetLambdaDim() const { return kLambdaDim; };

  const drake::multibody::MultibodyPlant<double>& plant() const {
    return plant_;
  };
  void Update(const drake::trajectories::Trajectory<double>& traj, double t);

 private:
  std::string name_;

  const drake::multibody::MultibodyPlant<double>& plant_;
  const drake::multibody::RigidBodyFrame<double>* body_frame_;
  std::shared_ptr<const multibody::WorldPointEvaluator<double>> frame_evaluator_;
  const Eigen::Vector3d pt_on_body_;

  Eigen::VectorXd lambda_des_;
  Eigen::MatrixXd W_;
};

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib