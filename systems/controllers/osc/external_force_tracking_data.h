#pragma once

#include <iostream>
#include <drake/common/trajectories/trajectory.h>
#include <drake/multibody/plant/multibody_plant.h>

namespace dairlib {
namespace systems {
namespace controllers {

/// ExternalForceTrackingData
/// Force tracking objective. Used to track desired external forces. Requires
/// contact points on the MultibodyPlant where contact forces enter the dynamics
class ExternalForceTrackingData {
 public:
  ExternalForceTrackingData(
      const std::string& name, const Eigen::MatrixXd& W,
      const drake::multibody::MultibodyPlant<double>& plant_w_spr,
      const drake::multibody::MultibodyPlant<double>& plant_wo_spr,
      std::string body_name, Eigen::Vector3d pt_on_body);

  ExternalForceTrackingData(
      const std::string& name, const Eigen::MatrixXd& W,
      const drake::multibody::MultibodyPlant<double>& plant_w_spr,
      const drake::multibody::MultibodyPlant<double>& plant_wo_spr,
      std::vector<std::string> body_names, 
      std::vector<Eigen::Vector3d> pts_on_bodies);

  const Eigen::MatrixXd& GetWeight() const { return W_; }

  const Eigen::MatrixXd& GetJ() const { return J_; }
  const Eigen::VectorXd& GetLambdaDes() const { return lambda_des_; }
  const std::string& GetName() const { return name_; };
  int GetLambdaDim() const { return n_lambda_; };

  const drake::multibody::MultibodyPlant<double>& plant_w_spr() const {
    return plant_w_spr_;
  };
  const drake::multibody::MultibodyPlant<double>& plant_wo_spr() const {
    return plant_wo_spr_;
  };
  void Update(const Eigen::VectorXd& x_w_spr,
              const drake::systems::Context<double>& context_w_spr,
              const Eigen::VectorXd& x_wo_spr,
              const drake::systems::Context<double>& context_wo_spr,
              const drake::trajectories::Trajectory<double>& traj,
              double t);

 protected:
 private:
  std::string name_;

  const drake::multibody::MultibodyPlant<double>& plant_w_spr_;
  const drake::multibody::MultibodyPlant<double>& plant_wo_spr_;
  // World frames
  const drake::multibody::RigidBodyFrame<double>& world_w_spr_;
  const drake::multibody::RigidBodyFrame<double>& world_wo_spr_;

  std::vector<const drake::multibody::RigidBodyFrame<double>*> body_frames_w_spr_;
  std::vector<const drake::multibody::RigidBodyFrame<double>*> body_frames_wo_spr_;
  std::vector<Eigen::Vector3d> pts_on_bodies_;
  std::vector<std::string> body_names_;

  int n_lambda_;

  Eigen::VectorXd lambda_des_;
  Eigen::MatrixXd J_;
  Eigen::MatrixXd W_;
};

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib