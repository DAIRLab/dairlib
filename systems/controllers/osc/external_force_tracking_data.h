#pragma once

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
      const drake::multibody::MultibodyPlant<double>& plant_wo_spr);

  const Eigen::MatrixXd& GetWeight() const { return W_; }

  const Eigen::MatrixXd& GetJ() const { return J_; }
  const std::string& GetName() const { return name_; };
  int GetYDim() const { return n_y_; };

  const drake::multibody::MultibodyPlant<double>& plant_w_spr() const {
    return plant_w_spr_;
  };
  const drake::multibody::MultibodyPlant<double>& plant_wo_spr() const {
    return plant_wo_spr_;
  };

 protected:
 private:
  std::string name_;

  // This method is called from the parent class (OscTrackingData) due to C++
  // polymorphism.
  void UpdateActual(const Eigen::VectorXd& x_w_spr,
                    const drake::systems::Context<double>& context_w_spr,
                    const Eigen::VectorXd& x_wo_spr,
                    const drake::systems::Context<double>& context_wo_spr,
                    double t);

  const drake::multibody::MultibodyPlant<double>& plant_w_spr_;
  const drake::multibody::MultibodyPlant<double>& plant_wo_spr_;
  // World frames
  const drake::multibody::BodyFrame<double>& world_w_spr_;
  const drake::multibody::BodyFrame<double>& world_wo_spr_;

  const drake::multibody::BodyFrame<double>* body_frame_w_spr_;
  const drake::multibody::BodyFrame<double>* body_frame_wo_spr_;
  Eigen::Vector3d pt_on_body_;

  int n_y_ = 3;

  Eigen::MatrixXd J_;
  Eigen::MatrixXd W_;
};

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib