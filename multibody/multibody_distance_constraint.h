#include <string>
#include <drake/multibody/plant/multibody_plant.h>

namespace dairlib {

namespace multibody {

class MultibodyDistanceConstraint {

 public:
  MultibodyDistanceConstraint(
      const drake::multibody::MultibodyPlant<double>& plant,
      const drake::multibody::Body<double>& body1,
      const Eigen::Vector3d& pt_1,
      const drake::multibody::Body<double>& body2,
      const Eigen::Vector3d& pt_2,
      double distance);

  void updateConstraint(const drake::systems::Context<double>& context);

  Eigen::VectorXd getC() const;

  Eigen::VectorXd getCDot() const;

  Eigen::MatrixXd getJ() const;

  Eigen::VectorXd getJdotv() const;

 private:
  const drake::multibody::MultibodyPlant<double>& plant_;

  const drake::multibody::Body<double>& body1_;
  const Eigen::Vector3d pt1_;
  const drake::multibody::Body<double>& body2_;
  const Eigen::Vector3d pt2_;
  double distance_;
  Eigen::VectorXd c_;
  Eigen::VectorXd cdot_;
  Eigen::MatrixXd J_;
  Eigen::VectorXd Jdotv_;
};

}  // namespace multibody
}  // namespace dairlib