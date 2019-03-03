#pragma once

#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "attic/multibody/contact_toolkit.h"
#include "systems/controllers/affine_controller.h"

namespace dairlib {
namespace systems {

class ConstrainedLQRController : public AffineController {
 public:
  ConstrainedLQRController(const RigidBodyTree<double>& tree,
                           dairlib::multibody::ContactInfo contact_info =
                               dairlib::multibody::ContactInfo());
  void SetupController(Eigen::VectorXd q0, Eigen::VectorXd u0,
                       Eigen::VectorXd lambda0, Eigen::MatrixXd Q,
                       Eigen::MatrixXd R);

  Eigen::MatrixXd get_A() { return A_; }
  Eigen::MatrixXd get_B() { return B_; }
  Eigen::MatrixXd get_K() { return K_; }
  Eigen::MatrixXd get_E() { return E_; }

 private:
  const RigidBodyTree<double>& tree_;
  dairlib::multibody::ContactInfo contact_info_;
  std::unique_ptr<dairlib::multibody::ContactToolkit<drake::AutoDiffXd>>
      contact_toolkit_;
  Eigen::MatrixXd A_;
  Eigen::MatrixXd B_;
  Eigen::MatrixXd K_;
  Eigen::MatrixXd E_;
  drake::systems::controllers::LinearQuadraticRegulatorResult lqr_result_;
  const int num_positions_;
  const int num_velocities_;
  const int num_states_;
  const int num_efforts_;
  const int num_forces_;
};

}  // namespace systems
}  // namespace dairlib
