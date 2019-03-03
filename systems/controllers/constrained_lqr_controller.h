#pragma once

#include "attic/multibody/contact_toolkit.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
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

  const drake::systems::OutputPort<double>& get_output_port_params() const {
    return this->get_output_port(output_port_params_index_);
  }
  int get_output_port_params_index() { return output_port_params_index_; }
  Eigen::MatrixXd get_A() { return A_; }
  Eigen::MatrixXd get_B() { return B_; }
  Eigen::MatrixXd get_K() { return K_; }
  Eigen::MatrixXd get_E() { return E_; }

 private:
  void CalcControl(const drake::systems::Context<double>& context,
                   AffineParams* control) const;

  const RigidBodyTree<double>& tree_;
  dairlib::multibody::ContactInfo contact_info_;
  std::unique_ptr<dairlib::multibody::ContactToolkit<drake::AutoDiffXd>>
      contact_toolkit_;
  int output_port_params_index_;
  Eigen::MatrixXd A_;
  Eigen::MatrixXd B_;
  Eigen::MatrixXd K_;
  Eigen::MatrixXd E_;
  Eigen::VectorXd x_desired_;
  drake::systems::controllers::LinearQuadraticRegulatorResult lqr_result_;
  const int num_positions_;
  const int num_velocities_;
  const int num_states_;
  const int num_efforts_;
  const int num_forces_;
};

}  // namespace systems
}  // namespace dairlib
