#pragma once

#include "attic/multibody/contact_toolkit.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/output_vector.h"

namespace dairlib {
namespace systems {

class ConstrainedLQRController : public drake::systems::LeafSystem<double> {
 public:
  ConstrainedLQRController(const RigidBodyTree<double>& tree,
                           Eigen::VectorXd q0, Eigen::VectorXd u0,
                           Eigen::VectorXd lambda0, Eigen::MatrixXd Q,
                           Eigen::MatrixXd R,
                           dairlib::multibody::ContactInfo contact_info =
                               dairlib::multibody::ContactInfo());

  const drake::systems::InputPort<double>& get_input_port_info() const {
    return this->get_input_port(input_port_info_index_);
  }

  const drake::systems::OutputPort<double>& get_output_port_efforts() const {
    return this->get_output_port(output_port_efforts_index_);
  }

  int get_input_port_info_index() { return input_port_info_index_; }
  int get_output_port_efforts_index() { return output_port_efforts_index_; }
  Eigen::MatrixXd get_K() { return K_; }
  Eigen::VectorXd get_E() { return E_; }
  Eigen::VectorXd get_x_desired() { return x_desired_; }
  Eigen::VectorXd get_A() { return A_; }
  Eigen::VectorXd get_B() { return B_; }
  drake::systems::controllers::LinearQuadraticRegulatorResult get_lqr_result() {
    return lqr_result_;
  }

 private:
  void CalcControl(const drake::systems::Context<double>& context,
                   TimestampedVector<double>* control) const;

  const RigidBodyTree<double>& tree_;
  dairlib::multibody::ContactInfo contact_info_;
  std::unique_ptr<dairlib::multibody::ContactToolkit<drake::AutoDiffXd>>
      contact_toolkit_;
  int input_port_info_index_;
  int output_port_efforts_index_;
  Eigen::MatrixXd K_;
  Eigen::VectorXd E_;
  Eigen::VectorXd x_desired_;
  Eigen::MatrixXd A_;
  Eigen::MatrixXd B_;
  drake::systems::controllers::LinearQuadraticRegulatorResult lqr_result_;
  const int num_positions_;
  const int num_velocities_;
  const int num_states_;
  const int num_efforts_;
  const int num_forces_;
};

}  // namespace systems
}  // namespace dairlib
