#pragma once

#include "attic/multibody/contact_toolkit.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/output_vector.h"

namespace dairlib {
namespace systems {

/*
 * ConstrainedLQRController class that implements an LQR controller that also
 * takes into account constraints in the state space.
 *
 * The implementation follows the results from the publication:
 *
 * Posa, Michael, Scott Kuindersma, and Russ Tedrake. "Optimization and
 * stabilization of trajectories for constrained dynamical systems." 2016 IEEE
 * International Conference on Robotics and Automation (ICRA). IEEE, 2016.

 * The controller follows an affine structure. It is not a subclass of
 * AffineController due to difficulties in incorporating the required input
 * ports.
 * u = K(x_desired - x_current) + E
 * K is obtained using regular LQR on the reduced dimension space.
 * E is the by product of having to control the system at a particular u*.
 * The FixedPointSolver must be run before the controller is initialized to
 * obtain q*, u* and lambda* (Fixed point vectors).
 * The Q and R matrices need to be specified, which define the LQR cost
 * matrices.
 * If the system is in contact, ContactInfo may be provide and this information
 * is taken into account while computing the Jacobians. If it is not provided,
 * the controller assumes no contact.
 */
class ConstrainedLQRController : public drake::systems::LeafSystem<double> {
 public:
  /*
   * The constructor computes K and E and is the major computational block of
   * the controller class.
   */
  ConstrainedLQRController(const RigidBodyTree<double>& tree,
                           const Eigen::VectorXd& q, const Eigen::VectorXd& u,
                           const Eigen::VectorXd& lambda,
                           const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
                           const dairlib::multibody::ContactInfo& contact_info =
                               dairlib::multibody::ContactInfo());

  /*
   * Function to get the input port that takes in the current state
   * information.
   */
  const drake::systems::InputPort<double>& get_input_port_info() const {
    return this->get_input_port(input_port_info_index_);
  }
  /*
   * Function to get the output port that outputs the computed control inputs to
   * the actuators.
   */
  const drake::systems::OutputPort<double>& get_output_port_efforts() const {
    return this->get_output_port(output_port_efforts_index_);
  }

  /*
   * Function to get the index of the input port that takes in the current state
   * information.
   */
  int get_input_port_info_index() { return input_port_info_index_; }
  /*
   * Function to get the index of the output port that outputs the computed
   * control inputs to the actuators.
   */
  int get_output_port_efforts_index() { return output_port_efforts_index_; }
  /*
   * Function to get the gain matrix K as computed by the controller.
   */
  Eigen::MatrixXd get_K() { return K_; }
  /*
   * Function to get the E vector as computed by the controller.
   */
  Eigen::VectorXd get_E() { return E_; }
  Eigen::VectorXd get_desired_state() { return desired_state_; }
  /*
   * Function to get the A matrix in the linearized equation xdot = Ax + Bu.
   * This is the matrix after converting the system to the reduced dimensions.
   */
  Eigen::MatrixXd get_A() { return A_; }
  /*
   * Function to get the B matrix in the linearized equation xdot = Ax + Bu.
   * This is the matrix after converting the system to the reduced dimensions.
   */
  Eigen::MatrixXd get_B() { return B_; }
  Eigen::MatrixXd get_Q() { return Q_; }
  Eigen::MatrixXd get_R() { return R_; }
  /*
   * Function to get the result of the LQR call. It is of type
   * LinearQuadraticRegulatorResult and is a structure consisting of the LQR
   * solutions K and S.
   */
  drake::systems::controllers::LinearQuadraticRegulatorResult get_lqr_result() {
    return lqr_result_;
  }

 private:
  void CalcControl(const drake::systems::Context<double>& context,
                   TimestampedVector<double>* control) const;

  const RigidBodyTree<double>& tree_;
  const dairlib::multibody::ContactInfo& contact_info_;
  std::unique_ptr<dairlib::multibody::ContactToolkit<drake::AutoDiffXd>>
      contact_toolkit_;
  int input_port_info_index_;
  int output_port_efforts_index_;
  Eigen::MatrixXd K_;
  Eigen::VectorXd E_;
  Eigen::VectorXd desired_state_;
  Eigen::MatrixXd A_;
  Eigen::MatrixXd B_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;
  drake::systems::controllers::LinearQuadraticRegulatorResult lqr_result_;
  const int num_positions_;
  const int num_velocities_;
  const int num_states_;
  const int num_efforts_;
  const int num_forces_;
};

}  // namespace systems
}  // namespace dairlib
