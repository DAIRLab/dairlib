#include "systems/controllers/constrained_lqr_controller.h"

namespace dairlib {
namespace systems {

using std::make_unique;
using std::unique_ptr;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::HouseholderQR;
using dairlib::multibody::ContactInfo;
using dairlib::multibody::ContactToolkit;
using drake::AutoDiffXd;
using drake::AutoDiffVecXd;
using drake::math::initializeAutoDiff;
using drake::math::autoDiffToGradientMatrix;
using drake::math::autoDiffToValueMatrix;
using drake::systems::Context;
using drake::systems::controllers::LinearQuadraticRegulator;
using drake::systems::controllers::LinearQuadraticRegulatorResult;

ConstrainedLQRController::ConstrainedLQRController(
    const RigidBodyTree<double>& tree, const VectorXd& q, const VectorXd& u,
    const VectorXd& lambda, const MatrixXd& Q, const MatrixXd& R,
    const ContactInfo& contact_info)
    : tree_(tree),
      contact_info_(contact_info),
      contact_toolkit_(
          make_unique<ContactToolkit<AutoDiffXd>>(tree, contact_info)),
      num_positions_(tree.get_num_positions()),
      num_velocities_(tree.get_num_velocities()),
      num_states_(num_positions_ + num_velocities_),
      num_efforts_(tree.get_num_actuators()),
      num_forces_(tree.getNumPositionConstraints() +
                  contact_info.num_contacts * 3) {
  // Input port that takes in an OutputVector containing the current Cassie
  // state
  input_port_info_index_ =
      this
          ->DeclareVectorInputPort(OutputVector<double>(
              num_positions_, num_velocities_, num_efforts_))
          .get_index();

  // Output port that outputs the efforts
  output_port_efforts_index_ =
      this->DeclareVectorOutputPort(TimestampedVector<double>(num_efforts_),
                                    &ConstrainedLQRController::CalcControl)
          .get_index();

  // checking the validity of the dimensions of the parameters
  DRAKE_DEMAND(q.size() == num_positions_);
  DRAKE_DEMAND(u.size() == num_efforts_);
  DRAKE_DEMAND(lambda.size() == num_forces_);
  DRAKE_DEMAND(Q.rows() == num_positions_ + num_velocities_);
  DRAKE_DEMAND(Q.rows() == Q.cols());
  DRAKE_DEMAND(R.rows() == num_efforts_);
  DRAKE_DEMAND(R.rows() == R.cols());

  // Boolean variable to decide whether to represent the Jacobians using qdot or
  // not.
  // For quaternions (num_positions != num_velocities), we need it in terms of
  // the generalized velocities, hence it is set to false.
  in_terms_of_qdot_ = true;
  if (tree.get_num_positions() != tree.get_num_velocities()) {
    in_terms_of_qdot_ = false;
  }

  // Creating the full state vector (Velocities are zero as it is a fixed point)
  VectorXd x(num_states_);
  VectorXd v = VectorXd::Zero(num_velocities_);
  x << q, v;
  desired_state_ = x;

  // Computing the full Jacobian (Position and contact)
  MatrixXd J_tree, J_contact;
  KinematicsCache<double> k_cache = tree_.doKinematics(q);

  // Position Jacobian
  J_tree = tree_.positionConstraintsJacobian(k_cache, true);

  // Contact Jacobian (If contact information is provided)
  if (contact_info_.num_contacts > 0) {
    J_contact = autoDiffToValueMatrix(
        contact_toolkit_->CalcContactJacobian(initializeAutoDiff(x), false));
    std::cout << J_contact.rows() << " " << J_contact.cols() << std::endl;
  }

  MatrixXd J(J_tree.rows() + J_contact.rows(), num_positions_);
  J << J_tree, J_contact;

  // Computing F
  // F is the constraint matrix that represents the constraint in the form
  // Fx = 0 (where x is the full state vector of the model)
  MatrixXd F = MatrixXd::Zero(2 * J.rows(), 2 * J.cols());
  F.block(0, 0, J.rows(), J.cols()) = J;
  F.block(J.rows(), J.cols(), J.rows(), J.cols()) = J;

  // Computing the null space of F
  HouseholderQR<MatrixXd> qr_decomp(F.transpose());
  MatrixXd q_decomp = qr_decomp.householderQ();

  MatrixXd P =
      q_decomp.block(0, F.rows(), q_decomp.rows(), q_decomp.cols() - F.rows());
  P.transposeInPlace();

  // To compute the linearization, xdot (autodiff) is computed
  // Creating a combined autodiff vector and then extracting the individual
  // components to ensure proper gradient initialization.
  VectorXd xul(num_states_ + num_efforts_ + num_forces_);
  xul << x, u, lambda;
  AutoDiffVecXd xul_autodiff = initializeAutoDiff(xul);
  AutoDiffVecXd x_autodiff = xul_autodiff.head(num_states_);
  AutoDiffVecXd u_autodiff = xul_autodiff.segment(num_states_, num_efforts_);
  AutoDiffVecXd lambda_autodiff = xul_autodiff.tail(num_forces_);

  // xdot
  AutoDiffVecXd xdot_autodiff = contact_toolkit_->CalcTimeDerivatives(
      x_autodiff, u_autodiff, lambda_autodiff);

  // Making sure that the derivative is zero
  // DRAKE_DEMAND(autoDiffToValueMatrix(xdot_autodiff).isZero(1e-6));

  MatrixXd AB = autoDiffToGradientMatrix(xdot_autodiff);
  MatrixXd A = AB.leftCols(num_states_);
  MatrixXd B = AB.block(0, num_states_, AB.rows(), num_efforts_);

  // A and B matrices in the new coordinates
  A_ = P * A * P.transpose();
  B_ = P * B;
  // Remapping the Q costs to the new coordinates
  Q_ = P * Q * P.transpose();
  R_ = R;

  // Validating the required dimesions after the matrix operations.
  DRAKE_DEMAND(B_.cols() == R_.rows());

  lqr_result_ = LinearQuadraticRegulator(A_, B_, Q_, R_);
  K_ = lqr_result_.K * P;
  E_ = u;

  std::cout << A.rows() << " " << A.cols() << std::endl;
  std::cout << B.rows() << " " << B.cols() << std::endl;
  std::cout << P.rows() << " " << P.cols() << std::endl;
  std::cout << A_.rows() << " " << A_.cols() << std::endl;
  std::cout << B_.rows() << " " << B_.cols() << std::endl;
  std::cout << K_.rows() << " " << K_.cols() << std::endl;

}

void ConstrainedLQRController::CalcControl(
    const Context<double>& context, TimestampedVector<double>* control) const {
  const OutputVector<double>* info =
      (OutputVector<double>*)this->EvalVectorInput(context,
                                                   input_port_info_index_);

  // Computing the controller output.
  VectorXd u = K_ * (desired_state_ - info->GetState()) + E_;
  //std::cout << "---" << std::endl;
  //std::cout << desired_state_.size() << std::endl;
  //std::cout << K_.rows() << K_.cols() << std::endl;
  //std::cout << info->GetState().size() << std::endl;
  //std::cout << E_.size() << std::endl;
  //std::cout << "---" << std::endl;
  control->SetDataVector(u);
  control->set_timestamp(info->get_timestamp());
}

}  // namespace systems
}  // namespace dairlib
