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

  // Creating the full state vector (Velocities are zero as it is a fixed point)
  VectorXd x(num_states_);
  VectorXd v = VectorXd::Zero(num_velocities_);
  x << q, v;
  desired_state_ = x;

  // Computing the full Jacobian (Position and contact)
  MatrixXd J_tree_qdot, J_tree_v, J_contact_qdot, J_contact_v;
  KinematicsCache<double> k_cache = tree_.doKinematics(q);

  // Two separate Jacobians -- in terms of qdot and v are computed as the number
  // of positions and velocities may be different (Quaternion base model)

  // Position Jacobian
  J_tree_qdot = tree_.positionConstraintsJacobian(k_cache, true);
  J_tree_v = tree_.positionConstraintsJacobian(k_cache, false);

  // Contact Jacobian (If contact information is provided)
  if (contact_info_.num_contacts > 0) {
    J_contact_qdot = autoDiffToValueMatrix(
        contact_toolkit_->CalcContactJacobian(initializeAutoDiff(x), true));
    J_contact_v = autoDiffToValueMatrix(
        contact_toolkit_->CalcContactJacobian(initializeAutoDiff(x), false));
  }

  // Stacking them up
  MatrixXd J_qdot(J_tree_qdot.rows() + J_contact_qdot.rows(),
                  J_tree_qdot.cols());
  MatrixXd J_v(J_tree_v.rows() + J_contact_v.rows(), J_tree_v.cols());
  J_qdot << J_tree_qdot, J_contact_qdot;
  J_v << J_tree_v, J_contact_v;

  // Computing F
  // F is the constraint matrix that represents the constraint in the form
  // Fx = 0 (where x is the full state vector of the model)
  MatrixXd F =
      MatrixXd::Zero(J_qdot.rows() + J_v.rows(), J_qdot.cols() + J_v.cols());
  F.block(0, 0, J_qdot.rows(), J_qdot.cols()) = J_qdot;
  F.block(J_qdot.rows(), J_qdot.cols(), J_v.rows(), J_v.cols()) = J_v;

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
}

void ConstrainedLQRController::CalcControl(
    const Context<double>& context, TimestampedVector<double>* control) const {
  const OutputVector<double>* info =
      (OutputVector<double>*)this->EvalVectorInput(context,
                                                   input_port_info_index_);

  // Computing the controller output.
  VectorXd u = K_ * (desired_state_ - info->GetState()) + E_;
  control->SetDataVector(u);
  control->set_timestamp(info->get_timestamp());
}

}  // namespace systems
}  // namespace dairlib
