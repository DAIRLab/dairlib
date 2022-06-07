#include "systems/controllers/constrained_lqr_controller.h"
#include "multibody/multibody_utils.h"

#include "drake/math/autodiff_gradient.h"

namespace dairlib {
namespace systems {

using Eigen::VectorXd;
using Eigen::MatrixXd;
using drake::MatrixX;
using drake::VectorX;
using drake::AutoDiffXd;
using drake::AutoDiffVecXd;
using drake::math::InitializeAutoDiff;
using drake::math::ExtractGradient;
using drake::math::ExtractValue;
using drake::systems::Context;
using drake::systems::controllers::LinearQuadraticRegulator;

ConstrainedLQRController::ConstrainedLQRController(
      const multibody::KinematicEvaluatorSet<AutoDiffXd>& evaluators,
      const Context<AutoDiffXd>& context, const VectorXd& lambda,
      const MatrixXd& Q, const Eigen::MatrixXd& R)
    : evaluators_(evaluators),
      plant_(evaluators.plant()),
      num_forces_(evaluators.count_full()) {
  // Input port that takes in an OutputVector containing the current Cassie
  // state
  input_port_info_index_ =
      this->DeclareVectorInputPort("x, u, t",
                                   OutputVector<double>(plant_.num_positions(),
                                                        plant_.num_velocities(),
                                                        plant_.num_actuators()))
          .get_index();

  // Output port that outputs the efforts
  output_port_efforts_index_ =
      this->DeclareVectorOutputPort(
              "u, t", TimestampedVector<double>(plant_.num_actuators()),
              &ConstrainedLQRController::CalcControl)
          .get_index();

  // checking the validity of the dimensions of the parameters
  DRAKE_DEMAND(lambda.size() == num_forces_);
  DRAKE_DEMAND(Q.rows() == plant_.num_positions() + plant_.num_velocities());
  DRAKE_DEMAND(Q.rows() == Q.cols());
  DRAKE_DEMAND(R.rows() == plant_.num_actuators());
  DRAKE_DEMAND(R.rows() == R.cols());

  auto J_active_v = evaluators_.EvalActiveJacobian(context);

  // convert to w.r.t. qdot one column at a time
  MatrixX<AutoDiffXd> J_active_qdot(J_active_v.rows(), plant_.num_positions());
  for (int i = 0; i < plant_.num_positions(); i++) {
    AutoDiffVecXd v_i(plant_.num_velocities());
    AutoDiffVecXd qdot = AutoDiffVecXd::Zero(plant_.num_positions());
    qdot(i) = 1;
    plant_.MapQDotToVelocity(context, qdot, &v_i);
    J_active_qdot.col(i) = J_active_v * v_i;
  }

  // Add quaternion constraints to F
  // Constraints come from the kinematic fact that ||q|| = 1, linearized is
  // q_0^T delta_q = 0
  // Note that there is no corresponding constraint on angular velocity, which
  // is already 3-dimensional (not 4).
  int num_quat = 0;
  std::vector<int> quat_start;
  auto bodies = plant_.GetFloatingBaseBodies();
  for (auto body : bodies) {
    if (plant_.get_body(body).has_quaternion_dofs()) {
      num_quat++;
      quat_start.push_back(plant_.get_body(body).floating_positions_start());
    }
  }

  // F = [J_active_qdot, 0         ]
  //     [0            , J_active_v]
  //     [          F_quat         ]
  // Note that d/dt J = 0 since this is time-invariant.
  MatrixXd F_quat =
      MatrixXd::Zero(num_quat, J_active_qdot.cols() + J_active_v.cols());
  for (int i = 0; i < num_quat; i++) {
    F_quat.row(i).segment(quat_start.at(i), 4) = ExtractValue(
        plant_.GetPositions(context).segment(quat_start.at(i),4));
  }

  // Computing F
  // F is the constraint matrix that represents the constraint in the form
  // Fx = 0 (where x is the full state vector of the model)
  MatrixXd F(J_active_qdot.rows() + J_active_v.rows() + num_quat,
             J_active_qdot.cols() + J_active_v.cols());
  F << ExtractValue(J_active_qdot),
       MatrixXd::Zero(J_active_qdot.rows(), J_active_v.cols()),
       MatrixXd::Zero(J_active_v.rows(), J_active_qdot.cols()),
       ExtractValue(J_active_v),
       F_quat;

  // Computing the null space of F
  Eigen::HouseholderQR<MatrixXd> qr_decomp(F.transpose());
  MatrixXd q_decomp = qr_decomp.householderQ();

  MatrixXd P =
      q_decomp.block(0, F.rows(), q_decomp.rows(), q_decomp.cols() - F.rows());
  P.transposeInPlace();

  // To compute the linearization, xdot (autodiff) is computed
  // Creating a combined autodiff vector and then extracting the individual
  // components to ensure proper gradient initialization.

  VectorXd xu(plant_.num_positions() + plant_.num_velocities()
      + plant_.num_actuators());
  auto x = ExtractValue(plant_.GetPositionsAndVelocities(context));
  auto u =
      ExtractValue(plant_.get_actuation_input_port().Eval(context));
  xu << x, u;
  AutoDiffVecXd xu_ad = InitializeAutoDiff(xu);

  AutoDiffVecXd x_ad = xu_ad.head(plant_.num_positions()
      + plant_.num_velocities());
  AutoDiffVecXd u_ad = xu_ad.segment(plant_.num_positions()
      + plant_.num_velocities(), plant_.num_actuators());

  auto context_ad = multibody::CreateContext<AutoDiffXd>(plant_, x_ad, u_ad);

  AutoDiffVecXd xdot = evaluators_.CalcTimeDerivatives(*context_ad);

  MatrixXd AB = ExtractGradient(xdot);
  MatrixXd A = AB.leftCols(plant_.num_positions() + plant_.num_velocities());
  MatrixXd B = AB.rightCols(plant_.num_actuators());

  A_full_ = A;
  B_full_ = B;

  // A and B matrices in the new coordinates
  A_ = P * A * P.transpose();
  B_ = P * B;
  // Remapping the Q costs to the new coordinates
  Q_ = P * Q * P.transpose();
  R_ = R;
  F_ = F;
  P_ = P;

  // Validating the required dimesions after the matrix operations.
  DRAKE_DEMAND(B_.cols() == R_.rows());

  lqr_result_ = LinearQuadraticRegulator(A_, B_, Q_, R_);
  K_ = lqr_result_.K * P;
  E_ = u;
  desired_state_ = x;
}

void ConstrainedLQRController::CalcControl(
    const Context<double>& context, TimestampedVector<double>* control) const {
  const OutputVector<double>* info =
      (OutputVector<double>*)this->EvalVectorInput(context,
                                                   input_port_info_index_);

  // Computing the controller output.
  VectorXd dx = (desired_state_ - info->GetState());
  VectorXd u = K_ * dx + E_;
  control->SetDataVector(u);
  control->set_timestamp(info->get_timestamp());
}

}  // namespace systems
}  // namespace dairlib
