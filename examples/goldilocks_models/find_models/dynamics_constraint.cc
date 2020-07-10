#include "examples/goldilocks_models/find_models/dynamics_constraint.h"

#include "multibody/multibody_utils.h"

using std::map;
using std::string;
using std::vector;
using std::list;
using std::unique_ptr;
using std::make_unique;
using std::make_shared;
using std::isnan;
using std::isinf;

using Eigen::Dynamic;
using Eigen::AutoDiffScalar;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Matrix;
using Eigen::MatrixXd;
using drake::VectorX;
using drake::MatrixX;
using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::math::DiscardGradient;
using drake::math::autoDiffToValueMatrix;
using drake::math::autoDiffToGradientMatrix;
using drake::math::initializeAutoDiff;
using drake::solvers::to_string;
using drake::solvers::VectorXDecisionVariable;
using drake::solvers::Constraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::Constraint;
using drake::solvers::VariableRefList;
using drake::solvers::Binding;
using drake::symbolic::Variable;
using drake::symbolic::Expression;
using drake::multibody::MultibodyPlant;

namespace dairlib {
namespace goldilocks_models {
namespace find_models {

DynamicsConstraint::DynamicsConstraint(const ReducedOrderModel& rom,
                                       const MultibodyPlant<double>& plant,
                                       bool is_head,
                                       const std::string& description)
    : NonlinearConstraint<double>(
          rom.n_yddot(),
          2 * (plant.num_positions() + plant.num_velocities() + rom.n_tau()) +
              1,
          VectorXd::Zero(rom.n_yddot()), VectorXd::Zero(rom.n_yddot()),
          description),
      rom_(rom.Clone()),
      n_q_(plant.num_positions()),
      n_v_(plant.num_velocities()),
      n_u_(plant.num_actuators()),
      n_tau_(rom.n_tau()),
      is_head_(is_head),
      plant_(plant),
      context_(plant.CreateDefaultContext()) {}

// Getters
VectorXd DynamicsConstraint::GetY(
    const VectorXd& q, const drake::systems::Context<double>& context) const {
  return rom_->EvalMappingFunc(q, context);
};
VectorXd DynamicsConstraint::GetYdot(
    const VectorXd& x, const drake::systems::Context<double>& context) const {
  return rom_->EvalMappingFuncJV(x.head(n_q_), x.tail(n_v_), context);
};
VectorXd DynamicsConstraint::GetY(const VectorXd& q) const {
  plant_.SetPositions(context_.get(), q);
  return rom_->EvalMappingFunc(q, *context_);
};
VectorXd DynamicsConstraint::GetYdot(const VectorXd& x) const {
  plant_.SetPositionsAndVelocities(context_.get(), x);
  return rom_->EvalMappingFuncJV(x.head(n_q_), x.tail(n_v_), *context_);
};
VectorXd DynamicsConstraint::GetYddot(const VectorXd& y, const VectorXd& ydot,
                                      const VectorXd& tau) const {
  return rom_->EvalDynamicFunc(y, ydot, tau);
};

void DynamicsConstraint::EvaluateConstraint(
  const Eigen::Ref<const drake::VectorX<double>>& x,
  drake::VectorX<double>* y) const {

  // Extract elements
  VectorXd x_i = x.head(n_q_ + n_v_);
  VectorXd tau_i = x.segment(n_q_ + n_v_, n_tau_);
  VectorXd x_iplus1 = x.segment(n_q_ + n_v_ + n_tau_, n_q_ + n_v_);
  VectorXd tau_iplus1 = x.segment(2 * (n_q_ + n_v_) + n_tau_, n_tau_);
  const VectorXd h_i = x.tail(1);

  *y = EvalConstraintWithModelParams(x_i, tau_i, x_iplus1, tau_iplus1, h_i,
                                     rom_->theta_y(), rom_->theta_yddot());
}
VectorXd DynamicsConstraint::EvalConstraintWithModelParams(
  const VectorXd & x_i, const VectorXd & tau_i,
  const VectorXd & x_iplus1, const VectorXd & tau_iplus1,
  const VectorXd & h_i,
  const VectorXd & theta_y, const VectorXd & theta_yddot) const {

  // Get s and ds at knot i and i+1
  rom_->SetThetaY(theta_y);
  rom_->SetThetaYddot(theta_yddot);
  plant_.SetPositionsAndVelocities(context_.get(), x_i);
  VectorXd s_i = GetY(x_i.head(n_q_), *context_);
  VectorXd ds_i = GetYdot(x_i, *context_);
  plant_.SetPositionsAndVelocities(context_.get(), x_iplus1);
  VectorXd s_iplus1 = GetY(x_iplus1.head(n_q_), *context_);
  VectorXd ds_iplus1 = GetYdot(x_iplus1, *context_);

  // cout << "s_i = " << s_i.transpose() << endl;
  // cout << "ds_i = " << ds_i.transpose() << endl;
  // cout << "s_iplus1 = " << s_iplus1.transpose() << endl;
  // cout << "ds_iplus1 = " << ds_iplus1.transpose() << endl;
  // cout << "dds(0) = " << (2 * (-3 * (s_i - s_iplus1) - h_i(0) * (ds_iplus1 + 2 * ds_i)) /
  //          (h_i(0) * h_i(0))).transpose() << endl;

  // Get constraint value
  if (is_head_) {
    return 2 * (-3 * (s_i - s_iplus1) - h_i(0) * (ds_iplus1 + 2 * ds_i)) /
               (h_i(0) * h_i(0)) -
           rom_->EvalDynamicFunc(s_i, ds_i, tau_i);
  } else {
    return (6 * (s_i - s_iplus1) + h_i(0) * (4 * ds_iplus1 + 2 * ds_i)) /
               (h_i(0) * h_i(0)) -
           rom_->EvalDynamicFunc(s_iplus1, ds_iplus1, tau_iplus1);
  }
}

MatrixXd DynamicsConstraint::getGradientWrtTheta(
  const VectorXd & x_i_double, const VectorXd & tau_i,
  const VectorXd & x_iplus1_double, const VectorXd & tau_iplus1,
  const VectorXd & h_i_double) const {
  // It's a nonlinear function in theta, so we use autoDiff to get the gradient.
  // The calculation here will not be the same as the one in eval(), because
  // we have totally different autodiff, and the second autodiff requires
  // costumization.
  // Also, since they are nonilnear, each row might have different gradient.

  // You'll need to create autoDiff yourself first, cause the input is double
  // and you need to jacobian to get ds.

  // ////////// V1: Do forward differencing wrt theta //////////////////////////
  /*
  // Get the gradient wrt theta_y and theta_yddot
  VectorXd theta(n_theta_y_ + n_theta_yddot_);
  theta << theta_y_, theta_yddot_;
  MatrixXd gradWrtTheta(n_y_, theta.size());
  vector<VectorXd> y_vec;
  for (int k = 0; k < theta.size(); k++) {
    for (double shift : fd_shift_vec_) {
      theta(k) += shift;

      VectorXd theta_y = theta.head(n_theta_y_);
      VectorXd theta_yddot = theta.tail(n_theta_yddot_);

      // Evaluate constraint value
      // y_vec.push_back(autoDiffToValueMatrix(getConstraintValueInAutoDiff(
      //                                         x_i, x_iplus1, h_i,
      //                                         theta_y, theta_yddot)));
      y_vec.push_back(EvalConstraintWithModelParams(
                        x_i_double, tau_i,
                        x_iplus1_double, tau_iplus1,
                        h_i_double,
                        theta_y, theta_yddot));

      theta(k) -= shift;
    }

    // Get gradient
    gradWrtTheta.col(k) = (y_vec[1] - y_vec[0]) / eps_fd_;
    y_vec.clear();
  }*/

  // ////////// V2: Do central differencing wrt theta //////////////////////////
  // Get the gradient wrt theta_y and theta_yddot
  VectorXd theta = rom_->theta();
  MatrixXd gradWrtTheta(rom_->n_y(), theta.size());
  vector<VectorXd> y_vec;
  for (int k = 0; k < theta.size(); k++) {
    for (double shift : cd_shift_vec_) {
      theta(k) += shift;

      // Evaluate constraint value
      // y_vec.push_back(autoDiffToValueMatrix(getConstraintValueInAutoDiff(
      //                                         x_i, x_iplus1, h_i,
      //                                         theta.head(rom_->n_theta_y()),
      //                                         theta.tail(rom_->n_theta_yddot())));
      y_vec.push_back(EvalConstraintWithModelParams(
          x_i_double, tau_i, x_iplus1_double, tau_iplus1, h_i_double,
          theta.head(rom_->n_theta_y()), theta.tail(rom_->n_theta_yddot())));

      theta(k) -= shift;
    }

    // Get gradient
    gradWrtTheta.col(k) = (y_vec[1] - y_vec[0]) / eps_cd_;
    y_vec.clear();
  }

  // /////////// V3: higher order method of finite difference //////////////////
  // Reference: https://en.wikipedia.org/wiki/Numerical_differentiation#Higher-order_methods

  /*
  // Get the gradient wrt theta_y and theta_yddot
  VectorXd theta(n_theta_y_ + n_theta_yddot_);
  theta << theta_y_, theta_yddot_;
  MatrixXd gradWrtTheta(n_y_, theta.size());
  vector<VectorXd> y_vec;
  for (int k = 0; k < theta.size(); k++) {
    for (double shift : ho_shift_vec_) {
      theta(k) += shift;

      VectorXd theta_y = theta.head(n_theta_y_);
      VectorXd theta_yddot = theta.tail(n_theta_yddot_);

      // Evaluate constraint value
      // y_vec.push_back(autoDiffToValueMatrix(getConstraintValueInAutoDiff(
      //                                         x_i, x_iplus1, h_i,
      //                                         theta_y, theta_yddot)));
      y_vec.push_back(EvalConstraintWithModelParams(
                        x_i_double, tau_i,
                        x_iplus1_double, tau_iplus1,
                        h_i_double,
                        theta_y, theta_yddot));

      theta(k) -= shift;
    }

    // Get gradient
    gradWrtTheta.col(k) =
      (-y_vec[3] + 8 * y_vec[2] - 8 * y_vec[1] + y_vec[0]) / (3 * eps_ho_);
    y_vec.clear();
  }*/

  //////////////////////////////////////////////////////////////////////////////

  // Finding the optimal step size
  // https://math.stackexchange.com/questions/815113/is-there-a-general-formula-for-estimating-the-step-size-h-in-numerical-different
  // https://www.uio.no/studier/emner/matnat/math/MAT-INF1100/h10/kompendiet/kap11.pdf

  return gradWrtTheta;
}

// Careful: need to initialize the size of s and ds before calling GetYAndYdotInDouble
/*void DynamicsConstraint::GetYAndYdotInDouble(const VectorXd& x, VectorXd& s,
                                             VectorXd& ds) const {
  VectorXd q = x.head(n_q_);
  VectorXd v = x.tail(n_v_);

  s = rom_->EvalMappingFunc(q);
  ds = rom_->EvalMappingFuncJV(q, v);

  // TODO: check can you incorporate the following old API here for
  // computeTauToExtendModel()
  //  s = kin_expression_double_.getExpression(theta_y, q,
  //                                           theta_y.size() / n_feature_y_);
}*/

VectorXd DynamicsConstraint::computeTauToExtendModel(
  const VectorXd & x_i_double, const VectorXd & x_iplus1_double,
  const VectorXd & h_i, const VectorXd & theta_y_append) {
  // TODO: finish implementing the rest of the ROM
  throw std::runtime_error("Not implemented");
/*
  // Reset the model dimension, so that we can get the extended part of the model
  int n_extend = theta_y_append.rows() / n_feature_y_;
  kin_expression_.setModelDimension(n_extend);

  // Get s and ds at knot i and i+1
  VectorXd s_i = VectorXd::Zero(n_extend);
  VectorXd ds_i = VectorXd::Zero(n_extend);
  VectorXd s_iplus1 = VectorXd::Zero(n_extend);
  VectorXd ds_iplus1 = VectorXd::Zero(n_extend);
  GetYAndYdotInDouble(x_i_double, s_i, ds_i, theta_y_append);
  GetYAndYdotInDouble(x_iplus1_double, s_iplus1, ds_iplus1, theta_y_append);

  // Set the model dimension back just in case
  kin_expression_.setModelDimension(n_y_);

  // Get constraint value (without h(s,ds) and tau)
  if (is_head_) {
    return 2 * (-3 * (s_i - s_iplus1) - h_i(0) * (ds_iplus1 + 2 * ds_i)) /
           (h_i(0) * h_i(0));
  } else {
    return (6 * (s_i - s_iplus1) + h_i(0) * (4 * ds_iplus1 + 2 * ds_i)) /
           (h_i(0) * h_i(0));
  }*/
}

}  // namespace find_models
}  // namespace goldilocks_models
}  // namespace dairlib
