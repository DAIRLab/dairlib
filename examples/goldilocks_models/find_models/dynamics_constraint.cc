#include "examples/goldilocks_models/find_models/dynamics_constraint.h"

#include "multibody/multibody_utils.h"

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
      is_head_(is_head) {}

// Getters
VectorXd DynamicsConstraint::GetY(const VectorXd& q) const {
  return rom_->EvalMappingFunc(q);
};
VectorXd DynamicsConstraint::GetYdot(const VectorXd& x) const {
  return rom_->EvalMappingFuncJV(x.head(n_q_), x.tail(n_v_));
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
  VectorXd s_i = GetY(x_i.head(n_q_));
  VectorXd ds_i = GetYdot(x_i);
  VectorXd s_iplus1 = GetY(x_iplus1.head(n_q_));
  VectorXd ds_iplus1 = GetYdot(x_iplus1);

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




////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Below is Autodiff version (DoEval is implemented in Autodiff one)

//DynamicsConstraintAutodiffVersion::DynamicsConstraintAutodiffVersion(
//  int n_y, int n_feature_y,
//  const VectorXd & theta_y,
//  int n_yddot, int n_feature_yddot,
//  const VectorXd & theta_yddot,
//  int n_tau,
//  MatrixXd B_tau,
//  const MultibodyPlant<AutoDiffXd> * plant,
//  bool is_head,
//  int robot_option,
//  const std::string& description):
//  Constraint(n_yddot,
//             2 * (plant->num_positions() + plant->num_velocities() + n_tau) + 1,
//             VectorXd::Zero(n_yddot),
//             VectorXd::Zero(n_yddot),
//             description),
//  plant_(plant),
//  n_q_(plant->num_positions()),
//  n_v_(plant->num_velocities()),
//  n_y_(n_y),
//  n_feature_y_(n_feature_y),
//  n_theta_y_(theta_y.size()),
//  theta_y_(theta_y),
//  n_yddot_(n_yddot),
//  n_feature_yddot_(n_feature_yddot),
//  n_theta_yddot_(theta_yddot.size()),
//  theta_yddot_(theta_yddot),
//  n_tau_(n_tau),
//  kin_expression_(KinematicsExpression<AutoDiffXd>(n_y, n_feature_y, plant,
//                  robot_option)),
//  dyn_expression_(DynamicsExpression(n_yddot, n_feature_yddot, B_tau,
//                                     robot_option)),
//  is_head_(is_head) {
//
//  // Check the theta size
//  DRAKE_DEMAND(n_y * n_feature_y == theta_y.size());
//
//  // Check the feature size implemented in the model expression
//  AutoDiffVecXd q_temp =
//    initializeAutoDiff(VectorXd::Zero(plant->num_positions()));
//  DRAKE_DEMAND(n_feature_y == kin_expression_.getFeature(q_temp).size());
//
//  // Check the theta size
//  DRAKE_DEMAND(n_yddot * n_feature_yddot == theta_yddot.size());
//
//  // Check the feature size implemented in the model expression
//  VectorXd s_temp = VectorXd::Zero(n_yddot);
//  VectorXd ds_temp = VectorXd::Zero(n_yddot);
//  DRAKE_DEMAND(n_feature_yddot ==
//               dyn_expression_.getFeature(s_temp, ds_temp).size());
//}
//
//
//void DynamicsConstraintAutodiffVersion::DoEval(const
//    Eigen::Ref<const Eigen::VectorXd>& q,
//    Eigen::VectorXd* y) const {
//  AutoDiffVecXd y_t;
//  Eval(initializeAutoDiff(q), &y_t);
//  *y = autoDiffToValueMatrix(y_t);
//}
//
//void DynamicsConstraintAutodiffVersion::DoEval(const
//    Eigen::Ref<const AutoDiffVecXd>& qvtqvth,
//    AutoDiffVecXd* y) const {
//  // Extract elements
//  AutoDiffVecXd x_i = qvtqvth.head(n_q_ + n_v_);
//  AutoDiffVecXd tau_i = qvtqvth.segment(n_q_ + n_v_, n_tau_);
//  AutoDiffVecXd x_iplus1 = qvtqvth.segment(n_q_ + n_v_ + n_tau_, n_q_ + n_v_);
//  AutoDiffVecXd tau_iplus1 = qvtqvth.segment(2 * (n_q_ + n_v_) + n_tau_, n_tau_);
//  const AutoDiffVecXd h_i = qvtqvth.tail(1);
//
//  // Impose dynamics constraint
//  // Way 1 /////////////////////////////////////////////////////////////////////
//  *y = getConstraintValueInAutoDiff(x_i, tau_i, x_iplus1, tau_iplus1, h_i,
//                                    theta_y_, theta_yddot_);
//
//
//  // Way 2 /////////////////////////////////////////////////////////////////////
//  // (not up-to-date)
//  // rhs is copied from DynamicsExpression.getExpression().
//  /*// Get s and ds at knot i and i+1
//  AutoDiffVecXd s_i = initializeAutoDiff(VectorXd::Zero(n_y_));
//  AutoDiffVecXd ds_i = initializeAutoDiff(VectorXd::Zero(n_y_));
//  AutoDiffVecXd s_iplus1 = initializeAutoDiff(VectorXd::Zero(n_y_));
//  AutoDiffVecXd ds_iplus1 = initializeAutoDiff(VectorXd::Zero(n_y_));
//  getSAndSDotInAutoDiff(x_i, s_i, ds_i, 0, theta_y_);
//  getSAndSDotInAutoDiff(x_iplus1, s_iplus1, ds_iplus1, n_q_ + n_v_ + n_tau_, theta_y_);
//
//  // Get constraint value in autoDiff
//  if (is_head_) {
//    AutoDiffVecXd lhs =
//      2 * (-3 * (s_i - s_iplus1) - h_i(0) * (ds_iplus1 + 2 * ds_i)) /
//      (h_i(0) * h_i(0));
//
//    // AutoDiffVecXd rhs =
//    //   dyn_expression_.getExpression(theta_yddot_, s_i, ds_i, tau_i);
//    AutoDiffVecXd rhs = initializeAutoDiff(VectorXd::Zero(n_yddot_));
//    for (int i = 0; i < n_yddot_; i++)
//      rhs(i) = theta_yddot_.segment(i * n_feature_yddot_, n_feature_yddot_).dot(
//                 dyn_expression_.getFeature(s_i, ds_i));
//
//    *y = lhs - rhs;
//  }
//  else {
//    AutoDiffVecXd lhs =
//      (6 * (s_i - s_iplus1) + h_i(0) * (4 * ds_iplus1 + 2 * ds_i)) /
//      (h_i(0) * h_i(0));
//
//    // AutoDiffVecXd rhs =
//    //   dyn_expression_.getExpression(theta_yddot_, s_iplus1, ds_iplus1, tau_iplus1);
//    AutoDiffVecXd rhs = initializeAutoDiff(VectorXd::Zero(n_yddot_));
//    for (int i = 0; i < n_yddot_; i++)
//      rhs(i) = theta_yddot_.segment(i * n_feature_yddot_, n_feature_yddot_).dot(
//                 dyn_expression_.getFeature(s_iplus1, ds_iplus1));
//
//    *y = lhs - rhs;
//  }*/
//  //////////////////////////////////////////////////////////////////////////////
//}
//
//void DynamicsConstraintAutodiffVersion::DoEval(const
//    Eigen::Ref<const VectorX<Variable>>& x,
//    VectorX<Expression>*y) const {
//  throw std::logic_error(
//    "This constraint class does not support symbolic evaluation.");
//}
//
//
//AutoDiffVecXd DynamicsConstraintAutodiffVersion::getConstraintValueInAutoDiff(
//  const AutoDiffVecXd & x_i, const AutoDiffVecXd & tau_i,
//  const AutoDiffVecXd & x_iplus1, const AutoDiffVecXd & tau_iplus1,
//  const AutoDiffVecXd & h_i,
//  const VectorXd & theta_y, const VectorXd & theta_yddot) const {
//
//  // Get s and ds at knot i and i+1
//  AutoDiffVecXd s_i = initializeAutoDiff(VectorXd::Zero(n_y_));
//  AutoDiffVecXd ds_i = initializeAutoDiff(VectorXd::Zero(n_y_));
//  AutoDiffVecXd s_iplus1 = initializeAutoDiff(VectorXd::Zero(n_y_));
//  AutoDiffVecXd ds_iplus1 = initializeAutoDiff(VectorXd::Zero(n_y_));
//  getSAndSDotInAutoDiff(x_i, s_i, ds_i, 0, theta_y);
//  getSAndSDotInAutoDiff(x_iplus1, s_iplus1, ds_iplus1, n_q_ + n_v_ + n_tau_,
//                        theta_y);
//
//  // Get constraint value in autoDiff
//  if (is_head_) {
//    return 2 * (-3 * (s_i - s_iplus1) - h_i(0) * (ds_iplus1 + 2 * ds_i)) /
//           (h_i(0) * h_i(0)) -
//           dyn_expression_.getExpression(theta_yddot, s_i, ds_i, tau_i);
//  } else {
//    return (6 * (s_i - s_iplus1) + h_i(0) * (4 * ds_iplus1 + 2 * ds_i)) /
//           (h_i(0) * h_i(0)) -
//           dyn_expression_.getExpression(theta_yddot, s_iplus1, ds_iplus1, tau_iplus1);
//  }
//}
//
//
//void DynamicsConstraintAutodiffVersion::getSAndSDotInAutoDiff(AutoDiffVecXd x,
//    AutoDiffVecXd & s, AutoDiffVecXd & ds,
//    const int & i_start,
//    const VectorXd & theta_y) const {
//  AutoDiffVecXd q = x.head(n_q_);
//  AutoDiffVecXd v = x.tail(n_v_);
//  // s
//  s = kin_expression_.getExpression(theta_y, q);
//
//  // ds
//  MatrixXd d_phi_d_q =
//    autoDiffToGradientMatrix(
//      kin_expression_.getFeature(q)).block(0, i_start, n_feature_y_, n_q_);
//  VectorXd v_val = DiscardGradient(x.tail(n_v_));
//  VectorXd dphi0_dt = d_phi_d_q * v_val;
//
//  //TODO(yminchen): replace 2 * (n_q_ + n_v_ + n_tau_) + 1 with d_phi_d_q.cols() below
//
//  /////////////////// V2: forward differencing /////////////////////////////////
//  /*MatrixXd grad_dphidt = MatrixXd::Zero(n_feature_y_, 2 * (n_q_ + n_v_ + n_tau_) + 1);
//  for (int i = 0; i < n_q_ + n_v_; i++) {
//    x(i) += eps_fd_;
//
//    q = x.head(n_q_);
//    d_phi_d_q =
//      autoDiffToGradientMatrix(
//        kin_expression_.getFeature(q)).block(0, i_start, n_feature_y_, n_q_);
//    v_val = DiscardGradient(x.tail(n_v_));
//    VectorXd dphii_dt = d_phi_d_q * v_val;
//
//    x(i) -= eps_fd_;
//
//    grad_dphidt.col(i_start + i) = (dphii_dt - dphi0_dt) / eps_fd_;
//  }*/
//
//  /////////////////// V2: central differencing /////////////////////////////////
//  MatrixXd grad_dphidt = MatrixXd::Zero(n_feature_y_,
//                                        2 * (n_q_ + n_v_ + n_tau_) + 1);
//  vector<VectorXd> dphii_dt_vec(2, VectorXd::Zero(n_feature_y_));
//  for (int i = 0; i < n_q_ + n_v_; i++) {
//    for (unsigned int j = 0; j < cd_shift_vec_.size(); j++) {
//      x(i) += cd_shift_vec_[j];
//
//      // overwrite q, d_phi_d_q and v_val for computational speed
//      // (instead of declare variable)
//      q = x.head(n_q_);
//      d_phi_d_q =
//        autoDiffToGradientMatrix(
//          kin_expression_.getFeature(q)).block(0, i_start, n_feature_y_, n_q_);
//      v_val = DiscardGradient(x.tail(n_v_));
//      dphii_dt_vec[j] = d_phi_d_q * v_val;
//
//      x(i) -= cd_shift_vec_[j];
//    }
//
//    grad_dphidt.col(i_start + i) = (dphii_dt_vec[1] - dphii_dt_vec[0]) / eps_cd_;
//  }
//  //////////////////////////////////////////////////////////////////////////////
//
//  AutoDiffVecXd dphi_dt = initializeAutoDiff(dphi0_dt);
//  drake::math::initializeAutoDiffGivenGradientMatrix(
//    dphi0_dt, grad_dphidt, dphi_dt);
//
//  for (int i = 0; i < n_y_ ; i++) {
//    ds(i) = theta_y.segment(i * n_feature_y_, n_feature_y_).dot(dphi_dt);
//  }
//}
//
//
//void DynamicsConstraintAutodiffVersion::getSAndSDot(
//  const VectorXd & x,
//  VectorXd & s, VectorXd & ds) const {
//  // This is just for getting the double version of s and ds. (e.g. you want
//  // to record it, etc.)
//  // What we are doing here are:
//  // 1. initialize the autodiff yourself, so that it matches the format of the
//  //      autodiff version of getSAndSDot.
//  // 2. call the autodiff version of getSAndSDot
//  // 3. discard the gradient part
//
//  AutoDiffVecXd x_autoDiff = initializeAutoDiff(x);
//  AutoDiffVecXd s_autoDiff = initializeAutoDiff(VectorXd::Zero(n_y_));
//  AutoDiffVecXd ds_autoDiff = initializeAutoDiff(VectorXd::Zero(n_y_));
//  getSAndSDotInAutoDiff(x_autoDiff, s_autoDiff, ds_autoDiff, 0, theta_y_);
//
//  s = DiscardGradient(s_autoDiff);
//  ds = DiscardGradient(ds_autoDiff);
//}
//
//MatrixXd DynamicsConstraintAutodiffVersion::getGradientWrtTheta(
//  const VectorXd & x_i_double, const VectorXd & tau_i_double,
//  const VectorXd & x_iplus1_double, const VectorXd & tau_iplus1_double,
//  const VectorXd & h_i_double) const {
//  // It's a nonlinear function in theta, so we use autoDiff to get the gradient.
//  // The calculation here will not be the same as the one in eval(), because
//  // we have totally different autodiff, and the second autodiff requires
//  // costumization.
//  // Also, since they are nonilnear, each row might have different gradient.
//
//  // You'll need to create autoDiff yourself first, cause the input is double
//  // and you need to jacobian to get ds.
//
//  // ////////// V1: Do forward differencing wrt theta //////////////////////////
//  /*// Get x_i, x_iplus1 and h_i in autoDiff
//  VectorXd qvtqvth_double(2 * (n_q_ + n_v_ + n_tau_) + 1);
//  qvtqvth_double << x_i_double, tau_i_double,
//                    x_iplus1_double, tau_iplus1_double, h_i_double;
//  AutoDiffVecXd qvtqvth = initializeAutoDiff(qvtqvth_double);
//
//  AutoDiffVecXd x_i = qvtqvth.head(n_q_ + n_v_);
//  AutoDiffVecXd x_iplus1 = qvtqvth.segment(n_q_ + n_v_ + n_tau_, n_q_ + n_v_);
//  const AutoDiffVecXd h_i = qvtqvth.tail(1);
//
//  // Get the gradient wrt theta_y and theta_yddot
//  VectorXd theta(n_theta_y_ + n_theta_yddot_);
//  theta << theta_y_, theta_yddot_;
//  MatrixXd gradWrtTheta(n_y_, theta.size());
//  vector<VectorXd> y_vec;
//  for (int k = 0; k < theta.size(); k++) {
//    for (double shift : fd_shift_vec_) {
//      theta(k) += shift;
//
//      VectorXd theta_y = theta.head(n_theta_y_);
//      VectorXd theta_yddot = theta.tail(n_theta_yddot_);
//
//      // Evaluate constraint value
//      // y_vec.push_back(autoDiffToValueMatrix(getConstraintValueInAutoDiff(
//      //                                         x_i, x_iplus1, h_i,
//      //                                         theta_y, theta_yddot)));
//      y_vec.push_back(getConstraintValueInDouble(x_i, tau_i_double,
//                                              x_iplus1, tau_iplus1_double,
//                                              autoDiffToValueMatrix(h_i),
//                                              theta_y, theta_yddot));
//
//      theta(k) -= shift;
//    }
//
//    // Get gradient
//    gradWrtTheta.col(k) = (y_vec[1] - y_vec[0]) / eps_fd_;
//    y_vec.clear();
//  }*/
//
//  // ////////// V2: Do central differencing wrt theta //////////////////////////
//  // Get x_i, x_iplus1 and h_i in autoDiff
//  VectorXd qvtqvth_double(2 * (n_q_ + n_v_ + n_tau_) + 1);
//  qvtqvth_double << x_i_double, tau_i_double,
//                 x_iplus1_double, tau_iplus1_double, h_i_double;
//  AutoDiffVecXd qvtqvth = initializeAutoDiff(qvtqvth_double);
//
//  AutoDiffVecXd x_i = qvtqvth.head(n_q_ + n_v_);
//  AutoDiffVecXd x_iplus1 = qvtqvth.segment(n_q_ + n_v_ + n_tau_, n_q_ + n_v_);
//  const AutoDiffVecXd h_i = qvtqvth.tail(1);
//
//  // Get the gradient wrt theta_y and theta_yddot
//  VectorXd theta(n_theta_y_ + n_theta_yddot_);
//  theta << theta_y_, theta_yddot_;
//  MatrixXd gradWrtTheta(n_y_, theta.size());
//  vector<VectorXd> y_vec;
//  for (int k = 0; k < theta.size(); k++) {
//    for (double shift : cd_shift_vec_) {
//      theta(k) += shift;
//
//      VectorXd theta_y = theta.head(n_theta_y_);
//      VectorXd theta_yddot = theta.tail(n_theta_yddot_);
//
//      // Evaluate constraint value
//      // y_vec.push_back(autoDiffToValueMatrix(getConstraintValueInAutoDiff(
//      //                                         x_i, x_iplus1, h_i,
//      //                                         theta_y, theta_yddot)));
//      y_vec.push_back(getConstraintValueInDouble(
//                        x_i, tau_i_double,
//                        x_iplus1, tau_iplus1_double,
//                        autoDiffToValueMatrix(h_i),
//                        theta_y, theta_yddot));
//
//      theta(k) -= shift;
//    }
//
//    // Get gradient
//    gradWrtTheta.col(k) = (y_vec[1] - y_vec[0]) / eps_cd_;
//    y_vec.clear();
//  }
//
//  // /////////// V3: higher order method of finite difference //////////////////
//  // Reference: https://en.wikipedia.org/wiki/Numerical_differentiation#Higher-order_methods
//
//  /*// Get x_i, x_iplus1 and h_i in autoDiff
//  VectorXd qvtqvth_double(2 * (n_q_ + n_v_ + n_tau_) + 1);
//  qvtqvth_double << x_i_double, tau_i_double,
//                    x_iplus1_double, tau_iplus1_double, h_i_double;
//  AutoDiffVecXd qvtqvth = initializeAutoDiff(qvtqvth_double);
//
//  AutoDiffVecXd x_i = qvtqvth.head(n_q_ + n_v_);
//  AutoDiffVecXd x_iplus1 = qvtqvth.segment(n_q_ + n_v_ + n_tau_, n_q_ + n_v_);
//  const AutoDiffVecXd h_i = qvtqvth.tail(1);
//
//  // Get the gradient wrt theta_y and theta_yddot
//  VectorXd theta(n_theta_y_ + n_theta_yddot_);
//  theta << theta_y_, theta_yddot_;
//  MatrixXd gradWrtTheta(n_y_, theta.size());
//  vector<VectorXd> y_vec;
//  for (int k = 0; k < theta.size(); k++) {
//    for (double shift : ho_shift_vec_) {
//      theta(k) += shift;
//
//      VectorXd theta_y = theta.head(n_theta_y_);
//      VectorXd theta_yddot = theta.tail(n_theta_yddot_);
//
//      // Evaluate constraint value
//      // y_vec.push_back(autoDiffToValueMatrix(getConstraintValueInAutoDiff(
//      //                                         x_i, x_iplus1, h_i,
//      //                                         theta_y, theta_yddot)));
//      y_vec.push_back(getConstraintValueInDouble(x_i, tau_i_double,
//                                              x_iplus1, tau_iplus1_double,
//                                              autoDiffToValueMatrix(h_i),
//                                              theta_y, theta_yddot));
//
//      theta(k) -= shift;
//    }
//
//    // Get gradient
//    gradWrtTheta.col(k) =
//      (-y_vec[3] + 8 * y_vec[2] - 8 * y_vec[1] + y_vec[0]) / (3 * eps_ho_);
//    y_vec.clear();
//  }*/
//
//  //////////////////////////////////////////////////////////////////////////////
//
//  // Finding the optimal step size
//  // https://math.stackexchange.com/questions/815113/is-there-a-general-formula-for-estimating-the-step-size-h-in-numerical-different
//  // https://www.uio.no/studier/emner/matnat/math/MAT-INF1100/h10/kompendiet/kap11.pdf
//
//  return gradWrtTheta;
//}
//
//VectorXd DynamicsConstraintAutodiffVersion::computeTauToExtendModel(
//  const VectorXd & x_i_double, const VectorXd & x_iplus1_double,
//  const VectorXd & h_i, const VectorXd & theta_y_append) {
//
//  // Reset the model dimension, so that we can get the extended part of the model
//  int n_extend = theta_y_append.rows() / n_feature_y_;
//  kin_expression_.setModelDimension(n_extend);
//
//  // Get x_i, x_iplus1 in autoDiff (to get s and ds)
//  AutoDiffVecXd x_i = initializeAutoDiff(x_i_double);
//  AutoDiffVecXd x_iplus1 = initializeAutoDiff(x_iplus1_double);
//
//  // Get s and ds at knot i and i+1
//  VectorXd s_i = VectorXd::Zero(n_extend);
//  VectorXd ds_i = VectorXd::Zero(n_extend);
//  VectorXd s_iplus1 = VectorXd::Zero(n_extend);
//  VectorXd ds_iplus1 = VectorXd::Zero(n_extend);
//  getSAndSDotInDouble(x_i, s_i, ds_i, 0, theta_y_append);
//  getSAndSDotInDouble(x_iplus1, s_iplus1, ds_iplus1, 0, theta_y_append);
//
//  // Set the model dimension back just in case
//  kin_expression_.setModelDimension(n_y_);
//
//  // Get constraint value (without h(s,ds) and tau)
//  if (is_head_) {
//    return 2 * (-3 * (s_i - s_iplus1) - h_i(0) * (ds_iplus1 + 2 * ds_i)) /
//           (h_i(0) * h_i(0));
//  } else {
//    return (6 * (s_i - s_iplus1) + h_i(0) * (4 * ds_iplus1 + 2 * ds_i)) /
//           (h_i(0) * h_i(0));
//  }
//}
//
//
//
//
//
//
//
//// getConstraintValueInDouble() and getSAndSDotInDouble() are partially
//// duplicated from getConstraintValueInAutoDiff() and getSAndSDotInAutoDiff().
//// Despite the duplication, we still use these functions for computation speed.
//VectorXd DynamicsConstraintAutodiffVersion::getConstraintValueInDouble(
//  const AutoDiffVecXd & x_i, const VectorXd & tau_i,
//  const AutoDiffVecXd & x_iplus1, const VectorXd & tau_iplus1,
//  const VectorXd & h_i,
//  const VectorXd & theta_y, const VectorXd & theta_yddot) const {
//
//  // Get s and ds at knot i and i+1
//  VectorXd s_i = VectorXd::Zero(n_y_);
//  VectorXd ds_i = VectorXd::Zero(n_y_);
//  VectorXd s_iplus1 = VectorXd::Zero(n_y_);
//  VectorXd ds_iplus1 = VectorXd::Zero(n_y_);
//  getSAndSDotInDouble(x_i, s_i, ds_i, 0, theta_y);
//  getSAndSDotInDouble(x_iplus1, s_iplus1, ds_iplus1, n_q_ + n_v_ + n_tau_,
//                      theta_y);
//
//  // Get constraint value in autoDiff
//  if (is_head_) {
//    return 2 * (-3 * (s_i - s_iplus1) - h_i(0) * (ds_iplus1 + 2 * ds_i)) /
//           (h_i(0) * h_i(0)) -
//           dyn_expression_.getExpression(theta_yddot, s_i, ds_i, tau_i);
//  } else {
//    return (6 * (s_i - s_iplus1) + h_i(0) * (4 * ds_iplus1 + 2 * ds_i)) /
//           (h_i(0) * h_i(0)) -
//           dyn_expression_.getExpression(theta_yddot, s_iplus1, ds_iplus1, tau_iplus1);
//  }
//}
//void DynamicsConstraintAutodiffVersion::getSAndSDotInDouble(AutoDiffVecXd x,
//    VectorXd & s, VectorXd & ds,
//    const int & i_start,
//    const VectorXd & theta_y) const {
//  AutoDiffVecXd q = x.head(n_q_);
//  // s
//  // TODO(yminchen): Below could be sped up by using double kin_expression_.
//  s = autoDiffToValueMatrix(kin_expression_.getExpression(theta_y, q));
//  // s = kin_expression_double.getExpression(theta_y, autoDiffToValueMatrix(q));
//
//  // ds
//  MatrixXd d_phi_d_q =
//    autoDiffToGradientMatrix(
//      kin_expression_.getFeature(q)).block(0, i_start, n_feature_y_, n_q_);
//  VectorXd v_val = DiscardGradient(x.tail(n_v_));
//  VectorXd dphi0_dt = d_phi_d_q * v_val;
//
//  for (int i = 0; i < n_y_ ; i++) {
//    ds(i) = theta_y.segment(i * n_feature_y_, n_feature_y_).dot(dphi0_dt);
//  }
//}
//
//


}  // namespace find_models
}  // namespace goldilocks_models
}  // namespace dairlib
