#include "examples/goldilocks_models/find_models/dynamics_constraint.h"

#include "multibody/multibody_utils.h"

namespace dairlib {
namespace goldilocks_models {
namespace find_models {


DynamicsConstraint::DynamicsConstraint(
  int n_s, int n_feature_s,
  const VectorXd & theta_s,
  int n_sDDot, int n_feature_sDDot,
  const VectorXd & theta_sDDot,
  int n_tau,
  MatrixXd B_tau,
  const MultibodyPlant<AutoDiffXd> * plant,
  const MultibodyPlant<double> * plant_double,
  vector<double> var_scale,
  double tau_scale,
  bool is_head,
  int robot_option,
  const std::string& description) : DirconAbstractConstraint<double>(n_sDDot,
        2 * (plant->num_positions() + plant->num_velocities() + n_tau) + 1,
        VectorXd::Zero(n_sDDot),
        VectorXd::Zero(n_sDDot),
        description),
  plant_double_(plant_double),
  n_q_(plant->num_positions()),
  n_v_(plant->num_velocities()),
  n_u_(plant->num_actuators()),
  n_s_(n_s),
  n_feature_s_(n_feature_s),
  n_theta_s_(theta_s.size()),
  theta_s_(theta_s),
  n_sDDot_(n_sDDot),
  n_feature_sDDot_(n_feature_sDDot),
  n_theta_sDDot_(theta_sDDot.size()),
  theta_sDDot_(theta_sDDot),
  n_tau_(n_tau),
  kin_expression_(KinematicsExpression<AutoDiffXd>(n_s, n_feature_s,
                  plant, robot_option)),
  kin_expression_double_(KinematicsExpression<double>(n_s, n_feature_s,
                         plant_double, robot_option)),
  dyn_expression_(DynamicsExpression(n_sDDot, n_feature_sDDot, B_tau,
                                     robot_option)),
  is_head_(is_head),
  quaternion_scale_(var_scale[4]),
  omega_scale_(var_scale[0]),
  tau_scale_(tau_scale) {

  // Check the theta size
  DRAKE_DEMAND(n_s * n_feature_s == theta_s.size());

  // Check the feature size implemented in the model expression
  AutoDiffVecXd q_temp =
    initializeAutoDiff(VectorXd::Ones(plant->num_positions()));
  DRAKE_DEMAND(n_feature_s == kin_expression_.getFeature(q_temp).size());

  // Check the theta size
  DRAKE_DEMAND(n_sDDot * n_feature_sDDot == theta_sDDot.size());

  // Check the feature size implemented in the model expression
  VectorXd s_temp = VectorXd::Zero(n_sDDot);
  VectorXd ds_temp = VectorXd::Zero(n_sDDot);
  DRAKE_DEMAND(n_feature_sDDot ==
               dyn_expression_.getFeature(s_temp, ds_temp).size());
}

void DynamicsConstraint::EvaluateConstraint(
  const Eigen::Ref<const drake::VectorX<double>>& x,
  drake::VectorX<double>* y) const {

  // Extract elements
  VectorXd x_i = x.head(n_q_ + n_v_);
  VectorXd tau_i = x.segment(n_q_ + n_v_, n_tau_);
  VectorXd x_iplus1 = x.segment(n_q_ + n_v_ + n_tau_, n_q_ + n_v_);
  VectorXd tau_iplus1 = x.segment(2 * (n_q_ + n_v_) + n_tau_, n_tau_);
  const VectorXd h_i = x.tail(1);

  double model_scale = 10;
  *y = getConstraintValueInDouble(x_i, tau_i,
                                  x_iplus1, tau_iplus1,
                                  h_i,
                                  theta_s_, theta_sDDot_) / model_scale;
}

VectorXd DynamicsConstraint::getConstraintValueInDouble(
  const VectorXd & x_i, const VectorXd & tau_i,
  const VectorXd & x_iplus1, const VectorXd & tau_iplus1,
  const VectorXd & h_i,
  const VectorXd & theta_s, const VectorXd & theta_sDDot) const {

  // Get s and ds at knot i and i+1
  VectorXd s_i = VectorXd::Zero(n_s_);
  VectorXd ds_i = VectorXd::Zero(n_s_);
  VectorXd s_iplus1 = VectorXd::Zero(n_s_);
  VectorXd ds_iplus1 = VectorXd::Zero(n_s_);
  getSAndSDotInDouble(x_i, s_i, ds_i, theta_s);
  getSAndSDotInDouble(x_iplus1, s_iplus1, ds_iplus1, theta_s);

  VectorXd tau_i_scaled = tau_scale_ * tau_i;
  VectorXd tau_iplus1_scaled = tau_scale_ * tau_iplus1;

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
           dyn_expression_.getExpression(theta_sDDot, s_i, ds_i, tau_i_scaled);
  } else {
    return (6 * (s_i - s_iplus1) + h_i(0) * (4 * ds_iplus1 + 2 * ds_i)) /
           (h_i(0) * h_i(0)) -
           dyn_expression_.getExpression(theta_sDDot, s_iplus1, ds_iplus1, tau_iplus1_scaled);
  }
}

// Careful: need to initialize the size of s and ds before calling getSAndSDotInDouble
void DynamicsConstraint::getSAndSDotInDouble(VectorXd x,
    VectorXd & s, VectorXd & ds,
    const VectorXd & theta_s) const {
  // Scale to real-life scale
  VectorXd state(n_q_ + n_v_);
  state << x.segment(0, 4) * quaternion_scale_,
        x.segment(4, n_q_ - 4),
        x.segment(n_q_, n_v_) * omega_scale_;

  VectorXd q = state.head(n_q_);
  VectorXd v = state.tail(n_v_);

  VectorXd qdot(n_q_);
  VectorXd u = VectorXd::Zero(n_u_);
  auto context = multibody::createContext(*plant_double_, state, u);
  plant_double_->MapVelocityToQDot(*context, v, &qdot);

  // 1. s
  s = kin_expression_double_.getExpression(theta_s, q);

  // 2. ds
  // get gradient of feature wrt q =============================================
  /// Forward differencing
  // MatrixXd d_phi_d_q = MatrixXd(n_feature_s_, q.size());
  // VectorXd phi_0(n_feature_s_);
  // VectorXd phi_1(n_feature_s_);
  // phi_0 = kin_expression_double_.getFeature(q);
  // for (int i = 0; i < q.size(); i++) {
  //   q(i) += eps_fd_feature_;
  //   phi_1 = kin_expression_double_.getFeature(q);
  //   q(i) -= eps_fd_feature_;
  //   d_phi_d_q.col(i) = (phi_1 - phi_0) / eps_fd_feature_;
  // }

  /// Central differencing
  // MatrixXd d_phi_d_q = MatrixXd(n_feature_s_, q.size());
  // VectorXd phi_0(n_feature_s_);
  // VectorXd phi_1(n_feature_s_);
  // for (int i = 0; i < q.size(); i++) {
  //   q(i) -= eps_cd_feature_/2;
  //   phi_0 = kin_expression_double_.getFeature(q);
  //   q(i) += eps_cd_feature_;
  //   phi_1 = kin_expression_double_.getFeature(q);
  //   q(i) -= eps_cd_feature_/2;
  //   d_phi_d_q.col(i) = (phi_1 - phi_0) / eps_cd_feature_;
  // }

  /// high order of finite differencing
  MatrixXd d_phi_d_q = MatrixXd(n_feature_s_, q.size());
  VectorXd phi_0(n_feature_s_);
  VectorXd phi_1(n_feature_s_);
  VectorXd phi_2(n_feature_s_);
  VectorXd phi_3(n_feature_s_);
  for (int i = 0; i < q.size(); i++) {
    q(i) -= eps_ho_feature_ / 2;
    phi_0 = kin_expression_double_.getFeature(q);
    q(i) += eps_ho_feature_ / 4;
    phi_1 = kin_expression_double_.getFeature(q);
    q(i) += eps_ho_feature_ / 2;
    phi_2 = kin_expression_double_.getFeature(q);
    q(i) += eps_ho_feature_ / 4;
    phi_3 = kin_expression_double_.getFeature(q);
    q(i) -= eps_ho_feature_ / 2;
    d_phi_d_q.col(i) = (-phi_3 + 8 * phi_2 - 8 * phi_1 + phi_0) /
                       (3 * eps_ho_feature_);
  }

  /// ground truth (testing the accuracy of gradients)
  // AutoDiffVecXd x_autodiff = initializeAutoDiff(x);
  // AutoDiffVecXd q_autodiff = x_autodiff.head(n_q_);
  // MatrixXd d_phi_d_q_GROUNDTRUTH =
  //   autoDiffToGradientMatrix(
  //     kin_expression_.getFeature(q_autodiff)).block(0, 0, n_feature_s_, n_q_);
  // cout << "gradient difference = " << (d_phi_d_q_GROUNDTRUTH - d_phi_d_q).norm()
  //      << endl;
  // End of getting gradient ===================================================

  VectorXd dphi0_dt = d_phi_d_q * qdot;
  for (int i = 0; i < n_s_ ; i++) {
    ds(i) = theta_s.segment(i * n_feature_s_, n_feature_s_).dot(dphi0_dt);
  }
}

void DynamicsConstraint::getSAndSDot(const VectorXd & x,
                                     VectorXd & s, VectorXd & ds) const {
  getSAndSDotInDouble(x, s, ds, theta_s_);
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

  VectorXd tau_i_scaled = tau_scale_ * tau_i;
  VectorXd tau_iplus1_scaled = tau_scale_ * tau_iplus1;

  // ////////// V1: Do forward differencing wrt theta //////////////////////////
  /*
  // Get the gradient wrt theta_s and theta_sDDot
  VectorXd theta(n_theta_s_ + n_theta_sDDot_);
  theta << theta_s_, theta_sDDot_;
  MatrixXd gradWrtTheta(n_s_, theta.size());
  vector<VectorXd> y_vec;
  for (int k = 0; k < theta.size(); k++) {
    for (double shift : fd_shift_vec_) {
      theta(k) += shift;

      VectorXd theta_s = theta.head(n_theta_s_);
      VectorXd theta_sDDot = theta.tail(n_theta_sDDot_);

      // Evaluate constraint value
      // y_vec.push_back(autoDiffToValueMatrix(getConstraintValueInAutoDiff(
      //                                         x_i, x_iplus1, h_i,
      //                                         theta_s, theta_sDDot)));
      y_vec.push_back(getConstraintValueInDouble(
                        x_i_double, tau_i_scaled,
                        x_iplus1_double, tau_iplus1_scaled,
                        h_i_double,
                        theta_s, theta_sDDot));

      theta(k) -= shift;
    }

    // Get gradient
    gradWrtTheta.col(k) = (y_vec[1] - y_vec[0]) / eps_fd_;
    y_vec.clear();
  }*/

  // ////////// V2: Do central differencing wrt theta //////////////////////////
  // Get the gradient wrt theta_s and theta_sDDot
  VectorXd theta(n_theta_s_ + n_theta_sDDot_);
  theta << theta_s_, theta_sDDot_;
  MatrixXd gradWrtTheta(n_s_, theta.size());
  vector<VectorXd> y_vec;
  for (int k = 0; k < theta.size(); k++) {
    for (double shift : cd_shift_vec_) {
      theta(k) += shift;

      VectorXd theta_s = theta.head(n_theta_s_);
      VectorXd theta_sDDot = theta.tail(n_theta_sDDot_);

      // Evaluate constraint value
      // y_vec.push_back(autoDiffToValueMatrix(getConstraintValueInAutoDiff(
      //                                         x_i, x_iplus1, h_i,
      //                                         theta_s, theta_sDDot)));
      y_vec.push_back(getConstraintValueInDouble(
                        x_i_double, tau_i_scaled,
                        x_iplus1_double, tau_iplus1_scaled,
                        h_i_double,
                        theta_s, theta_sDDot));

      theta(k) -= shift;
    }

    // Get gradient
    gradWrtTheta.col(k) = (y_vec[1] - y_vec[0]) / eps_cd_;
    y_vec.clear();
  }

  // /////////// V3: higher order method of finite difference //////////////////
  // Reference: https://en.wikipedia.org/wiki/Numerical_differentiation#Higher-order_methods

  /*
  // Get the gradient wrt theta_s and theta_sDDot
  VectorXd theta(n_theta_s_ + n_theta_sDDot_);
  theta << theta_s_, theta_sDDot_;
  MatrixXd gradWrtTheta(n_s_, theta.size());
  vector<VectorXd> y_vec;
  for (int k = 0; k < theta.size(); k++) {
    for (double shift : ho_shift_vec_) {
      theta(k) += shift;

      VectorXd theta_s = theta.head(n_theta_s_);
      VectorXd theta_sDDot = theta.tail(n_theta_sDDot_);

      // Evaluate constraint value
      // y_vec.push_back(autoDiffToValueMatrix(getConstraintValueInAutoDiff(
      //                                         x_i, x_iplus1, h_i,
      //                                         theta_s, theta_sDDot)));
      y_vec.push_back(getConstraintValueInDouble(
                        x_i_double, tau_i_scaled,
                        x_iplus1_double, tau_iplus1_scaled,
                        h_i_double,
                        theta_s, theta_sDDot));

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

VectorXd DynamicsConstraint::computeTauToExtendModel(
  const VectorXd & x_i_double, const VectorXd & x_iplus1_double,
  const VectorXd & h_i, const VectorXd & theta_s_append) {

  // Reset the model dimension, so that we can get the extended part of the model
  int n_extend = theta_s_append.rows() / n_feature_s_;
  kin_expression_.setModelDimension(n_extend);

  // Get s and ds at knot i and i+1
  VectorXd s_i = VectorXd::Zero(n_extend);
  VectorXd ds_i = VectorXd::Zero(n_extend);
  VectorXd s_iplus1 = VectorXd::Zero(n_extend);
  VectorXd ds_iplus1 = VectorXd::Zero(n_extend);
  getSAndSDotInDouble(x_i_double, s_i, ds_i, theta_s_append);
  getSAndSDotInDouble(x_iplus1_double, s_iplus1, ds_iplus1, theta_s_append);

  // Set the model dimension back just in case
  kin_expression_.setModelDimension(n_s_);

  // Get constraint value (without h(s,ds) and tau)
  if (is_head_) {
    return 2 * (-3 * (s_i - s_iplus1) - h_i(0) * (ds_iplus1 + 2 * ds_i)) /
           (h_i(0) * h_i(0)) / tau_scale_;
  } else {
    return (6 * (s_i - s_iplus1) + h_i(0) * (4 * ds_iplus1 + 2 * ds_i)) /
           (h_i(0) * h_i(0)) / tau_scale_;
  }
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

DynamicsConstraintAutodiffVersion::DynamicsConstraintAutodiffVersion(
  int n_s, int n_feature_s,
  const VectorXd & theta_s,
  int n_sDDot, int n_feature_sDDot,
  const VectorXd & theta_sDDot,
  int n_tau,
  MatrixXd B_tau,
  const MultibodyPlant<AutoDiffXd> * plant,
  bool is_head,
  int robot_option,
  const std::string& description):
  Constraint(n_sDDot,
             2 * (plant->num_positions() + plant->num_velocities() + n_tau) + 1,
             VectorXd::Zero(n_sDDot),
             VectorXd::Zero(n_sDDot),
             description),
  plant_(plant),
  n_q_(plant->num_positions()),
  n_v_(plant->num_velocities()),
  n_s_(n_s),
  n_feature_s_(n_feature_s),
  n_theta_s_(theta_s.size()),
  theta_s_(theta_s),
  n_sDDot_(n_sDDot),
  n_feature_sDDot_(n_feature_sDDot),
  n_theta_sDDot_(theta_sDDot.size()),
  theta_sDDot_(theta_sDDot),
  n_tau_(n_tau),
  kin_expression_(KinematicsExpression<AutoDiffXd>(n_s, n_feature_s, plant,
                  robot_option)),
  dyn_expression_(DynamicsExpression(n_sDDot, n_feature_sDDot, B_tau,
                                     robot_option)),
  is_head_(is_head) {

  // Check the theta size
  DRAKE_DEMAND(n_s * n_feature_s == theta_s.size());

  // Check the feature size implemented in the model expression
  AutoDiffVecXd q_temp =
    initializeAutoDiff(VectorXd::Zero(plant->num_positions()));
  DRAKE_DEMAND(n_feature_s == kin_expression_.getFeature(q_temp).size());

  // Check the theta size
  DRAKE_DEMAND(n_sDDot * n_feature_sDDot == theta_sDDot.size());

  // Check the feature size implemented in the model expression
  VectorXd s_temp = VectorXd::Zero(n_sDDot);
  VectorXd ds_temp = VectorXd::Zero(n_sDDot);
  DRAKE_DEMAND(n_feature_sDDot ==
               dyn_expression_.getFeature(s_temp, ds_temp).size());
}


void DynamicsConstraintAutodiffVersion::DoEval(const
    Eigen::Ref<const Eigen::VectorXd>& q,
    Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  Eval(initializeAutoDiff(q), &y_t);
  *y = autoDiffToValueMatrix(y_t);
}

void DynamicsConstraintAutodiffVersion::DoEval(const
    Eigen::Ref<const AutoDiffVecXd>& qvtqvth,
    AutoDiffVecXd* y) const {
  // Extract elements
  AutoDiffVecXd x_i = qvtqvth.head(n_q_ + n_v_);
  AutoDiffVecXd tau_i = qvtqvth.segment(n_q_ + n_v_, n_tau_);
  AutoDiffVecXd x_iplus1 = qvtqvth.segment(n_q_ + n_v_ + n_tau_, n_q_ + n_v_);
  AutoDiffVecXd tau_iplus1 = qvtqvth.segment(2 * (n_q_ + n_v_) + n_tau_, n_tau_);
  const AutoDiffVecXd h_i = qvtqvth.tail(1);

  // Impose dynamics constraint
  // Way 1 /////////////////////////////////////////////////////////////////////
  *y = getConstraintValueInAutoDiff(x_i, tau_i, x_iplus1, tau_iplus1, h_i,
                                    theta_s_, theta_sDDot_);


  // Way 2 /////////////////////////////////////////////////////////////////////
  // (not up-to-date)
  // rhs is copied from DynamicsExpression.getExpression().
  /*// Get s and ds at knot i and i+1
  AutoDiffVecXd s_i = initializeAutoDiff(VectorXd::Zero(n_s_));
  AutoDiffVecXd ds_i = initializeAutoDiff(VectorXd::Zero(n_s_));
  AutoDiffVecXd s_iplus1 = initializeAutoDiff(VectorXd::Zero(n_s_));
  AutoDiffVecXd ds_iplus1 = initializeAutoDiff(VectorXd::Zero(n_s_));
  getSAndSDotInAutoDiff(x_i, s_i, ds_i, 0, theta_s_);
  getSAndSDotInAutoDiff(x_iplus1, s_iplus1, ds_iplus1, n_q_ + n_v_ + n_tau_, theta_s_);

  // Get constraint value in autoDiff
  if (is_head_) {
    AutoDiffVecXd lhs =
      2 * (-3 * (s_i - s_iplus1) - h_i(0) * (ds_iplus1 + 2 * ds_i)) /
      (h_i(0) * h_i(0));

    // AutoDiffVecXd rhs =
    //   dyn_expression_.getExpression(theta_sDDot_, s_i, ds_i, tau_i);
    AutoDiffVecXd rhs = initializeAutoDiff(VectorXd::Zero(n_sDDot_));
    for (int i = 0; i < n_sDDot_; i++)
      rhs(i) = theta_sDDot_.segment(i * n_feature_sDDot_, n_feature_sDDot_).dot(
                 dyn_expression_.getFeature(s_i, ds_i));

    *y = lhs - rhs;
  }
  else {
    AutoDiffVecXd lhs =
      (6 * (s_i - s_iplus1) + h_i(0) * (4 * ds_iplus1 + 2 * ds_i)) /
      (h_i(0) * h_i(0));

    // AutoDiffVecXd rhs =
    //   dyn_expression_.getExpression(theta_sDDot_, s_iplus1, ds_iplus1, tau_iplus1);
    AutoDiffVecXd rhs = initializeAutoDiff(VectorXd::Zero(n_sDDot_));
    for (int i = 0; i < n_sDDot_; i++)
      rhs(i) = theta_sDDot_.segment(i * n_feature_sDDot_, n_feature_sDDot_).dot(
                 dyn_expression_.getFeature(s_iplus1, ds_iplus1));

    *y = lhs - rhs;
  }*/
  //////////////////////////////////////////////////////////////////////////////
}

void DynamicsConstraintAutodiffVersion::DoEval(const
    Eigen::Ref<const VectorX<Variable>>& x,
    VectorX<Expression>*y) const {
  throw std::logic_error(
    "This constraint class does not support symbolic evaluation.");
}


AutoDiffVecXd DynamicsConstraintAutodiffVersion::getConstraintValueInAutoDiff(
  const AutoDiffVecXd & x_i, const AutoDiffVecXd & tau_i,
  const AutoDiffVecXd & x_iplus1, const AutoDiffVecXd & tau_iplus1,
  const AutoDiffVecXd & h_i,
  const VectorXd & theta_s, const VectorXd & theta_sDDot) const {

  // Get s and ds at knot i and i+1
  AutoDiffVecXd s_i = initializeAutoDiff(VectorXd::Zero(n_s_));
  AutoDiffVecXd ds_i = initializeAutoDiff(VectorXd::Zero(n_s_));
  AutoDiffVecXd s_iplus1 = initializeAutoDiff(VectorXd::Zero(n_s_));
  AutoDiffVecXd ds_iplus1 = initializeAutoDiff(VectorXd::Zero(n_s_));
  getSAndSDotInAutoDiff(x_i, s_i, ds_i, 0, theta_s);
  getSAndSDotInAutoDiff(x_iplus1, s_iplus1, ds_iplus1, n_q_ + n_v_ + n_tau_,
                        theta_s);

  // Get constraint value in autoDiff
  if (is_head_) {
    return 2 * (-3 * (s_i - s_iplus1) - h_i(0) * (ds_iplus1 + 2 * ds_i)) /
           (h_i(0) * h_i(0)) -
           dyn_expression_.getExpression(theta_sDDot, s_i, ds_i, tau_i);
  } else {
    return (6 * (s_i - s_iplus1) + h_i(0) * (4 * ds_iplus1 + 2 * ds_i)) /
           (h_i(0) * h_i(0)) -
           dyn_expression_.getExpression(theta_sDDot, s_iplus1, ds_iplus1, tau_iplus1);
  }
}


void DynamicsConstraintAutodiffVersion::getSAndSDotInAutoDiff(AutoDiffVecXd x,
    AutoDiffVecXd & s, AutoDiffVecXd & ds,
    const int & i_start,
    const VectorXd & theta_s) const {
  AutoDiffVecXd q = x.head(n_q_);
  AutoDiffVecXd v = x.tail(n_v_);
  // s
  s = kin_expression_.getExpression(theta_s, q);

  // ds
  MatrixXd d_phi_d_q =
    autoDiffToGradientMatrix(
      kin_expression_.getFeature(q)).block(0, i_start, n_feature_s_, n_q_);
  VectorXd v_val = DiscardGradient(x.tail(n_v_));
  VectorXd dphi0_dt = d_phi_d_q * v_val;

  //TODO(yminchen): replace 2 * (n_q_ + n_v_ + n_tau_) + 1 with d_phi_d_q.cols() below

  /////////////////// V2: forward differencing /////////////////////////////////
  /*MatrixXd grad_dphidt = MatrixXd::Zero(n_feature_s_, 2 * (n_q_ + n_v_ + n_tau_) + 1);
  for (int i = 0; i < n_q_ + n_v_; i++) {
    x(i) += eps_fd_;

    q = x.head(n_q_);
    d_phi_d_q =
      autoDiffToGradientMatrix(
        kin_expression_.getFeature(q)).block(0, i_start, n_feature_s_, n_q_);
    v_val = DiscardGradient(x.tail(n_v_));
    VectorXd dphii_dt = d_phi_d_q * v_val;

    x(i) -= eps_fd_;

    grad_dphidt.col(i_start + i) = (dphii_dt - dphi0_dt) / eps_fd_;
  }*/

  /////////////////// V2: central differencing /////////////////////////////////
  MatrixXd grad_dphidt = MatrixXd::Zero(n_feature_s_,
                                        2 * (n_q_ + n_v_ + n_tau_) + 1);
  vector<VectorXd> dphii_dt_vec(2, VectorXd::Zero(n_feature_s_));
  for (int i = 0; i < n_q_ + n_v_; i++) {
    for (unsigned int j = 0; j < cd_shift_vec_.size(); j++) {
      x(i) += cd_shift_vec_[j];

      // overwrite q, d_phi_d_q and v_val for computational speed
      // (instead of declare variable)
      q = x.head(n_q_);
      d_phi_d_q =
        autoDiffToGradientMatrix(
          kin_expression_.getFeature(q)).block(0, i_start, n_feature_s_, n_q_);
      v_val = DiscardGradient(x.tail(n_v_));
      dphii_dt_vec[j] = d_phi_d_q * v_val;

      x(i) -= cd_shift_vec_[j];
    }

    grad_dphidt.col(i_start + i) = (dphii_dt_vec[1] - dphii_dt_vec[0]) / eps_cd_;
  }
  //////////////////////////////////////////////////////////////////////////////

  AutoDiffVecXd dphi_dt = initializeAutoDiff(dphi0_dt);
  drake::math::initializeAutoDiffGivenGradientMatrix(
    dphi0_dt, grad_dphidt, dphi_dt);

  for (int i = 0; i < n_s_ ; i++) {
    ds(i) = theta_s.segment(i * n_feature_s_, n_feature_s_).dot(dphi_dt);
  }
}


void DynamicsConstraintAutodiffVersion::getSAndSDot(
  const VectorXd & x,
  VectorXd & s, VectorXd & ds) const {
  // This is just for getting the double version of s and ds. (e.g. you want
  // to record it, etc.)
  // What we are doing here are:
  // 1. initialize the autodiff yourself, so that it matches the format of the
  //      autodiff version of getSAndSDot.
  // 2. call the autodiff version of getSAndSDot
  // 3. discard the gradient part

  AutoDiffVecXd x_autoDiff = initializeAutoDiff(x);
  AutoDiffVecXd s_autoDiff = initializeAutoDiff(VectorXd::Zero(n_s_));
  AutoDiffVecXd ds_autoDiff = initializeAutoDiff(VectorXd::Zero(n_s_));
  getSAndSDotInAutoDiff(x_autoDiff, s_autoDiff, ds_autoDiff, 0, theta_s_);

  s = DiscardGradient(s_autoDiff);
  ds = DiscardGradient(ds_autoDiff);
}

MatrixXd DynamicsConstraintAutodiffVersion::getGradientWrtTheta(
  const VectorXd & x_i_double, const VectorXd & tau_i_double,
  const VectorXd & x_iplus1_double, const VectorXd & tau_iplus1_double,
  const VectorXd & h_i_double) const {
  // It's a nonlinear function in theta, so we use autoDiff to get the gradient.
  // The calculation here will not be the same as the one in eval(), because
  // we have totally different autodiff, and the second autodiff requires
  // costumization.
  // Also, since they are nonilnear, each row might have different gradient.

  // You'll need to create autoDiff yourself first, cause the input is double
  // and you need to jacobian to get ds.

  // ////////// V1: Do forward differencing wrt theta //////////////////////////
  /*// Get x_i, x_iplus1 and h_i in autoDiff
  VectorXd qvtqvth_double(2 * (n_q_ + n_v_ + n_tau_) + 1);
  qvtqvth_double << x_i_double, tau_i_double,
                    x_iplus1_double, tau_iplus1_double, h_i_double;
  AutoDiffVecXd qvtqvth = initializeAutoDiff(qvtqvth_double);

  AutoDiffVecXd x_i = qvtqvth.head(n_q_ + n_v_);
  AutoDiffVecXd x_iplus1 = qvtqvth.segment(n_q_ + n_v_ + n_tau_, n_q_ + n_v_);
  const AutoDiffVecXd h_i = qvtqvth.tail(1);

  // Get the gradient wrt theta_s and theta_sDDot
  VectorXd theta(n_theta_s_ + n_theta_sDDot_);
  theta << theta_s_, theta_sDDot_;
  MatrixXd gradWrtTheta(n_s_, theta.size());
  vector<VectorXd> y_vec;
  for (int k = 0; k < theta.size(); k++) {
    for (double shift : fd_shift_vec_) {
      theta(k) += shift;

      VectorXd theta_s = theta.head(n_theta_s_);
      VectorXd theta_sDDot = theta.tail(n_theta_sDDot_);

      // Evaluate constraint value
      // y_vec.push_back(autoDiffToValueMatrix(getConstraintValueInAutoDiff(
      //                                         x_i, x_iplus1, h_i,
      //                                         theta_s, theta_sDDot)));
      y_vec.push_back(getConstraintValueInDouble(x_i, tau_i_double,
                                              x_iplus1, tau_iplus1_double,
                                              autoDiffToValueMatrix(h_i),
                                              theta_s, theta_sDDot));

      theta(k) -= shift;
    }

    // Get gradient
    gradWrtTheta.col(k) = (y_vec[1] - y_vec[0]) / eps_fd_;
    y_vec.clear();
  }*/

  // ////////// V2: Do central differencing wrt theta //////////////////////////
  // Get x_i, x_iplus1 and h_i in autoDiff
  VectorXd qvtqvth_double(2 * (n_q_ + n_v_ + n_tau_) + 1);
  qvtqvth_double << x_i_double, tau_i_double,
                 x_iplus1_double, tau_iplus1_double, h_i_double;
  AutoDiffVecXd qvtqvth = initializeAutoDiff(qvtqvth_double);

  AutoDiffVecXd x_i = qvtqvth.head(n_q_ + n_v_);
  AutoDiffVecXd x_iplus1 = qvtqvth.segment(n_q_ + n_v_ + n_tau_, n_q_ + n_v_);
  const AutoDiffVecXd h_i = qvtqvth.tail(1);

  // Get the gradient wrt theta_s and theta_sDDot
  VectorXd theta(n_theta_s_ + n_theta_sDDot_);
  theta << theta_s_, theta_sDDot_;
  MatrixXd gradWrtTheta(n_s_, theta.size());
  vector<VectorXd> y_vec;
  for (int k = 0; k < theta.size(); k++) {
    for (double shift : cd_shift_vec_) {
      theta(k) += shift;

      VectorXd theta_s = theta.head(n_theta_s_);
      VectorXd theta_sDDot = theta.tail(n_theta_sDDot_);

      // Evaluate constraint value
      // y_vec.push_back(autoDiffToValueMatrix(getConstraintValueInAutoDiff(
      //                                         x_i, x_iplus1, h_i,
      //                                         theta_s, theta_sDDot)));
      y_vec.push_back(getConstraintValueInDouble(
                        x_i, tau_i_double,
                        x_iplus1, tau_iplus1_double,
                        autoDiffToValueMatrix(h_i),
                        theta_s, theta_sDDot));

      theta(k) -= shift;
    }

    // Get gradient
    gradWrtTheta.col(k) = (y_vec[1] - y_vec[0]) / eps_cd_;
    y_vec.clear();
  }

  // /////////// V3: higher order method of finite difference //////////////////
  // Reference: https://en.wikipedia.org/wiki/Numerical_differentiation#Higher-order_methods

  /*// Get x_i, x_iplus1 and h_i in autoDiff
  VectorXd qvtqvth_double(2 * (n_q_ + n_v_ + n_tau_) + 1);
  qvtqvth_double << x_i_double, tau_i_double,
                    x_iplus1_double, tau_iplus1_double, h_i_double;
  AutoDiffVecXd qvtqvth = initializeAutoDiff(qvtqvth_double);

  AutoDiffVecXd x_i = qvtqvth.head(n_q_ + n_v_);
  AutoDiffVecXd x_iplus1 = qvtqvth.segment(n_q_ + n_v_ + n_tau_, n_q_ + n_v_);
  const AutoDiffVecXd h_i = qvtqvth.tail(1);

  // Get the gradient wrt theta_s and theta_sDDot
  VectorXd theta(n_theta_s_ + n_theta_sDDot_);
  theta << theta_s_, theta_sDDot_;
  MatrixXd gradWrtTheta(n_s_, theta.size());
  vector<VectorXd> y_vec;
  for (int k = 0; k < theta.size(); k++) {
    for (double shift : ho_shift_vec_) {
      theta(k) += shift;

      VectorXd theta_s = theta.head(n_theta_s_);
      VectorXd theta_sDDot = theta.tail(n_theta_sDDot_);

      // Evaluate constraint value
      // y_vec.push_back(autoDiffToValueMatrix(getConstraintValueInAutoDiff(
      //                                         x_i, x_iplus1, h_i,
      //                                         theta_s, theta_sDDot)));
      y_vec.push_back(getConstraintValueInDouble(x_i, tau_i_double,
                                              x_iplus1, tau_iplus1_double,
                                              autoDiffToValueMatrix(h_i),
                                              theta_s, theta_sDDot));

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

VectorXd DynamicsConstraintAutodiffVersion::computeTauToExtendModel(
  const VectorXd & x_i_double, const VectorXd & x_iplus1_double,
  const VectorXd & h_i, const VectorXd & theta_s_append) {

  // Reset the model dimension, so that we can get the extended part of the model
  int n_extend = theta_s_append.rows() / n_feature_s_;
  kin_expression_.setModelDimension(n_extend);

  // Get x_i, x_iplus1 in autoDiff (to get s and ds)
  AutoDiffVecXd x_i = initializeAutoDiff(x_i_double);
  AutoDiffVecXd x_iplus1 = initializeAutoDiff(x_iplus1_double);

  // Get s and ds at knot i and i+1
  VectorXd s_i = VectorXd::Zero(n_extend);
  VectorXd ds_i = VectorXd::Zero(n_extend);
  VectorXd s_iplus1 = VectorXd::Zero(n_extend);
  VectorXd ds_iplus1 = VectorXd::Zero(n_extend);
  getSAndSDotInDouble(x_i, s_i, ds_i, 0, theta_s_append);
  getSAndSDotInDouble(x_iplus1, s_iplus1, ds_iplus1, 0, theta_s_append);

  // Set the model dimension back just in case
  kin_expression_.setModelDimension(n_s_);

  // Get constraint value (without h(s,ds) and tau)
  if (is_head_) {
    return 2 * (-3 * (s_i - s_iplus1) - h_i(0) * (ds_iplus1 + 2 * ds_i)) /
           (h_i(0) * h_i(0));
  } else {
    return (6 * (s_i - s_iplus1) + h_i(0) * (4 * ds_iplus1 + 2 * ds_i)) /
           (h_i(0) * h_i(0));
  }
}







// getConstraintValueInDouble() and getSAndSDotInDouble() are partially
// duplicated from getConstraintValueInAutoDiff() and getSAndSDotInAutoDiff().
// Despite the duplication, we still use these functions for computation speed.
VectorXd DynamicsConstraintAutodiffVersion::getConstraintValueInDouble(
  const AutoDiffVecXd & x_i, const VectorXd & tau_i,
  const AutoDiffVecXd & x_iplus1, const VectorXd & tau_iplus1,
  const VectorXd & h_i,
  const VectorXd & theta_s, const VectorXd & theta_sDDot) const {

  // Get s and ds at knot i and i+1
  VectorXd s_i = VectorXd::Zero(n_s_);
  VectorXd ds_i = VectorXd::Zero(n_s_);
  VectorXd s_iplus1 = VectorXd::Zero(n_s_);
  VectorXd ds_iplus1 = VectorXd::Zero(n_s_);
  getSAndSDotInDouble(x_i, s_i, ds_i, 0, theta_s);
  getSAndSDotInDouble(x_iplus1, s_iplus1, ds_iplus1, n_q_ + n_v_ + n_tau_,
                      theta_s);

  // Get constraint value in autoDiff
  if (is_head_) {
    return 2 * (-3 * (s_i - s_iplus1) - h_i(0) * (ds_iplus1 + 2 * ds_i)) /
           (h_i(0) * h_i(0)) -
           dyn_expression_.getExpression(theta_sDDot, s_i, ds_i, tau_i);
  } else {
    return (6 * (s_i - s_iplus1) + h_i(0) * (4 * ds_iplus1 + 2 * ds_i)) /
           (h_i(0) * h_i(0)) -
           dyn_expression_.getExpression(theta_sDDot, s_iplus1, ds_iplus1, tau_iplus1);
  }
}
void DynamicsConstraintAutodiffVersion::getSAndSDotInDouble(AutoDiffVecXd x,
    VectorXd & s, VectorXd & ds,
    const int & i_start,
    const VectorXd & theta_s) const {
  AutoDiffVecXd q = x.head(n_q_);
  // s
  // TODO(yminchen): Below could be sped up by using double kin_expression_.
  s = autoDiffToValueMatrix(kin_expression_.getExpression(theta_s, q));
  // s = kin_expression_double.getExpression(theta_s, autoDiffToValueMatrix(q));

  // ds
  MatrixXd d_phi_d_q =
    autoDiffToGradientMatrix(
      kin_expression_.getFeature(q)).block(0, i_start, n_feature_s_, n_q_);
  VectorXd v_val = DiscardGradient(x.tail(n_v_));
  VectorXd dphi0_dt = d_phi_d_q * v_val;

  for (int i = 0; i < n_s_ ; i++) {
    ds(i) = theta_s.segment(i * n_feature_s_, n_feature_s_).dot(dphi0_dt);
  }
}




}  // namespace find_models
}  // namespace goldilocks_models
}  // namespace dairlib
