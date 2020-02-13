#include "examples/goldilocks_models/dynamics_expression.h"


namespace dairlib {
namespace goldilocks_models {

DynamicsExpression::DynamicsExpression(int n_sDDot, int n_feature_sDDot,
                                       int robot_option) {
  n_feature_sDDot_ = n_feature_sDDot;
  n_sDDot_ = n_sDDot;
  robot_option_ = robot_option;
}
DynamicsExpression::DynamicsExpression(int n_sDDot, int n_feature_sDDot,
                                       MatrixXd B_tau, int robot_option) {
  n_feature_sDDot_ = n_feature_sDDot;
  n_sDDot_ = n_sDDot;
  B_tau_ = B_tau;
  robot_option_ = robot_option;
}

int DynamicsExpression::getDimFeature() {
  return n_feature_sDDot_;
}

template <typename U, typename V>
V DynamicsExpression::getExpression(const U & theta,
                                    const V & s, const V & ds,
                                    const V & tau) const {
  // DRAKE_DEMAND(n_sDDot_ * n_feature_sDDot_ == theta.size());  // check theta size
  // DRAKE_DEMAND(n_feature_sDDot_ == getFeature(s).size());  // check feature size

  V expression(n_sDDot_);
  for (int i = 0; i < n_sDDot_; i++)
    expression(i) = theta.segment(i * n_feature_sDDot_,
                                  n_feature_sDDot_).dot(getFeature(s, ds));
  expression += B_tau_ * tau;
  // cout << "B_tau_ = " << B_tau_ << endl;
  // cout << "tau = " << tau.transpose() << endl;
  // cout << "expression = " << expression << endl;

  return expression;
}

template <typename T>
T DynamicsExpression::getFeature(const T & s, const T & ds) const {

  // Implement your choice of features below
  // Be careful that the dimension should match with n_feature_sDDot_
  // TODO(yminchen): find a way to avoid hard coding the features here

  // Version 1: for dynamics_expression_test
  // T feature(5);
  // feature << s(0),
  //            s(1)*s(1)*s(1),
  //            s(0) * s(1),
  //            cos(s(0)),
  //            sqrt(s(1));

  // Version 2: testing
  // T feature(1);
  // feature << 0;

  // Version 3: testing
  // T feature(1);
  // feature << s(0);

  // Version 4: testing
  // T feature(2);
  // feature << s(0), s(1);

  // Version 5: ns = 2, all combinations until quadratic
  // DRAKE_DEMAND(n_sDDot_ == 2);
  // T feature(21);
  // feature << 1,     // constant
  //            s(0),  // linear
  //            s(1),
  //            ds(0),
  //            ds(1),
  //            s(0) * s(0),  // quadratic
  //            s(1) * s(0),
  //            ds(0) * s(0),
  //            ds(1) * s(0),
  //            s(0) * s(1),
  //            s(1) * s(1),
  //            ds(0) * s(1),
  //            ds(1) * s(1),
  //            s(0) * ds(0),
  //            s(1) * ds(0),
  //            ds(0) * ds(0),
  //            ds(1) * ds(0),
  //            s(0) * ds(1),
  //            s(1) * ds(1),
  //            ds(0) * ds(1),
  //            ds(1) * ds(1);

  // Version 6: ns = 1, all combinations until quadratic
  if (n_sDDot_ == 1) {
    // DRAKE_DEMAND(n_sDDot_ == 1);
    T feature(6);
    feature << 1,     // constant
            s(0),  // linear
            ds(0),
            s(0) * s(0),  // quadratic
            ds(0) * s(0),
            ds(0) * ds(0);
    return feature;
  }

  // Version 7: testing (debug B matrix)
  /*DRAKE_DEMAND(n_sDDot_ == 1);
  T feature(1);
  feature << s(0)*s(0)*s(0);*/

  // return feature;

  // Version 8: ns = 2, all combinations until quadratic
  /*if (n_sDDot_ == 2) {
    // DRAKE_DEMAND(n_sDDot_ == 2);
    T feature(15);  // 1 + 4 + (4Choose2 + 4) = 1 + 4 + 10 = 15
    feature << 1,   // constant
            s(0),
            s(1),
            ds(0),
            ds(1),  // linear
            s(0) * s(0),
            s(0) * s(1),
            s(0) * ds(0),
            s(0) * ds(1),
            s(1) * s(1),
            s(1) * ds(0),
            s(1) * ds(1),
            ds(0) * ds(0),
            ds(0) * ds(1),
            ds(1) * ds(1);  // quadratic
    return feature;
  }*/

  // Version 9: ns = 3, all combinations until quadratic
  /*if (n_sDDot_ == 3) {
    // DRAKE_DEMAND(n_sDDot_ == 3);
    T feature(28);  // 1 + 6 + (6Choose2 + 6) = 1 + 6 + 21 = 28
    feature << 1,  // constant
            s(0),
            s(1),
            s(2),
            ds(0),
            ds(1),
            ds(2),  // linear
            s(0) * s(0),
            s(0) * s(1),
            s(0) * s(2),
            s(0) * ds(0),
            s(0) * ds(1),
            s(0) * ds(2),
            s(1) * s(1),
            s(1) * s(2),
            s(1) * ds(0),
            s(1) * ds(1),
            s(1) * ds(2),
            s(2) * s(2),
            s(2) * ds(0),
            s(2) * ds(1),
            s(2) * ds(2),
            ds(0) * ds(0),
            ds(0) * ds(1),
            ds(0) * ds(2),
            ds(1) * ds(1),
            ds(1) * ds(2),
            ds(2) * ds(2);  // quadratic
    return feature;
  }*/

  // Version 10: ns = 2, 2D LIPM with all quadratic combination
  if (n_sDDot_ == 2) {
    // DRAKE_DEMAND(n_sDDot_ == 2);
    T feature(16);  // 1 + 1 + 4 + (4Choose2 + 4) = 1 + 1 + 4 + 10 = 16
    T first_element(1);
    if (s(1) == 0) {
      first_element << (9.80665 / (s(1) + 1e-8))*s(0); // avoid sigularity
    } else {
      first_element << (9.80665 / s(1))*s(0);
    }
    feature << first_element(0),
            1,   // constant
            s(0),
            s(1),
            ds(0),
            ds(1),  // linear
            s(0) * s(0),
            s(0) * s(1),
            s(0) * ds(0),
            s(0) * ds(1),
            s(1) * s(1),
            s(1) * ds(0),
            s(1) * ds(1),
            ds(0) * ds(0),
            ds(0) * ds(1),
            ds(1) * ds(1);  // quadratic
    return feature;
  }

  // Version 11: ns = 4, 2D LIPM with swing foot, with all quadratic combination
  if (n_sDDot_ == 4) {
    // DRAKE_DEMAND(n_sDDot_ == 4);
    T feature(46);  // 1 + 1 + 8 + (8Choose2 + 8) = 1 + 1 + 8 + 36 = 46
    T first_element(1);
    if (s(1) == 0) {
      cout << "here\n";
      first_element << (9.80665 / (s(1) + 1e-8))*s(0); // avoid sigularity
    }
    else
      first_element << (9.80665 / s(1))*s(0);
    feature << first_element(0),
            1,   // constant
            s(0),
            s(1),
            s(2),
            s(3),
            ds(0),
            ds(1),
            ds(2),
            ds(3),  // linear
            s(0) * s(0),
            s(0) * s(1),
            s(0) * s(2),
            s(0) * s(3),
            s(0) * ds(0),
            s(0) * ds(1),
            s(0) * ds(2),
            s(0) * ds(3),
            s(1) * s(1),
            s(1) * s(2),
            s(1) * s(3),
            s(1) * ds(0),
            s(1) * ds(1),
            s(1) * ds(2),
            s(1) * ds(3),
            s(2) * s(2),
            s(2) * s(3),
            s(2) * ds(0),
            s(2) * ds(1),
            s(2) * ds(2),
            s(2) * ds(3),
            s(3) * s(3),
            s(3) * ds(0),
            s(3) * ds(1),
            s(3) * ds(2),
            s(3) * ds(3),
            ds(0) * ds(0),
            ds(0) * ds(1),
            ds(0) * ds(2),
            ds(0) * ds(3),
            ds(1) * ds(1),
            ds(1) * ds(2),
            ds(1) * ds(3),
            ds(2) * ds(2),
            ds(2) * ds(3),
            ds(3) * ds(3);  // quadratic

    return feature;
  }

  // if (robot_option_ == 0) {
  // } else if (robot_option_ == 1) {

  // }

  DRAKE_DEMAND(false);  // shouldn't reach to this line of code
}


// Instantiation
// TODO(yminchen): is there a way to implement getExpression() that returns
// VectorX<double>?
template VectorX<double> DynamicsExpression::getExpression(
  const VectorX<double> &, const VectorX<double> &,
  const VectorX<double> &, const VectorX<double> &) const;
template VectorX<AutoDiffXd> DynamicsExpression::getExpression(
  const VectorX<double> &, const VectorX<AutoDiffXd> &,
  const VectorX<AutoDiffXd> &, const VectorX<AutoDiffXd> &) const;
// template VectorX<AutoDiffXd> DynamicsExpression::getExpression(
//   const VectorX<AutoDiffXd> &, const VectorX<double> &, const VectorX<double> &) const;
// template VectorX<AutoDiffXd> DynamicsExpression::getExpression(
//   const VectorX<AutoDiffXd> &, const VectorX<AutoDiffXd> &, const VectorX<AutoDiffXd> &) const;

template VectorX<double> DynamicsExpression::getFeature(
  const VectorX<double> &,
  const VectorX<double> &) const;
template VectorX<AutoDiffXd> DynamicsExpression::getFeature(
  const VectorX<AutoDiffXd> &,
  const VectorX<AutoDiffXd> &) const;

}  // namespace goldilocks_models
}  // namespace dairlib

