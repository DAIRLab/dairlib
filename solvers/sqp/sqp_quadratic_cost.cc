#include "sqp_quadratic_cost.h"

namespace dairlib::solvers::sqp {

using Eigen::MatrixXd;
using Eigen::VectorXd;


SqpQuadraticCost::SqpQuadraticCost(
    const MatrixXd &Q, const VectorXd &b, double c,
    const std::string &description) :
    NonlinearLeastSquaresCost<double>(Q.rows(), Q.rows(), Q, description),
    Q_(Q), b_(b), c_(c) {
  DRAKE_DEMAND(Q.rows() == Q.cols());
  DRAKE_DEMAND(Q.rows() == b.rows());
}

void SqpQuadraticCost::EvaluateInnerTerm(const Eigen::Ref<const VectorXd> &x,
                                         VectorXd *y) const {
  throw std::runtime_error("SqpQuadraticCost has no inner term");
}

void SqpQuadraticCost::EvaluateInnerTerm(const Eigen::Ref<const drake::AutoDiffVecXd> &x,
                                         drake::AutoDiffVecXd *y) const {
  throw std::runtime_error("SqpQuadraticCost has no inner term");
}

void SqpQuadraticCost::UpdateReference(const Eigen::VectorXd &y) {
  throw std::runtime_error("SqpQuadraticCost has no reference");
}

void SqpQuadraticCost::UpdateCoefficients(const Eigen::MatrixXd &Q,
                                          const Eigen::VectorXd &b,
                                          double c) {
  DRAKE_DEMAND(Q.rows() == Q_.rows());
  DRAKE_DEMAND(Q.cols() == Q_.cols());
  DRAKE_DEMAND(Q.rows() == b.rows());
  Q_ = Q;
  b_ = b;
  c_ = c;
}

void SqpQuadraticCost::EvaluateCost(const Eigen::Ref<const Eigen::VectorXd> &x,
                                    Eigen::VectorXd *y) const {
  y->resize(1);
  *y = .5 * x.transpose() * Q_ * x + b_.transpose() * x;
  (*y)(0) += c_;
}

GaussNewtonApproximation SqpQuadraticCost::CalcGaussNewtonApproximation(
    const Eigen::Ref<const Eigen::VectorXd> &x) const {
  return GaussNewtonApproximation {
    Q_,
    Q_ * x + b_,
    0.5 * x.dot(Q_ * x) + b_.dot(x) + c_
  };
}


}