#include "whittling_solver.h"
#include <algorithm>

namespace dairlib::geometry {

using Eigen::MatrixXd;
using Eigen::Matrix2d;
using Eigen::Matrix3d;

using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::VectorXd;

namespace {
VectorXd centroid (const MatrixXd& verts) {
  const int n = verts.cols();
  const int m = verts.rows();
  double sum_weight = 0;
  VectorXd center = VectorXd::Zero(m);
  for(int i = 0; i < n; i++) {
    int idx_prev = (i - 1 < 0) ? n - 1 : i - 1;
    int idx_next = (i + 1) % n;
    double w = (verts.col(i) - verts.col(idx_prev)).norm() +
        (verts.col(i) - verts.col(idx_next)).norm();
    center += w * verts.col(i);
    sum_weight += w;
  }
  return (1.0 / sum_weight) * center;
}

double cross(const Vector2d& v0, const Vector2d& v1) {
  return v0.x() * v1.y() - v0.y() * v1.x();
}

}

std::pair<Eigen::Vector2d, double> WhittlingSolver::SolveForBestCut(
    const Vector2d& interior_vertex, const Vector2d& direction,
    const MatrixXd& vertices) const {
  assert(vertices.rows() == 2);

  double alpha = 0.5;
  double beta = 0.8;
  double eps = 1e-4;

  Vector2d initial_guess = interior_vertex - centroid(vertices);
  Vector2d x = initial_guess.normalized();

  Vector2d g = SquaredHingeLossGradient(x, interior_vertex, vertices);

  while (abs(cross(x, g)) > eps) {
    Vector2d dx = GetNewtonDirection(x, interior_vertex, vertices);

    // line search
    double t = 1;
    double fx = SquaredHingeLoss(x, interior_vertex, vertices);
    while (SquaredHingeLoss(x + t * dx, interior_vertex, vertices) > fx + alpha * t * g.transpose() * dx) {
      t *= beta;
    }
    x = x + t * dx;
    x.normalize(); // normalizing x always decreases the cost
  }
  return {x, x.dot(interior_vertex)};
}

double WhittlingSolver::SquaredHingeLoss(
    const Vector2d& direction, const Vector2d& vertex,
    const MatrixXd& vertices) const {
  double b = direction.dot(vertex);
  double cost  = 0;
  for (int i = 0; i < vertices.cols(); ++i) {
    double s = std::max(0.0, direction.dot(vertices.col(i)) - b);
    cost += s*s;
  }
  return cost;
}

Eigen::Vector2d WhittlingSolver::SquaredHingeLossGradient(
    const Vector2d& direction, const Vector2d& vertex,
    const MatrixXd& vertices) const {
  Vector2d grad = Vector2d::Zero();
  double b = direction.dot(vertex);
  for (int i = 0; i < vertices.cols(); ++i) {
    if (direction.dot(vertices.col(i)) > b) {
      Vector2d c = vertex - vertices.col(i);
      grad += 2 * c * c.transpose() * direction - 2 * c;
    }
  }
  return grad;
}

Eigen::Matrix2d WhittlingSolver::SquaredHingeLossHessian(
    const Vector2d& direction, const Vector2d& vertex,
    const MatrixXd& vertices) const {
  Matrix2d hess = Matrix2d::Zero();
  double b = direction.dot(vertex);
  for (int i = 0; i < vertices.cols(); ++i) {
    if (direction.dot(vertices.col(i)) > b) {
      Vector2d c = vertex - vertices.col(i);
      hess += 2 * c * c.transpose();
    }
  }
  return hess;
}

Eigen::Vector2d WhittlingSolver::GetNewtonDirection(
    const Vector2d &direction, const Vector2d &vertex,
    const MatrixXd &vertices) const {

  Matrix3d kkt_mat = Matrix3d::Zero();
  kkt_mat.topLeftCorner<2,2>() = SquaredHingeLossHessian(direction, vertex, vertices);

  // Constraint jacobian of unit norm constraint:
  // d/dx x^Tx - 1 = 2x
  kkt_mat.topRightCorner<2,1>() = 2 * direction;
  kkt_mat.bottomLeftCorner<1, 2>() = 2 * direction.transpose();

  Vector3d kkt_rhs = Vector3d::Zero();
  kkt_rhs.head<2>() = -SquaredHingeLossGradient(direction, vertex, vertices);
  kkt_rhs(2) = 1 - direction.squaredNorm();

  Eigen::LDLT<Matrix3d> solver;
  solver.compute(kkt_mat);
  return solver.solve(kkt_rhs).head<2>();
}

}