#include "whittling_solver.h"
#include "include/_usr_include_eigen3/Eigen/Core"
#include <algorithm>
#include <iostream>

namespace dairlib::geometry {

using Eigen::MatrixXd;
using Eigen::Matrix2d;
using Eigen::Matrix3d;

using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::VectorXd;

namespace {

double cross(const Vector2d& v0, const Vector2d& v1) {
  return v0.x() * v1.y() - v0.y() * v1.x();
}

Eigen::Vector2d Rotate(const Vector2d& v, double a) {
  return v.x() * Vector2d(std::cos(a) , std::sin(a)) + v.y() * Vector2d(-std::sin(a), std::cos(a));
}

}

std::pair<Eigen::Vector2d, double> WhittlingSolver::SolveForBestCut(
    const Vector2d& interior_vertex, const MatrixXd& vertices,
    const Eigen::Vector2d& initial_guess) const {
  assert(vertices.rows() == 2);

  double beta = 0.8;
  double eps = 1e-8;

  Vector2d x = initial_guess.normalized();

  int count = 0;
  double dx = 10;
  while (dx > eps and count < 50) {
    dx = GetSearchDirection(x, interior_vertex, vertices);
    if (dx == 0) { break; } // can't improve current direction

    // line search
    double t = 0.1 / fabs(dx);
    double fx = SquaredHingeLoss(x, interior_vertex, vertices);
    while (SquaredHingeLoss(Rotate(x, t * dx), interior_vertex, vertices) > fx) {
      t *= beta;
      if (t < 1e-20) { break; }
    }
    x = Rotate(x, t * dx);
    x.normalize(); // normalizing x always decreases the cost
    ++count;
  }
//
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
      Vector2d c = vertices.col(i) - vertex;
      grad += 2 * c * c.transpose() * direction;
    }
  }
  return grad;
}

double WhittlingSolver::GetSearchDirection(
    const Eigen::Vector2d &direction, const Eigen::Vector2d &vertex,
    const Eigen::MatrixXd &vertices) const {

  Vector2d g = SquaredHingeLossGradient(direction, vertex, vertices);

  // (negative) component of the cost gradient in the tangent space of the
  // unit-norm constraint
  Vector2d step = -(g - g.dot(direction) * direction);

  // angle by which to rotate our candidate solution based on the gradient
  return cross(direction, step);

}

}