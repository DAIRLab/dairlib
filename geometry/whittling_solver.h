#pragma once

#include <Eigen/Dense>

namespace dairlib::geometry {
class WhittlingSolver {
 public:
  WhittlingSolver() = default;

  std::pair<Eigen::Vector2d, double> SolveForBestCut(
      const Eigen::Vector2d &interior_vertex,
      const Eigen::MatrixXd &vertices) const;

 private:
  double SquaredHingeLoss(
      const Eigen::Vector2d &direction, const Eigen::Vector2d &vertex,
      const Eigen::MatrixXd &vertices) const;

  Eigen::Vector2d SquaredHingeLossGradient(
      const Eigen::Vector2d &direction, const Eigen::Vector2d &vertex,
      const Eigen::MatrixXd &vertices) const;

};
}