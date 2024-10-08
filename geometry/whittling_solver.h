#pragma once

#include <Eigen/Dense>

namespace dairlib::geometry {

/*!
 * Gradient-based solver to find (locally) optimal cuts for thw whittling
 * algorithm. Minimizes a squared hinge loss on vertices excluded from the
 * polygon subject to unit-norm constraints on the halfspace normal
 */
class WhittlingSolver {
 public:
  WhittlingSolver() = default;
  std::pair<Eigen::Vector2d, double> SolveForBestCut(
      const Eigen::Vector2d &interior_vertex,
      const Eigen::MatrixXd &vertices,
      const Eigen::Vector2d& initial_guess) const;

 private:
  double SquaredHingeLoss(
      const Eigen::Vector2d &direction, const Eigen::Vector2d &vertex,
      const Eigen::MatrixXd &vertices) const;

  Eigen::Vector2d SquaredHingeLossGradient(
      const Eigen::Vector2d &direction, const Eigen::Vector2d &vertex,
      const Eigen::MatrixXd &vertices) const;

  double GetSearchDirection(const Eigen::Vector2d &direction,
                            const Eigen::Vector2d &vertex,
                            const Eigen::MatrixXd &vertices) const;

};
}