#include "whittling_solver.h"

namespace dairlib::geometry {

using Eigen::MatrixXd;
using Eigen::Vector2d;

std::pair<Eigen::Vector2d, double> WhittlingSolver::SolveForBestCut(
    const Vector2d& interior_vertex, const MatrixXd& vertices) const {
  return { Vector2d::Zero(), 0 };
}

}