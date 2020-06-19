#include "solvers/constraint_factory.h"

namespace dairlib {
namespace solvers {

using drake::solvers::LorentzConeConstraint;
using drake::solvers::LinearConstraint;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

std::shared_ptr<LorentzConeConstraint> CreateConicFrictionConstraint(double mu,
    int normal_index) {
  DRAKE_DEMAND(normal_index <= 2);
  DRAKE_DEMAND(normal_index >= 0);
  Matrix3d A = Matrix3d::Zero();
  A(0, normal_index) = mu;
  A(1, (normal_index + 1) % 3) = 1;
  A(2, (normal_index + 2) % 3) = 1;
  Vector3d b = Vector3d::Zero();
  return std::make_shared<LorentzConeConstraint>(A, b);
}

std::shared_ptr<LinearConstraint> CreateLinearFrictionConstraint(double mu,
    int num_faces, int normal_index, bool inscribed) {
  DRAKE_DEMAND(normal_index <= 2);
  DRAKE_DEMAND(normal_index >= 0);
  DRAKE_DEMAND(num_faces >= 3);

  double mu_lin = inscribed ? (mu * cos(M_PI/num_faces)) : mu;

  // Each face is fx cos(theta) + fy sin(theta) <= mu_lin * fz
  MatrixXd A(num_faces, 3);
  for (int i = 0; i < num_faces; i++) {
    double theta = i * 2 * M_PI / num_faces;
    A(i, normal_index) = mu_lin;
    A(i, (normal_index + 1) % 3) = -cos(theta);
    A(i, (normal_index + 2) % 3) = -sin(theta);
  }
  VectorXd lb = VectorXd::Zero(num_faces);
  VectorXd ub = VectorXd::Constant(num_faces, 
      std::numeric_limits<double>::infinity());
  return std::make_shared<LinearConstraint>(A, lb, ub);
}

}  // namespace solvers
}  // namespace dairlib
