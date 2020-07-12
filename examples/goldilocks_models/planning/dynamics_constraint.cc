#include "examples/goldilocks_models/planning/dynamics_constraint.h"

namespace dairlib {
namespace goldilocks_models {
namespace planning {

using std::isinf;
using std::isnan;
using std::list;
using std::make_shared;
using std::make_unique;
using std::map;
using std::string;
using std::unique_ptr;
using std::vector;

using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::MatrixX;
using drake::VectorX;
using drake::math::autoDiffToGradientMatrix;
using drake::math::autoDiffToValueMatrix;
using drake::math::DiscardGradient;
using drake::math::initializeAutoDiff;
using drake::multibody::MultibodyPlant;
using drake::solvers::Binding;
using drake::solvers::Constraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::to_string;
using drake::solvers::VariableRefList;
using drake::solvers::VectorXDecisionVariable;
using drake::symbolic::Expression;
using drake::symbolic::Variable;
using Eigen::AutoDiffScalar;
using Eigen::Dynamic;
using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

DynamicsConstraint::DynamicsConstraint(const ReducedOrderModel& rom,
                                       const std::string& description)
    : NonlinearConstraint<double>(2 * rom.n_y(),
                                  2 * (2 * rom.n_y() + rom.n_tau()) + 1,
                                  VectorXd::Zero(2 * rom.n_y()),
                                  VectorXd::Zero(2 * rom.n_y()), description),
      rom_(rom),
      n_y_(rom.n_y()),
      n_z_(2 * rom.n_y()),
      n_tau_(rom.n_tau()) {}

void DynamicsConstraint::EvaluateConstraint(
    const Eigen::Ref<const drake::VectorX<double>>& ztzth,
    drake::VectorX<double>* y) const {
  // Extract elements
  VectorX<double> z_i = ztzth.head(n_z_);
  VectorX<double> tau_i = ztzth.segment(n_z_, n_tau_);
  VectorX<double> z_iplus1 = ztzth.segment(n_z_ + n_tau_, n_z_);
  VectorX<double> tau_iplus1 = ztzth.segment(2 * (n_z_) + n_tau_, n_tau_);
  VectorX<double> h_i = ztzth.tail(1);

  // Evaluate derivatives at knot points
  VectorX<double> g_i = g(z_i, tau_i);
  VectorX<double> g_iplus1 = g(z_iplus1, tau_iplus1);

  // Value of the cubic spline at the collocation point
  VectorX<double> z_c = (z_i + z_iplus1) / 2 + (g_i - g_iplus1) * h_i(0) / 8;
  VectorX<double> tau_c = (tau_i + tau_iplus1) / 2;

  // Assign dynamics constraint value
  *y = (z_iplus1 - z_i) / h_i(0) - (g_i + 4 * g(z_c, tau_c) + g_iplus1) / 6;
}

VectorX<double> DynamicsConstraint::g(const VectorX<double>& z,
                                      const VectorX<double>& tau) const {
  VectorX<double> zdot(2 * n_y_);
  zdot << z.tail(n_y_), rom_.EvalDynamicFunc(z.head(n_y_), z.tail(n_y_), tau);
  return zdot;
}

}  // namespace planning
}  // namespace goldilocks_models
}  // namespace dairlib
