#include "examples/goldilocks_models/planning/dynamics_constraint.h"

namespace dairlib {
namespace goldilocks_models {
namespace planning {

using std::cout;
using std::endl;
using std::isinf;
using std::isnan;
using std::list;
using std::make_shared;
using std::make_unique;
using std::map;
using std::string;
using std::unique_ptr;
using std::vector;

using drake::MatrixX;
using drake::VectorX;
using drake::multibody::MultibodyPlant;
using drake::solvers::Binding;
using drake::solvers::Constraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::to_string;
using drake::solvers::VariableRefList;
using drake::solvers::VectorXDecisionVariable;
using drake::symbolic::Expression;
using drake::symbolic::Variable;
using Eigen::Dynamic;
using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

DynamicsConstraint::DynamicsConstraint(
    const ReducedOrderModel& rom, const std::set<int>& idx_constant_rom_vel,
    const std::string& description)
    : NonlinearConstraint<double>(2 * rom.n_y(),
                                  2 * (2 * rom.n_y() + rom.n_tau()) + 1,
                                  VectorXd::Zero(2 * rom.n_y()),
                                  VectorXd::Zero(2 * rom.n_y()), description),
      rom_(rom),
      n_y_(rom.n_y()),
      n_z_(2 * rom.n_y()),
      n_tau_(rom.n_tau()),
      idx_constant_rom_vel_(idx_constant_rom_vel) {}

void DynamicsConstraint::EvaluateConstraint(
    const Eigen::Ref<const drake::VectorX<double>>& ztzth,
    drake::VectorX<double>* y) const {
  // Extract elements
  const VectorX<double>& z_i = ztzth.head(n_z_);
  const VectorX<double>& tau_i = ztzth.segment(n_z_, n_tau_);
  const VectorX<double>& z_iplus1 = ztzth.segment(n_z_ + n_tau_, n_z_);
  const VectorX<double>& tau_iplus1 =
      ztzth.segment(2 * (n_z_) + n_tau_, n_tau_);
  const VectorX<double>& h_i = ztzth.tail(1);

  // Evaluate derivatives at knot points
  VectorX<double> g_i = g(z_i, tau_i);
  VectorX<double> g_iplus1 = g(z_iplus1, tau_iplus1);

  // Value of the cubic spline at the collocation point
  VectorX<double> z_c = (z_i + z_iplus1) / 2 + (g_i - g_iplus1) * h_i(0) / 8;
  VectorX<double> tau_c = (tau_i + tau_iplus1) / 2;

  // Assign dynamics constraint value
  *y = (z_iplus1 - z_i) / h_i(0) - (g_i + 4 * g(z_c, tau_c) + g_iplus1) / 6;

  /*if (this->get_description() == "rom_dyn_1_0") {
    cout << "z_iplus1 = " << z_iplus1 << endl;
    cout << "z_i = " << z_i << endl;
    cout << "h_i = " << h_i << endl;
    cout << "g_i = " << g_i << endl;
    cout << "z_c = " << z_c << endl;
    cout << "tau_c = " << tau_c << endl;
    cout << "g_iplus1 = " << g_iplus1 << endl;
    cout << "g(z_c, tau_c) = " << g(z_c, tau_c) << endl;
  }*/
}

VectorX<double> DynamicsConstraint::g(const VectorX<double>& z,
                                      const VectorX<double>& tau) const {
  VectorX<double> zdot(2 * n_y_);
  zdot << z.tail(n_y_), rom_.EvalDynamicFunc(z.head(n_y_), z.tail(n_y_), tau);

  // Set some acceleration to 0 (heuristic)
  for (const auto& idx : idx_constant_rom_vel_) {
    //    zdot(idx) = 0;
    zdot(idx) = -2 * zdot(idx - n_y_);  // heuristic
  }

  return zdot;
}

}  // namespace planning
}  // namespace goldilocks_models
}  // namespace dairlib
