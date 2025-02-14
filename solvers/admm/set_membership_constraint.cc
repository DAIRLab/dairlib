#include "set_membership_constraint.h"

namespace dairlib::solvers {

using Eigen::VectorXd;

SetMembershipConstraint::SetMembershipConstraint(
    int num_constraints, int num_vars, const VectorXd &lb, const VectorXd &ub) :
    NonlinearConstraint<double>(num_constraints, num_vars, lb, ub) {
  shift_ = VectorXd::Zero(num_vars);
}

void SetMembershipConstraint::EvaluateConstraint(
    const Eigen::Ref<const drake::VectorX<double>> &x,
    drake::VectorX<double> *y) const {
  throw std::runtime_error(
      "SetMembershipConstraint and it's derived classes "
      "aren't meant to be directly used within general purpose solvers."
  );
}

}