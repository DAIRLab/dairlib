#include "set_membership_constraint.h"

namespace dairlib::solvers {

using Eigen::VectorXd;

SetMembershipConstraint::SetMembershipConstraint(
    int num_constraints, int num_vars, const VectorXd &lb, const VectorXd &ub) :
    NonlinearConstraint<double>(num_constraints, num_vars, lb, ub) {}

}