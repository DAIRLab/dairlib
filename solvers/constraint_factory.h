#pragma once

#include "drake/solvers/constraint.h"

namespace dairlib {
namespace solvers {

/// Utility method to create a Lorentz cone constraint for friction.
/// Constraint is mu * lambda_n >= ||lambda_t||.
/// @param mu the coefficient of friction.
/// @param normal_index the index (0,1, or 2) into the force vector for the
///   normal force. Default = 2.
std::shared_ptr<drake::solvers::LorentzConeConstraint>
    CreateConicFrictionConstraint(double mu, int normal_index = 2);

/// Utility method to create a linear cone constraint for friction.
/// Constraint is a polytopic approximation of mu * lambda_n >= ||lambda_t||.
///   Creates n different linear inequalities, each of the form
///     fx cos(theta) + fy sin(theta) <= mu_lin * fz
///   where theta starts from 0 and goes in 2pi/num_faces increments.
/// The linearized cone can either be inscribed (conservative) or circumscribed.
/// In the case of the inscribed cone, mu_ln = mu / cos(pi / num_faces).
/// @param mu the coefficient of friction.
/// @param num_faces the number of linear inequalities. Must be at least 3.
///   default = 8.
/// @param normal_index the index (0,1, or 2) into the force vector for the
///   normal force. Default = 2.
/// @param inscribed Whether the approximation is inscribed or circumscribed.
///   Default = true.
std::shared_ptr<drake::solvers::LinearConstraint>
    CreateLinearFrictionConstraint(double mu, int num_faces = 8,
    int normal_index = 2, bool inscribed = true);

}  // namespace solvers
}  // namespace dairlib
