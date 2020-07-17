#pragma once

#include <string>
#include <Eigen/Dense>
#include "systems/trajectory_optimization/hybrid_dircon.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"
#include "multibody/multibody_utils.h"

namespace dairlib {
namespace goldilocks_models {
namespace misc {

void runSampleQp(std::string directory, std::string prefix);

}  // namespace misc
}  // namespace goldilocks_models
}  // namespace dairlib
