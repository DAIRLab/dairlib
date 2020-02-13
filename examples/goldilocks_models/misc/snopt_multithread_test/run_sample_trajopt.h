#pragma once

#include <string>
#include <Eigen/Dense>
#include "systems/trajectory_optimization/hybrid_dircon.h"

#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/solve.h"

#include "multibody/multibody_utils.h"

using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::SolutionResult;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

using drake::multibody::MultibodyPlant;
using drake::AutoDiffXd;

namespace dairlib {
namespace goldilocks_models  {
namespace misc {

void runSampleTrajopt(
    /*const MultibodyPlant<double> & plant,
    const MultibodyPlant<AutoDiffXd> & plant_autoDiff,*/
    double stride_length, double ground_incline,
    std::string directory, std::string init_file, std::string prefix);

}  // namespace misc
}  // namespace goldilocks_models
}  // namespace dairlib
