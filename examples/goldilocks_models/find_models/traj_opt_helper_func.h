#pragma once

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"

namespace dairlib {
namespace goldilocks_models {

// Do inverse kinematics to get configuration guess
std::vector<Eigen::VectorXd> GetCassieInitGuessForQ(
    int N, double stride_length, double ground_incline,
    const drake::multibody::MultibodyPlant<double>& plant);
// Get v by finite differencing q
std::vector<Eigen::VectorXd> GetCassieInitGuessForV(
    const std::vector<Eigen::VectorXd>& q_seed, double dt,
    const drake::multibody::MultibodyPlant<double>& plant);

}  // namespace goldilocks_models
}  // namespace dairlib
