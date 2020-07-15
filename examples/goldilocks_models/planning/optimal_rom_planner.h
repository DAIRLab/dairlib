#pragma once

#include <thread>

#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/leaf_system.h"

#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

namespace dairlib::goldilocks_models {

/// RomMPC formulates a trajectory optimization problem of a given reduced order
/// model, and solves it on a new thread.
class RomMPC {
 public:
  RomMPC(const drake::multibody::MultibodyPlant<double>& plant);

  // Open a new thread and solve if there is no MPC currently being solved
  void StartSolveIfNotBusy(const drake::VectorX<double>& init_guess);

  // Read solution
  drake::VectorX<double> ReadSolution();

  bool has_solution() const { return has_solution_; };

 private:
  void Solve(int test, int test2);

  // std::thread::join() is a blocking function, so we use is_solved_ to know
  // whether trajopt is solved and we can call join() without waiting
  bool is_solved_;
  // is_active_ tells us whether we can spawn a new solve on a thread
  bool is_active_;
  // has_solution_ indicates whether there exists a solution
  bool has_solution_;
  std::unique_ptr<std::thread> thread_;

  drake::VectorX<double> solution_;

  const drake::multibody::MultibodyPlant<double>& plant_;
};

}  // namespace dairlib::goldilocks_models
