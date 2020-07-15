#include "examples/goldilocks_models/planning/optimal_rom_planner.h"

#include <math.h>
#include <string>

using std::cout;
using std::endl;
using std::string;
using std::vector;

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::VectorX;
using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::trajectories::ExponentialPlusPiecewisePolynomial;
using drake::trajectories::PiecewisePolynomial;

using dairlib::systems::OutputVector;

namespace dairlib::goldilocks_models {

RomMPC::RomMPC(const MultibodyPlant<double>& plant)
    : is_solved_(false),
      is_active_(false),
      has_solution_(false),
      plant_(plant){};

void RomMPC::StartSolveIfNotBusy(const VectorX<double>& init_guess) {
  // delete the previous thread if it's done
  if (is_solved_ && is_active_) {
    // must join before deleting the thread
    thread_->join();
    thread_.reset();  // delete the std::thread object
    is_active_ = false;
  }

  // Start solving if there is no active thread
  if (!is_active_) {
    is_solved_ = false;  // Due to thread asynchrony, this flag can not be
                         // placed inside Solve() which is spawned on a new
                         // thead
    thread_ = std::make_unique<std::thread>(&RomMPC::Solve, this,
                                            std::ref(init_guess));
    is_active_ = true;
  }
}

drake::VectorX<double> RomMPC::GetSolution() const {
  DRAKE_ASSERT(has_solution_);
  return solution_;
}

void RomMPC::Solve(const VectorX<double>& init_guess) {
  cout << "init_guess = " << init_guess.transpose() << endl;

  auto start = std::chrono::high_resolution_clock::now();
  // Solve trajopt here
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  cout << "computation time = " << elapsed.count() << endl;

  cout << "Finished solving\n";
  // Assign solution

  // Update flags
  is_solved_ = true;
  has_solution_ = true;
}

}  // namespace dairlib::goldilocks_models
