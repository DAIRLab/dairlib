#pragma once

#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "solvers/fast_osqp_solver.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/solve.h"

// Dynamics QP is a general purpose interface for solving the manipulator
// equations subject to constraints like contact forces, inputs, etc. It can be
// used for things like operational space control, force control, etc.

namespace dairlib {
namespace multibody {
class DynamicsQP {
 public:
  DynamicsQP(const drake::multibody::MultibodyPlant<double> &plant,
             drake::systems::Context<double> *context);

  /*!
   * @brief Add the permanent kinematic constraints defined by evaluators
   */
  void AddKinematicConstraint(
      std::unique_ptr<const KinematicEvaluatorSet<double>> evaluators) {
    DRAKE_DEMAND(&evaluators->plant() == &plant_);
    kinematic_evaluators_ = std::move(evaluators);
  }



 private:

  const drake::multibody::MultibodyPlant<double> &plant_;
  drake::systems::Context<double> *context_;
  std::unique_ptr<const KinematicEvaluatorSet<double>> kinematic_evaluators_;


};
}
}