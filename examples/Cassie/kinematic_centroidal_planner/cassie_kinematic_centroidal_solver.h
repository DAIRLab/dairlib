#pragma once
#include <iostream>
#include "systems/trajectory_optimization/kinematic_centroidal_planner/kinematic_centroidal_solver.h"
#include "systems/trajectory_optimization/kinematic_centroidal_planner/reference_generation_utils.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "examples/Cassie/cassie_utils.h"

/*!
 * @brief Cassie specific child class for kinematic centroidal mpc. Adds loop closure, joint limits, and cassie contact points
 */
class CassieKinematicCentroidalSolver : public KinematicCentroidalSolver {
 public:
  CassieKinematicCentroidalSolver(const drake::multibody::MultibodyPlant<double>& plant, int n_knot_points, double dt, double mu) :
      KinematicCentroidalSolver(plant, n_knot_points, dt, CreateContactPoints(plant, mu)),
      l_loop_evaluator_(dairlib::LeftLoopClosureEvaluator(Plant())),
      r_loop_evaluator_(dairlib::RightLoopClosureEvaluator(Plant())),
      loop_closure_evaluators(Plant()){
    AddPlantJointLimits(dairlib::JointNames());
    AddLoopClosure();
  }

private:

  /*!
   * @brief Adds loop closure constraints to the mpc
   */
  void AddLoopClosure();

  dairlib::multibody::DistanceEvaluator<double> l_loop_evaluator_;
  dairlib::multibody::DistanceEvaluator<double> r_loop_evaluator_;
  dairlib::multibody::KinematicEvaluatorSet<double> loop_closure_evaluators;

};

