#pragma once
#include <iostream>
#include "systems/controllers/kinematic_centroidal_mpc/kinematic_centroidal_mpc.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "examples/Cassie/cassie_utils.h"

/*!
 * @brief Cassie specific child class for kinematic centroidal mpc. Adds loop closure, joint limits, and cassie contact points
 */
class CassieKinematicCentroidalMPC : public KinematicCentroidalMPC {
 public:
  CassieKinematicCentroidalMPC(const drake::multibody::MultibodyPlant<double>& plant, int n_knot_points, double dt, double mu) :
      KinematicCentroidalMPC(plant, n_knot_points, dt, CreateContactPoints(plant, mu)),
      l_loop_evaluator_(dairlib::LeftLoopClosureEvaluator(Plant())),
      r_loop_evaluator_(dairlib::RightLoopClosureEvaluator(Plant())),
      loop_closure_evaluators(Plant()){
    AddPlantJointLimits(dairlib::JointNames());
    AddLoopClosure();
  }

  /*!
   * @brief creates vector of world point evaluators for cassie
   * @param plant cassie plant
   * @param mu coefficient of friction
   * @return
   */
  std::vector<dairlib::multibody::WorldPointEvaluator<double>> CreateContactPoints(const drake::multibody::MultibodyPlant<double>& plant, double mu);
 private:

  /*!
   * @brief Adds loop closure constraints to the mpc
   */
  void AddLoopClosure();

  dairlib::multibody::DistanceEvaluator<double> l_loop_evaluator_;
  dairlib::multibody::DistanceEvaluator<double> r_loop_evaluator_;
  dairlib::multibody::KinematicEvaluatorSet<double> loop_closure_evaluators;

};

