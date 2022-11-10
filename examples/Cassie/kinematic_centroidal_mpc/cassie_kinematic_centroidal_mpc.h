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
      loop_closure_evaluators(Plant()),
      slip_contact_sequence_(n_knot_points){
    AddPlantJointLimits(dairlib::JointNames());
    AddLoopClosure();

    //TODO Add slip decision variables
    //TODO Create lifter functions
    std::vector<bool> stance_mode(2);
    std::fill(stance_mode.begin(), stance_mode.end(), true);
    std::fill(slip_contact_sequence_.begin(), slip_contact_sequence_.end(), stance_mode);
  }

  /*!
   * @brief creates vector of world point evaluators for cassie
   * @param plant cassie plant
   * @param mu coefficient of friction
   * @return
   */
  std::vector<dairlib::multibody::WorldPointEvaluator<double>> CreateContactPoints(const drake::multibody::MultibodyPlant<double>& plant, double mu);

  /*!
 * @brief Set the mode sequence
 * @param contact_sequence vector where
 * `contact_sequence[knot_point][contact_index]` tells you if at `knot_point`
 * is `contact_index` active
 */
  void SetModeSequence(const std::vector<std::vector<bool>>& contact_sequence) override;

  /*!
   * @brief Set the mode sequence via a trajectory. The value of the trajectory
   * at each time, cast to a bool is if a contact point is active or not
   * @param contact_sequence
   */
  void SetModeSequence(
      const drake::trajectories::PiecewisePolynomial<double>& contact_sequence) override;

 private:
  void MapModeSequence();

  void AddPlanarSlipConstraints(int knot_point) override;

  void AddPlanarSlipCost(double t, double terminal_gain) override;

  /*!
   * @brief Adds loop closure constraints to the mpc
   */
  void AddLoopClosure();

  dairlib::multibody::DistanceEvaluator<double> l_loop_evaluator_;
  dairlib::multibody::DistanceEvaluator<double> r_loop_evaluator_;
  dairlib::multibody::KinematicEvaluatorSet<double> loop_closure_evaluators;

  std::vector<std::vector<bool>> slip_contact_sequence_;

  std::map<std::vector<bool>, std::vector<bool>> complex_mode_to_slip_mode_{{{true, true, true, true},{true, true}},
                                                                            {{true, true, false, false},{true, false}},
                                                                            {{false, false, true, true},{false, true}},
                                                                            {{false, false, false, false},{false, false}}};
};

