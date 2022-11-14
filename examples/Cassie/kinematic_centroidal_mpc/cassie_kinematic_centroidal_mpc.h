#pragma once
#include <iostream>
#include "systems/controllers/kinematic_centroidal_mpc/kinematic_centroidal_mpc.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/kinematic_centroidal_mpc/simple_models/planar_slip_lifter.h"

/*!
 * @brief Cassie specific child class for kinematic centroidal mpc. Adds loop closure, joint limits, and cassie contact points
 */
class CassieKinematicCentroidalMPC : public KinematicCentroidalMPC {
 public:
  CassieKinematicCentroidalMPC(const drake::multibody::MultibodyPlant<double>& plant,
                               int n_knot_points,
                               double dt,
                               double mu,
                               const drake::VectorX<double>& nominal_stand,
                               double k = 1000,
                               double r0 = 0.5,
                               double stance_width = 0.2) :
      KinematicCentroidalMPC(plant, n_knot_points, dt, CreateContactPoints(plant, mu)),
      l_loop_evaluator_(dairlib::LeftLoopClosureEvaluator(Plant())),
      r_loop_evaluator_(dairlib::RightLoopClosureEvaluator(Plant())),
      loop_closure_evaluators(Plant()),
      slip_contact_sequence_(n_knot_points),
      k_(k),
      r0_(r0){
    AddPlantJointLimits(dairlib::JointNames());
    AddLoopClosure();

    slip_contact_points_ = CreateSlipContactPoints(plant,mu);
    for (int knot = 0; knot < n_knot_points; knot++) {
      slip_com_vars_.push_back(prog_->NewContinuousVariables(
          2, "slip_com_" + std::to_string(knot)));
      slip_vel_vars_.push_back(prog_->NewContinuousVariables(
          2, "slip_vel_" + std::to_string(knot)));
      slip_contact_pos_vars_.push_back(prog_->NewContinuousVariables(
          2*2, "slip_contact_pos_" + std::to_string(knot)));
      slip_contact_vel_vars_.push_back(prog_->NewContinuousVariables(
          2*2, "slip_contact_vel_" + std::to_string(knot)));
      lifters_.emplace_back(new PlanarSlipLifter(plant,
                                                 contexts_[knot].get(),
                                                 slip_contact_points_,
                                                 CreateContactPoints(plant, mu),
                                                 {{0, {0, 1}}, {1, {2, 3}}},
                                                 nominal_stand,
                                                 k,
                                                 r0,
                                                 {stance_width / 2, -stance_width / 2}));
    }
    std::vector<bool> stance_mode(2);
    std::fill(stance_mode.begin(), stance_mode.end(), true);
    std::fill(slip_contact_sequence_.begin(), slip_contact_sequence_.end(), stance_mode);

    m_=plant_.CalcTotalMass(*contexts_[0]);
  }

  /*!
   * @brief creates vector of world point evaluators for cassie
   * @param plant cassie plant
   * @param mu coefficient of friction
   * @return
   */
  std::vector<dairlib::multibody::WorldPointEvaluator<double>> CreateContactPoints(const drake::multibody::MultibodyPlant<double>& plant, double mu);

  std::vector<dairlib::multibody::WorldPointEvaluator<double>> CreateSlipContactPoints(const drake::multibody::MultibodyPlant<double>& plant, double mu);

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

  drake::solvers::VectorXDecisionVariable slip_contact_pos_vars(int knot_point_index, int slip_foot_index);

  drake::solvers::VectorXDecisionVariable slip_contact_vel_vars(int knot_point_index, int slip_foot_index);

 private:
  void MapModeSequence();

  void AddPlanarSlipConstraints(int knot_point) override;

  void AddPlanarSlipCost(int knot_point, double terminal_gain) override;

  void AddSlipReductionConstraint(int knot_point) override;

  void AddSlipLiftingConstraint(int knot_point)override;

  void AddSlipDynamics(int knot_point)override;

  /*!
   * @brief Adds loop closure constraints to the mpc
   */
  void AddLoopClosure();

  dairlib::multibody::DistanceEvaluator<double> l_loop_evaluator_;
  dairlib::multibody::DistanceEvaluator<double> r_loop_evaluator_;
  dairlib::multibody::KinematicEvaluatorSet<double> loop_closure_evaluators;

  std::vector<std::vector<bool>> slip_contact_sequence_;
  double k_;
  double r0_;
  double m_;

  const double slip_ground_offset_ = 0.0562;

  std::vector<drake::solvers::VectorXDecisionVariable> slip_com_vars_;
  std::vector<drake::solvers::VectorXDecisionVariable> slip_vel_vars_;
  std::vector<drake::solvers::VectorXDecisionVariable> slip_contact_pos_vars_;
  std::vector<drake::solvers::VectorXDecisionVariable> slip_contact_vel_vars_;

  std::vector<std::shared_ptr<PlanarSlipLifter>> lifters_;

  std::vector<dairlib::multibody::WorldPointEvaluator<double>> slip_contact_points_;

  std::map<std::vector<bool>, std::vector<bool>> complex_mode_to_slip_mode_{{{true, true, true, true},{true, true}},
                                                                            {{true, true, false, false},{true, false}},
                                                                            {{false, false, true, true},{false, true}},
                                                                            {{false, false, false, false},{false, false}}};
};

