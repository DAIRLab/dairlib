#pragma once
#include <iostream>

#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/kinematic_centroidal_planner/simple_models/planar_slip_lifter.h"
#include "examples/Cassie/kinematic_centroidal_planner/simple_models/planar_slip_reducer.h"
#include "systems/trajectory_optimization/kinematic_centroidal_planner/kinematic_centroidal_solver.h"
#include "systems/trajectory_optimization/kinematic_centroidal_planner/reference_generation_utils.h"

#include "drake/multibody/plant/multibody_plant.h"

/*!
 * @brief Cassie specific child class for kinematic centroidal mpc. Adds loop
 * closure, joint limits, and cassie contact points
 */
class CassieKinematicCentroidalSolver : public KinematicCentroidalSolver {
 public:
  CassieKinematicCentroidalSolver(
      const drake::multibody::MultibodyPlant<double>& plant, int n_knot_points,
      double dt, double mu, const drake::VectorX<double>& nominal_stand,
      double k = 1000, double b = 20, double r0 = 0.5,
      double stance_width = 0.2)
      : KinematicCentroidalSolver(plant, n_knot_points, dt,
                                  CreateContactPoints(plant, mu)),
        l_loop_evaluator_(dairlib::LeftLoopClosureEvaluator(Plant())),
        r_loop_evaluator_(dairlib::RightLoopClosureEvaluator(Plant())),
        loop_closure_evaluators(Plant()),
        slip_contact_sequence_(n_knot_points),
        k_(k),
        r0_(r0),
        b_(b),
        nominal_stand_(nominal_stand) {
    AddPlantJointLimits(dairlib::JointNames());
    AddLoopClosure();

    slip_contact_points_ = CreateSlipContactPoints(plant, mu);
    for (int knot = 0; knot < n_knot_points; knot++) {
      slip_com_vars_.push_back(
          prog_->NewContinuousVariables(3, "slip_com_" + std::to_string(knot)));
      slip_vel_vars_.push_back(
          prog_->NewContinuousVariables(3, "slip_vel_" + std::to_string(knot)));
      slip_contact_pos_vars_.push_back(prog_->NewContinuousVariables(
          2 * 3, "slip_contact_pos_" + std::to_string(knot)));
      slip_contact_vel_vars_.push_back(prog_->NewContinuousVariables(
          2 * 3, "slip_contact_vel_" + std::to_string(knot)));
      slip_force_vars_.push_back(prog_->NewContinuousVariables(
          2, "slip_force_" + std::to_string(knot)));
    }
    std::vector<bool> stance_mode(2);
    std::fill(stance_mode.begin(), stance_mode.end(), true);
    std::fill(slip_contact_sequence_.begin(), slip_contact_sequence_.end(),
              stance_mode);

    m_ = plant_.CalcTotalMass(*contexts_[0]);
  }

  std::vector<dairlib::multibody::WorldPointEvaluator<double>>
  CreateSlipContactPoints(const drake::multibody::MultibodyPlant<double>& plant,
                          double mu);

  /*!
   * @brief Set the mode sequence
   * @param contact_sequence vector where
   * `contact_sequence[knot_point][contact_index]` tells you if at `knot_point`
   * is `contact_index` active
   */
  void SetModeSequence(
      const std::vector<std::vector<bool>>& contact_sequence) override;

  /*!
   * @brief Set the mode sequence via a trajectory. The value of the trajectory
   * at each time, cast to a bool is if a contact point is active or not
   * @param contact_sequence
   */
  void SetModeSequence(const drake::trajectories::PiecewisePolynomial<double>&
                           contact_sequence) override;

  drake::solvers::VectorXDecisionVariable slip_contact_pos_vars(
      int knot_point_index, int slip_foot_index);

  drake::solvers::VectorXDecisionVariable slip_contact_vel_vars(
      int knot_point_index, int slip_foot_index);

  void SetComPositionGuess(
      const drake::trajectories::PiecewisePolynomial<double>& com_trajectory)
      override;

  void SetRobotStateGuess(
      const drake::trajectories::PiecewisePolynomial<double>& q_traj,
      const drake::trajectories::PiecewisePolynomial<double>& v_traj) override;

  void SetMomentumGuess(const drake::trajectories::PiecewisePolynomial<double>&
                            momentum_trajectory) override;

  void SetForceGuess(const drake::trajectories::PiecewisePolynomial<double>&
                         force_trajectory) override;

  void Build(const drake::solvers::SolverOptions& solver_options) override;

 private:
  void MapModeSequence();

  void AddPlanarSlipConstraints(int knot_point) override;

  void AddPlanarSlipCost(int knot_point, double terminal_gain) override;

  void AddSlipReductionConstraint(int knot_point) override;

  void AddSlipLiftingConstraint(int knot_point) override;

  void AddSlipDynamics(int knot_point) override;

  drake::VectorX<double> LiftSlipSolution(int knot_point) override;

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
  double b_;
  double m_;
  const drake::VectorX<double> nominal_stand_;
  const double slip_ground_offset_ = 0;

  std::vector<drake::solvers::VectorXDecisionVariable> slip_com_vars_;
  std::vector<drake::solvers::VectorXDecisionVariable> slip_vel_vars_;
  std::vector<drake::solvers::VectorXDecisionVariable> slip_contact_pos_vars_;
  std::vector<drake::solvers::VectorXDecisionVariable> slip_contact_vel_vars_;
  std::vector<drake::solvers::VectorXDecisionVariable> slip_force_vars_;

  std::vector<std::shared_ptr<PlanarSlipLifter>> lifters_;
  std::vector<std::shared_ptr<PlanarSlipReducer>> reducers;

  std::vector<dairlib::multibody::WorldPointEvaluator<double>>
      slip_contact_points_;

  std::map<std::vector<bool>, std::vector<bool>> complex_mode_to_slip_mode_{
      {{true, true, true, true}, {true, true}},
      {{true, true, false, false}, {true, false}},
      {{false, false, true, true}, {false, true}},
      {{false, false, false, false}, {false, false}}};
  std::vector<drake::solvers::Binding<drake::solvers::Constraint>>
      slip_dynamics_binding_;
};
