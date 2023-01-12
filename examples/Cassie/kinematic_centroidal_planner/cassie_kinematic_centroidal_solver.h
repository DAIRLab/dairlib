#pragma once
#include <iostream>

#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/kinematic_centroidal_planner/simple_models/slip_lifter.h"
#include "examples/Cassie/kinematic_centroidal_planner/simple_models/slip_reducer.h"
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
      double stance_width = 0.2);

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

  /*!
   * @brief Overload of setting kc com position to also set slip com position
   * @param com_trajectory
   */
  void SetComPositionGuess(
      const drake::trajectories::PiecewisePolynomial<double>& com_trajectory)
      override;

  /*!
   * @brief overload of setting kc generalized state guess to also set slip
   * contact postion and velocity
   * @param q_traj
   * @param v_traj
   */
  void SetRobotStateGuess(
      const drake::trajectories::PiecewisePolynomial<double>& q_traj,
      const drake::trajectories::PiecewisePolynomial<double>& v_traj) override;

  /*!
   * @brief overload of setting kc momentum guess to set slip com velocity
   * @param momentum_trajectory
   */
  void SetMomentumGuess(const drake::trajectories::PiecewisePolynomial<double>&
                            momentum_trajectory) override;

  /*!
   * @brief overload of setting kc force to set initial guess for slip force
   * @param force_trajectory
   */
  void SetForceGuess(const drake::trajectories::PiecewisePolynomial<double>&
                         force_trajectory) override;

  void Build(const drake::solvers::SolverOptions& solver_options) override;

  void SetMaximumSlipLegLength(double max_leg_length);

 private:
  /*!
   * @brief Add constraint at knot point for complex state to be on the slip
   * submanifold
   * @param knot_point
   */
  void AddSlipPosturePrincipleConstraint(int knot_point);

  /*!
   * @brief map complex mode sequence to the simple mode sequence
   */
  void MapModeSequence();

  void AddSlipConstraints(int knot_point) override;

  void AddSlipCost(int knot_point, double terminal_gain) override;

  void AddSlipEqualityConstraint(int knot_point) override;

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
  double max_slip_leg_length_ = 10;

  std::vector<drake::solvers::VectorXDecisionVariable> slip_com_vars_;
  std::vector<drake::solvers::VectorXDecisionVariable> slip_vel_vars_;
  std::vector<drake::solvers::VectorXDecisionVariable> slip_contact_pos_vars_;
  std::vector<drake::solvers::VectorXDecisionVariable> slip_contact_vel_vars_;
  std::vector<drake::solvers::VectorXDecisionVariable> slip_force_vars_;

  std::vector<std::shared_ptr<SlipLifter>> lifters_;
  std::vector<std::shared_ptr<SlipReducer>> reducers;

  std::vector<dairlib::multibody::WorldPointEvaluator<double>>
      slip_contact_points_;

  std::unordered_map<std::vector<bool>, std::vector<bool>>
      complex_mode_to_slip_mode_{
          {{true, true, true, true}, {true, true}},
          {{true, true, false, false}, {true, false}},
          {{false, false, true, true}, {false, true}},
          {{false, false, false, false}, {false, false}}};
  std::vector<drake::solvers::Binding<drake::solvers::Constraint>>
      slip_dynamics_binding_;

  std::map<std::string, int> positions_map_;
  std::map<std::string, int> velocity_map_;
};
