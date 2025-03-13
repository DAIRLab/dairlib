#pragma once

#include "id_mpc_params.h"

#include "core/knot_point_state.h"
#include "core/constrained_inverse_dynamics_info.h"
#include "core/timeline.h"

#include "constraints/collocation_constraint.h"
#include "constraints/kinematic_constraint.h"
#include "constraints/quaternion_norm_constraint.h"

#include "references/reference_manager.h"
#include "references/mpc_reference.h"

#include "solvers/qp_data.h"

#include "lcm/lcm_trajectory.h"

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"

namespace dairlib::systems::controllers::id_mpc {

using ForceEvaluatorsMap =
    std::unordered_map<std::string,
    std::vector<drake::solvers::LinearConstraint*>>;


class IDMPC {
 public:
  IDMPC(IDMPCParams params, std::unique_ptr<ConstrainedDynamicsInfo> dynamics);

  const drake::solvers::VectorXDecisionVariable knot_vars(int index) const {
    DRAKE_DEMAND(index <= params_.N);
    return knot_point_vars_.at(index);
  }
  const drake::solvers::VectorXDecisionVariable position_vars(int index) const {
    return timeline_.knots.at(index)->get_q(knot_vars(index));
  }
  const drake::solvers::VectorXDecisionVariable velocity_vars(int index) const {
    return timeline_.knots.at(index)->get_v(knot_vars(index));
  }
  const drake::solvers::VectorXDecisionVariable lambda_vars(int index) const {
    return timeline_.knots.at(index)->get_lambda(knot_vars(index));
  }
  const drake::solvers::VectorXDecisionVariable input_vars(int index) const {
    return timeline_.knots.at(index)->get_u(knot_vars(index));
  }

  bool has_lambdas_at_knot(int index) const {
    return timeline_.knots.at(index)->num_input_variables() > 0;
  }
  bool has_torques_at_knot(int index) const {
    return timeline_.knots.at(index)->has_torques();
  }

  int num_vars() const {return prog_.num_vars();}
  int num_constraints() const {return num_constraints_;}

  /*!
   * Set the value of the indices of the given decision variables within z to
   * the given value
   */
  void SetDecisionVariableValue(
      const drake::solvers::VectorXDecisionVariable& var,
      const Eigen::VectorXd& value,
      Eigen::VectorXd* z) const;

  Eigen::VectorXd GetDecisionVariableValue(
      const drake::solvers::VectorXDecisionVariable& var,
      const Eigen::VectorXd& z) const;

  /*!
   * Update the costs and constraints to reflect the desired trajectories and
   * contact constraints specified by reference
   * @param reference MPCReference object specifying the reference trasjectory
   * @param initial_state initial state of the MPC
   */
  void UpdateProblemData(const MPCReference& reference,
                         const Eigen::VectorXd& initial_state,
                         const Eigen::VectorXd& prev_sol);

  /*!
   * Copy the MPC solution into an LCMTrajectory
   * @param result mathematical program containing the solution
   */
  LcmTrajectory GetSolutionAsLcmTrajectory(
      const drake::solvers::MathematicalProgramResult& result) const;

  drake::solvers::MathematicalProgram& get_prog() { return prog_; }

  /*!
   * Create a quadratic approximation of the MPC problem about z, where z is
   * a set of stacked decision variables. The QP returned is the quadratic
   * approximation of the primal problem with dz = (z - z*) as the decision
   * variable, and all constraints represented as inequalities
   *
   * @param z Current value of all the MPC variables
   * @param qp QPData to hold the resulting QP
   */
  void ConstructSQPProgram(const Eigen::VectorXd& z, solvers::QPData* qp) const;

  /*!
   * Evaluate the maximum constraint violation (for the full nonlinear
   * constraint set), scaled by the MPC timestep,
   * for a candidate solution z.
   */
  double EvaluateConstraintViolation(const Eigen::VectorXd& z) const;

  /*!
   * Evaluate the full nonlinear cost for a candidate solution z
   */
  double EvaluateCost(const Eigen::VectorXd& z) const;

  const ConstrainedDynamicsInfo& dynamics() const {
    return *dynamics_;
  }

  /*!
   * Normalize any variables associated with quaternion position coordinates in
   * the solution z
   *
   *  TODO (@Brian-Acosta) use the plant to do this automatically instead of
   *   assuming a single floating base
   *
   */
  void ProjectToQuaternionConstraint(Eigen::VectorXd* z) const;

  /*!
   * Add a cost on the deviation of a kinematic function from a trajectory
   *
   * TODO (@Brian-Acosta) support costs which include velocities
   *
   * @tparam C type of cost to add
   * @param name name of the cost
   * @param args arguments to c's constructor
   */
  template<class C, typename... Args>
  void AddTaskCost(const std::string& name, Args&&... args) {
    DRAKE_DEMAND(reference_manager_.IsInitialized());
    reference_manager_.AddRunningStateCost<C>(
        name, std::forward<Args>(args)...);
    for (int i = 0; i < params_.N + 1; ++i) {
      prog_.AddCost(
          reference_manager_.GetEvaluator(name, i), position_vars(i));
    }
  }

 private:
  void UpdateCosts(const MPCReference& reference);
  void UpdateInitialState(const Eigen::VectorXd& x);
  void UpdateActiveContacts(int knot_index, std::vector<std::string> contacts);
  void UpdateFrictionCone(
      int knot_index, const std::vector<std::string>& active_contacts);


  void MakeMPCCosts();
  void MakeKnotPoints();
  void MakeForceLimits();
  void MakeKinematicConstraints();
  void MakeCollocationConstraints();

  const IDMPCParams params_;

  std::unique_ptr<ConstrainedDynamicsInfo> dynamics_;
  Timeline timeline_;

  ForceEvaluatorsMap contact_force_limits_{};
  ReferenceManager<double> reference_manager_;

  drake::solvers::LinearEqualityConstraint* initial_state_constraint_;
  drake::solvers::MathematicalProgram prog_;

  std::shared_ptr<solvers::sqp::QuadraticErrorCost<double>> smoothness_cost_;

  std::vector<drake::solvers::VectorXDecisionVariable> knot_point_vars_;
  int num_constraints_;
};

}