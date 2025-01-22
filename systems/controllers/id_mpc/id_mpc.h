#pragma once

#include "id_mpc_params.h"

#include "core/knot_point_state.h"
#include "core/constrained_inverse_dynamics_info.h"
#include "systems/controllers/id_mpc/contraints/collocation_constraint.h"
#include "systems/controllers/id_mpc/contraints/kinematic_constraint.h"
#include "core/timeline.h"
#include "systems/controllers/id_mpc/contraints/quaternion_norm_constraint.h"

#include "costs/reference_manager.h"
#include "costs/mpc_reference.h"

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

  void UpdateProblemData(const MPCReference& reference,
                         const Eigen::VectorXd& initial_state);

  LcmTrajectory GetSolutionAsLcmTrajectory(
      const drake::solvers::MathematicalProgramResult& result) const;

  drake::solvers::MathematicalProgram& get_prog() { return prog_; }

  void ConstructSQPProgram(const Eigen::VectorXd& x, solvers::QPData& qp) const;
  double EvaluateConstraintViolation(const Eigen::VectorXd& x) const;
  double EvaluateCost(const Eigen::VectorXd& x) const;

  const ConstrainedDynamicsInfo& dynamics() const {
    return *dynamics_;
  }

  void ProjectToQuaternionConstraint(Eigen::VectorXd& x);

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
  void MakeUnitQuaternionConstraints();

  void ParseCostsToSQP(const Eigen::VectorXd& x, solvers::QPData& qp) const;
  void ParseConstraintsToSQP(const Eigen::VectorXd& x, solvers::QPData& qp) const;
  const IDMPCParams params_;

  std::unique_ptr<ConstrainedDynamicsInfo> dynamics_;
  Timeline timeline_;
  std::vector<drake::solvers::Binding<drake::solvers::Constraint>> nonlin_constraints_;
  std::vector<drake::solvers::Binding<drake::solvers::Constraint>> quat_contraints_;

  ForceEvaluatorsMap contact_force_limits_{};
  ReferenceManager<double> reference_manager_;

  drake::solvers::LinearEqualityConstraint* initial_state_constraint_;
  drake::solvers::MathematicalProgram prog_;

  std::vector<drake::solvers::VectorXDecisionVariable> knot_point_vars_;
  std::shared_ptr<QuaternionNormConstraint<AutoDiffXd>> unit_quat_ = nullptr;

  int num_constraints_;
};

}