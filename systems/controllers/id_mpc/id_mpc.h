#pragma once

#include "core/knot_point_state.h"
#include "core/constrained_inverse_dynamics_info.h"
#include "core/collocation_constraint.h"
#include "core/kinematic_constraint.h"
#include "core/timeline.h"
#include "core/quaternion_norm_constraint.h"

#include "costs/reference_manager.h"

#include "solvers/qp_data.h"

#include "lcm/lcm_trajectory.h"

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"

namespace dairlib::systems::controllers::id_mpc {

struct IDMPCParams {
  int N;
  double dt;
  int num_full_torque_knots;
};

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
  const drake::solvers::VectorXDecisionVariable input_vars(int index) const {
    return timeline_.knots.at(index)->get_u(knot_vars(index));
  }
  const drake::solvers::VectorXDecisionVariable lambda_vars(int index) const {
    return timeline_.knots.at(index)->get_lambda(knot_vars(index));
  }

  LcmTrajectory GetSolutionAsLcmTrajectory(
      const drake::solvers::MathematicalProgramResult& result) const;

  void UpdateActiveContacts(int knot_index, std::vector<std::string> contacts);
  void UpdateInitialState(const Eigen::VectorXd& x);

  drake::solvers::MathematicalProgram& get_prog() { return prog_; }

  void ConstructSQPProgram(const Eigen::VectorXd& x, solvers::QPData& qp) const;
  void UpdateSQPProgram(const Eigen::VectorXd& x, solvers::QPData& qp) const;
  double EvaluateConstraintViolation(const Eigen::VectorXd& x) const;
  double EvaluateCost(const Eigen::VectorXd& x) const;

  const ConstrainedDynamicsInfo& dynamics() const {
    return *dynamics_;
  }

 private:

  // TODO (@Brian-Acosta) Methods for setting and updating costs,
  //  Methods for creating and updating contact and friction cone constraints
  void MakeKnotPoints();
  void MakeKinematicConstraints();
  void MakeCollocationConstraints();
  void MakeUnitQuaternionConstraints();


  void ParseCostsToSQP(const Eigen::VectorXd& x, solvers::QPData& qp) const;
  void ParseConstraintsToSQP(const Eigen::VectorXd& x, solvers::QPData& qp) const;
  const IDMPCParams params_;

  std::unique_ptr<ConstrainedDynamicsInfo> dynamics_;
  Timeline timeline_;
  std::vector<drake::solvers::Binding<drake::solvers::Constraint>> nonlin_constraints_;

  ReferenceManager<double> reference_manager_;

  drake::solvers::LinearEqualityConstraint* initial_state_constraint_;
  drake::solvers::MathematicalProgram prog_;

  std::vector<drake::solvers::VectorXDecisionVariable> knot_point_vars_;
  std::shared_ptr<QuaternionNormConstraint<AutoDiffXd>> unit_quat_ = nullptr;
};

}