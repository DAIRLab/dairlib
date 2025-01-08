#pragma once

#include "core/knot_point_state.h"
#include "core/constrained_inverse_dynamics_info.h"
#include "core/collocation_constraint.h"
#include "core/kinematic_constraint.h"
#include "core/timeline.h"
#include "core/quaternion_norm_constraint.h"

#include "costs/nonlinear_least_squares_cost.h"

#include "sqp/qp_data.h"

#include "lcm/lcm_trajectory.h"

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"

namespace dairlib::systems::controllers::id_mpc {

struct IDMPCParams {
  int N;
  double dt;
};

class IDMPC {
 public:
  IDMPC(IDMPCParams params, std::unique_ptr<ConstrainedDynamicsInfo> dynamics);

  const drake::solvers::VectorXDecisionVariable knot_vars(int index) const {
    DRAKE_DEMAND(index <= params_.N);
    return knot_point_vars_.at(index);
  }
  const drake::solvers::VectorXDecisionVariable position_vars(int index) const {
    return knot_vars(index).head(dynamics_->nq());
  }
  const drake::solvers::VectorXDecisionVariable velocity_vars(int index) const {
    return knot_vars(index).segment(dynamics_->nq(), dynamics_->nv());
  }
  const drake::solvers::VectorXDecisionVariable input_vars(int index) const {
    return knot_vars(index).segment(dynamics_->nx(), dynamics_->nu());
  }
  const drake::solvers::VectorXDecisionVariable lambda_h_vars(int index) const {
    return knot_vars(index).segment(dynamics_->nx() + dynamics_->nu(),
                                    dynamics_->nh());
  }
  const drake::solvers::VectorXDecisionVariable lambda_c_vars(int index) const {
    return knot_vars(index).tail(dynamics_->nc());
  }

  void AddUnitQuaternionConstraintToAllFloatingBodies();

  LcmTrajectory GetSolutionAsLcmTrajectory(
      const drake::solvers::MathematicalProgramResult& result) const;

  void UpdateActiveContacts(int knot_index, std::vector<std::string> contacts);
  void UpdateInitialState(const Eigen::VectorXd& x);

  drake::solvers::MathematicalProgram& get_prog() { return prog_; }


  void ConstructSQPProgram(const Eigen::VectorXd& x, QPData& qp) const;


 private:

  void ParseCostsToQP(const Eigen::VectorXd& x, QPData& qp) const;
  void ParseConstraintsToQP(const Eigen::VectorXd& x, QPData& qp) const;
  const IDMPCParams params_;

  std::unique_ptr<ConstrainedDynamicsInfo> dynamics_;
  Timeline timeline_;

  std::vector<drake::solvers::Binding<drake::solvers::Constraint>> nonlin_constraints_;
  std::vector<drake::solvers::Binding<drake::solvers::Cost>> costs_;

  std::shared_ptr<CollocationConstraint<double>> dynamics_constraint_;
  drake::solvers::LinearEqualityConstraint* initial_state_constraint_;

  drake::solvers::MathematicalProgram prog_;

  std::vector<drake::solvers::VectorXDecisionVariable> knot_point_vars_;
  std::shared_ptr<QuaternionNormConstraint<AutoDiffXd>> unit_quat_ = nullptr;
};

}