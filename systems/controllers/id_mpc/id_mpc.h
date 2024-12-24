#pragma once

#include "knot_point_state.h"
#include "constrained_dynamics_info.h"
#include "trapezoidal_collocation_constraint.h"
#include "kinematic_constraint.h"

#include "drake/solvers/mathematical_program.h"

namespace dairlib::systems::controllers::id_mpc {

struct IDMPCParams {
  int N;
  double dt;
};

class IDMPC {
 public:
  IDMPC(
      IDMPCParams params, std::unique_ptr<ConstrainedDynamicsInfo> dynamics);

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

  void SetActiveContacts(int knot_index, std::vector<std::string> contacts);
  void SetInitialState(const Eigen::VectorXd& x);

  drake::solvers::MathematicalProgram& get_prog() { return prog_; }

 private:
  const IDMPCParams params_;

  std::unique_ptr<ConstrainedDynamicsInfo> dynamics_;
  std::vector<KnotPointState> knot_point_work_;

  drake::solvers::MathematicalProgram prog_;
  drake::solvers::LinearEqualityConstraint* initial_state_constraint_;
  std::vector<drake::solvers::VectorXDecisionVariable> knot_point_vars_;
  std::vector<drake::solvers::Binding<drake::solvers::Constraint>> dynamics_constraints_;
  std::vector<drake::solvers::Binding<drake::solvers::Constraint>> kinematic_constraints_;

};

}