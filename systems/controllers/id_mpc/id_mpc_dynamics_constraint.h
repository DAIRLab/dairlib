#pragma once

#include "id_mpc_types.h"
#include "constrained_dynamics_info.h"
#include "solvers/nonlinear_constraint.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib::systems::controllers::id_mpc {

/*!
 * Wrapper class representing a trapezoidal collocation constraint on the
 * dynamics of the robot:
 *
 * follows the manipulator equations Mv̇ + c = Bu + Jₕᵀλₕ + J_cᵀλ_c + Jₑᵀλₑ
*/
class IDMPCDynamicsConstraint : solvers::NonlinearConstraint<drake::AutoDiffXd> {
 public:
  IDMPCDynamicsConstraint(const ConstrainedDynamicsInfo& constraints_);

  void UpdateActiveContacts(std::vector<std::string> active_contacts) {
    active_contacts_ = active_contacts;
    nc_ = 3 * active_contacts.size();
  }

 private:

  void EvaluateConstraint(
      const Eigen::Ref<const drake::VectorX<drake::AutoDiffXd>>& x,
      drake::VectorX<drake::AutoDiffXd>* y) const override;


  const ConstrainedDynamicsInfo& constraints_;
  std::vector<std::string> active_contacts_{};

  // Size of velocity and input of the plant
  int nq_{};
  int nv_{};
  int nu_{};
  int nh_{};
  int nc_{};

};

}
