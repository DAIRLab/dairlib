#include "cf_mpfc_constraints.h"
#include "systems/controllers/footstep_planning/nonlinear_pendulum_utils.h"

namespace dairlib::systems::controllers {

using Eigen::VectorXd;

using drake::VectorX;
using drake::Vector6;
using drake::Vector2;
using drake::AutoDiffXd;
using drake::AutoDiffVecXd;

using nonlinear_pendulum::CalcPendulumDynamics;

NonlinearPendulumDynamicsConstraint::NonlinearPendulumDynamicsConstraint(
    int num_intervals, double mass) : solvers::NonlinearConstraint<AutoDiffXd>(
        6, 17, VectorXd::Zero(6), VectorXd::Zero(6)){
  num_intervals_ = static_cast<double>(num_intervals);
  m_ = mass;
}

void NonlinearPendulumDynamicsConstraint::EvaluateConstraint(
    const Eigen::Ref<const drake::VectorX<drake::AutoDiffXd>>& x,
    drake::VectorX<drake::AutoDiffXd>* y) const {
  const Vector6<AutoDiffXd>& x0 = x.head<6>();
  const Vector6<AutoDiffXd>& x1 = x.segment<6>(6);
  const Vector2<AutoDiffXd>& u0 = x.segment<2>(12);
  const Vector2<AutoDiffXd>& u1 = x.segment<2>(14);
  const AutoDiffXd dt = x(16) / num_intervals_;

  AutoDiffVecXd xdot0 = CalcPendulumDynamics(x0, u0, m_);
  AutoDiffVecXd xdot1 = CalcPendulumDynamics(x1, u1, m_);

  *y = x1 - x0 - 0.5 * (xdot0 + xdot1) * dt;
}

}