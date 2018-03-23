#include "dircon.h"
#include "dircon_options.h"

#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "dircon_position_constraint.h"
#include "dircon_position_constraint.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {

using trajectories::PiecewisePolynomial;
using solvers::Binding;
using solvers::Constraint;
using solvers::MathematicalProgram;
using solvers::VectorXDecisionVariable;

DirconDynamicConstraint::DirconDynamicConstraint(const RigidBodyTree<double>& tree, DirconKinematicConstraintSet<AutoDiffXd>& constraints) :
  DirconDynamicConstraint(tree, constraints, tree.get_num_positions(), tree.get_num_velocities(), tree.get_num_actuators(), constraints.getNumConstraints()) {}

/*
DirconDynamicConstraint::DirconDynamicConstraint(const RigidBodyTree<double>& tree, DirconKinematicConstraintSet<AutoDiffXd>& constraints)
    : Constraint(tree.get_num_positions() + tree.get_num_velocities(), 1 + 2 *(tree.get_num_positions()+ tree.get_num_velocities()) + (2 * tree.get_num_actuators()) + (4 * constraints.getNumConstraints()),
                 Eigen::VectorXd::Zero(tree.get_num_positions()+ tree.get_num_velocities()), Eigen::VectorXd::Zero(tree.get_num_positions()+ tree.get_num_velocities())),
      num_states_{tree.get_num_positions()+tree.get_num_velocities()}, num_inputs_{tree.get_num_actuators()}, num_kinematic_constraints_{constraints.getNumConstraints()},
      num_positions_{tree.get_num_positions()}, num_velocities_{tree.get_num_velocities()} {
  tree_ = &tree;
  constraints_ = &constraints;
}*/

DirconDynamicConstraint::DirconDynamicConstraint(const RigidBodyTree<double>& tree, DirconKinematicConstraintSet<AutoDiffXd>& constraints,
                                                 int num_positions, int num_velocities, int num_inputs, int num_kinematic_constraints)
    : Constraint(num_positions + num_velocities, 1 + 2 *(num_positions+ num_velocities) + (2 * num_inputs) + (4 * num_kinematic_constraints),
                 Eigen::VectorXd::Zero(num_positions+ num_velocities), Eigen::VectorXd::Zero(num_positions+ num_velocities)),
      num_states_{num_positions+num_velocities}, num_inputs_{num_inputs}, num_kinematic_constraints_{num_kinematic_constraints},
      num_positions_{num_positions}, num_velocities_{num_velocities} {
  tree_ = &tree;
  constraints_ = &constraints;
}

void DirconDynamicConstraint::ConstrainedDynamics(const AutoDiffVecXd& state, const AutoDiffVecXd& input, const AutoDiffVecXd& forces,
                                            AutoDiffVecXd* xdot) const {
  //TODO: this and collocation_constrained_dynamics should share code rather than being copy-paste
  AutoDiffVecXd q = state.head(num_positions_);
  AutoDiffVecXd v = state.tail(num_velocities_);

  KinematicsCache<AutoDiffXd> kinsol = tree_->doKinematics(q,v,true);

  constraints_->updateConstraints(kinsol);

  const MatrixX<AutoDiffXd> M = tree_->massMatrix(kinsol);
  const drake::eigen_aligned_std_unordered_map<RigidBody<double> const*, drake::WrenchVector<AutoDiffXd>> no_external_wrenches;
  const MatrixX<AutoDiffXd> J_transpose = constraints_->getJ().transpose();

  // right_hand_side is the right hand side of the system's equations:
  // M*vdot -J^T*f = right_hand_side.
  AutoDiffVecXd right_hand_side = -tree_->dynamicsBiasTerm(kinsol, no_external_wrenches) + tree_->B * input + J_transpose*forces;

  AutoDiffVecXd vdot = M.llt().solve(right_hand_side);

  *xdot << tree_->transformVelocityToQDot(kinsol, v), vdot;
}

void DirconDynamicConstraint::CollocationConstrainedDynamics(const AutoDiffVecXd& state, const AutoDiffVecXd& input, 
        const AutoDiffVecXd& forces, const AutoDiffVecXd& position_slack, AutoDiffVecXd* xdot) const {
  AutoDiffVecXd q = state.head(num_positions_);
  AutoDiffVecXd v = state.tail(num_velocities_);

  KinematicsCache<AutoDiffXd> kinsol = tree_->doKinematics(q,v,true);

  constraints_->updateConstraints(kinsol);

  const MatrixX<AutoDiffXd> M = tree_->massMatrix(kinsol);
  const drake::eigen_aligned_std_unordered_map<RigidBody<double> const*, drake::WrenchVector<AutoDiffXd>> no_external_wrenches;
  const MatrixX<AutoDiffXd> J_transpose = constraints_->getJ().transpose();

  // right_hand_side is the right hand side of the system's equations:
  // M*vdot -J^T*f = right_hand_side.
  AutoDiffVecXd right_hand_side = -tree_->dynamicsBiasTerm(kinsol, no_external_wrenches) + tree_->B * input + J_transpose*forces;

  AutoDiffVecXd vdot = M.llt().solve(right_hand_side);

  *xdot << tree_->transformVelocityToQDot(kinsol, v) + J_transpose*position_slack, vdot;
}


//mposa: what is this function actually doing?
void DirconDynamicConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x,
    Eigen::VectorXd& y) const {
  AutoDiffVecXd y_t;
  Eval(math::initializeAutoDiff(x), y_t);
  y = math::autoDiffToValueMatrix(y_t);
}

// The format of the input to the eval() function is the
// tuple { timestep, state 0, state 1, input 0, input 1, force 0, force 1},
// which has a total length of 1 + 2*num_states + 2*num_inputs + dim*num_constraints.
void DirconDynamicConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd& y) const {
  DRAKE_ASSERT(x.size() == 1 + (2 * num_states_) + (2 * num_inputs_));

  // Extract our input variables:
  // h - current time (knot) value
  // x0, x1 state vector at time steps k, k+1
  // u0, u1 input vector at time steps k, k+1
  const AutoDiffXd h = x(0);
  const auto x0 = x.segment(1, num_states_);
  const auto x1 = x.segment(1 + num_states_, num_states_);
  const auto u0 = x.segment(1 + (2 * num_states_), num_inputs_);
  const auto u1 = x.segment(1 + (2 * num_states_) + num_inputs_, num_inputs_);
  const auto l0 = x.segment(1 + (2 * num_states_ + num_inputs_), num_kinematic_constraints_);
  const auto l1 = x.segment(1 + (2 * num_states_ + num_inputs_) + num_kinematic_constraints_, num_kinematic_constraints_);
  const auto lc = x.segment(1 + (2 * num_states_ + num_inputs_) + 2*num_kinematic_constraints_, num_kinematic_constraints_);
  const auto vc = x.segment(1 + (2 * num_states_ + num_inputs_) + 3*num_kinematic_constraints_, num_kinematic_constraints_);

  // TODO(sam.creasey): Use caching (when it arrives) to avoid recomputing
  // the dynamics.  Currently the dynamics evaluated here as {u1,x1} are
  // recomputed in the next constraint as {u0,x0}.
  AutoDiffVecXd xdot0;
  ConstrainedDynamics(x0, u0, l0, &xdot0);
  const Eigen::MatrixXd dxdot0 = math::autoDiffToGradientMatrix(xdot0);

  AutoDiffVecXd xdot1;
  ConstrainedDynamics(x1, u1, l1, &xdot1);
  const Eigen::MatrixXd dxdot1 = math::autoDiffToGradientMatrix(xdot1);

  // Cubic interpolation to get xcol and xdotcol.
  const AutoDiffVecXd xcol = 0.5 * (x0 + x1) + h / 8 * (xdot0 - xdot1);
  const AutoDiffVecXd xdotcol = -1.5 * (x0 - x1) / h - .25 * (xdot0 + xdot1);

  AutoDiffVecXd g;
  CollocationConstrainedDynamics(xcol, 0.5 * (u0 + u1), lc, vc, &g);
  y = xdotcol - g;
}

Binding<Constraint> AddDirconConstraint(
    std::shared_ptr<DirconDynamicConstraint> constraint,
    const Eigen::Ref<const VectorXDecisionVariable>& timestep,
    const Eigen::Ref<const VectorXDecisionVariable>& state,
    const Eigen::Ref<const VectorXDecisionVariable>& next_state,
    const Eigen::Ref<const VectorXDecisionVariable>& input,
    const Eigen::Ref<const VectorXDecisionVariable>& next_input,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& force,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& next_force,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& collocation_force,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& collocation_position_slack,
    MathematicalProgram* prog) {
  DRAKE_DEMAND(timestep.size() == 1);
  DRAKE_DEMAND(state.size() == constraint->num_states());
  DRAKE_DEMAND(next_state.size() == constraint->num_states());
  DRAKE_DEMAND(input.size() == constraint->num_inputs());
  DRAKE_DEMAND(next_input.size() == constraint->num_inputs());
  DRAKE_DEMAND(force.size() == constraint->num_kinematic_constraints());
  DRAKE_DEMAND(next_force.size() == constraint->num_kinematic_constraints());
  DRAKE_DEMAND(collocation_force.size() == constraint->num_kinematic_constraints());
  DRAKE_DEMAND(collocation_position_slack.size() == constraint->num_kinematic_constraints());
  return prog->AddConstraint(constraint,
                             {timestep, state, next_state, input, next_input, force,
                              next_force, collocation_force, collocation_position_slack});
}

//Dircon::Dircon(const RigidBodyTree<double>& tree, int num_time_samples, double minimum_timestep, double maximum_timestep,
//    DirconKinematicConstraintSet<AutoDiffXd>& constraints, DirconOptions options)
Dircon::Dircon(const RigidBodyTree<double>& tree, int num_time_samples, double minimum_timestep, double maximum_timestep,
    DirconKinematicConstraintSet<AutoDiffXd>& constraints, DirconOptions options)
    : MultipleShooting(tree.get_num_actuators(), tree.get_num_positions() + tree.get_num_velocities(), num_time_samples, minimum_timestep, maximum_timestep) {

      //DirconDynamicConstraint constraint;
//      auto constraint = DirconPositionConstraint<double>(tree, 0, Eigen::Vector3d::Zero());
      //DirconDynamicConstraint c = DirconDynamicConstraint(tree,constraints);
  //int x;
  // Add the dynamic constraints.
  //DirconDynamicConstraint(tree,constraints);
  //constraint.getJ();

  auto constraint = std::make_shared<DirconDynamicConstraint>(tree, constraints);

  DRAKE_ASSERT(static_cast<int>(constraint->num_constraints()) == num_states());

  // For N-1 timesteps, add a constraint which depends on the knot
  // value along with the state and input vectors at that knot and the
  // next.

  //TODO: declare all of the collocation variables and add them here
  for (int i = 0; i < N() - 1; i++) {
    AddConstraint(constraint,
                  {h_vars().segment<1>(i),
                   x_vars().segment(i * num_states(), num_states() * 2),
                   u_vars().segment(i * num_inputs(), num_inputs() * 2)});
  }
}

void Dircon::DoAddRunningCost(const symbolic::Expression& g) {
  // Trapezoidal integration:
  //    sum_{i=0...N-2} h_i/2.0 * (g_i + g_{i+1}), or
  // g_0*h_0/2.0 + [sum_{i=1...N-2} g_i*(h_{i-1} + h_i)/2.0] +
  // g_{N-1}*h_{N-2}/2.0.

  AddCost(0.5 * SubstitutePlaceholderVariables(g * h_vars()(0) / 2, 0));
  for (int i = 1; i < N() - 2; i++) {
    AddCost(SubstitutePlaceholderVariables(
        g * (h_vars()(i - 1) + h_vars()(i)) / 2, i));
  }
  AddCost(0.5 *
          SubstitutePlaceholderVariables(g * h_vars()(N() - 2) / 2, N() - 1));
}

PiecewisePolynomial<double>
Dircon::ReconstructInputTrajectory()
    const {
  Eigen::VectorXd times = GetSampleTimes();
  std::vector<double> times_vec(N());
  std::vector<Eigen::MatrixXd> inputs(N());

  for (int i = 0; i < N(); i++) {
    times_vec[i] = times(i);
    inputs[i] = GetSolution(input(i));
  }
  return PiecewisePolynomial<double>::FirstOrderHold(times_vec, inputs);
}
/*
PiecewisePolynomial<double>
Dircon::ReconstructStateTrajectory()
    const {
  Eigen::VectorXd times = GetSampleTimes();
  std::vector<double> times_vec(N());
  std::vector<Eigen::MatrixXd> states(N());
  std::vector<Eigen::MatrixXd> derivatives(N());

  for (int i = 0; i < N(); i++) {
    times_vec[i] = times(i);
    states[i] = GetSolution(state(i));
    input_port_value_->GetMutableVectorData<double>()->SetFromVector(
        GetSolution(input(i)));
    context_->get_mutable_continuous_state().SetFromVector(states[i]);
    system_->CalcTimeDerivatives(*context_, continuous_state_.get());
    derivatives[i] = continuous_state_->CopyToVector();
  }
  return PiecewisePolynomial<double>::Cubic(times_vec, states, derivatives);
}*/

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake