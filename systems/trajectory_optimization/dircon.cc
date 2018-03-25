#include "dircon.h"
#include "dircon_options.h"

#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {

using trajectories::PiecewisePolynomial;
using solvers::Binding;
using solvers::Constraint;
using solvers::MathematicalProgram;
using solvers::VectorXDecisionVariable;

DirconDynamicConstraint::DirconDynamicConstraint(const RigidBodyTree<double>& tree, DirconKinematicDataSet<AutoDiffXd>& constraints) :
  DirconDynamicConstraint(tree, constraints, tree.get_num_positions(), tree.get_num_velocities(), tree.get_num_actuators(), constraints.getNumConstraints()) {}

DirconDynamicConstraint::DirconDynamicConstraint(const RigidBodyTree<double>& tree, DirconKinematicDataSet<AutoDiffXd>& constraints,
                                                 int num_positions, int num_velocities, int num_inputs, int num_kinematic_constraints)
    : Constraint(num_positions + num_velocities, 1 + 2 *(num_positions+ num_velocities) + (2 * num_inputs) + (4 * num_kinematic_constraints),
                 Eigen::VectorXd::Zero(num_positions + num_velocities), Eigen::VectorXd::Zero(num_positions + num_velocities)),
      num_states_{num_positions+num_velocities}, num_inputs_{num_inputs}, num_kinematic_constraints_{num_kinematic_constraints},
      num_positions_{num_positions}, num_velocities_{num_velocities} {
  tree_ = &tree;
  constraints_ = &constraints;
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
  DRAKE_ASSERT(x.size() == 1 + (2 * num_states_) + (2 * num_inputs_) + 4*(num_kinematic_constraints_));

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

  constraints_->updateData(x0, u0, l0);
  AutoDiffVecXd xdot0 = constraints_->getXDot();
  const Eigen::MatrixXd dxdot0 = math::autoDiffToGradientMatrix(xdot0);

  constraints_->updateData(x1, u1, l1);
  AutoDiffVecXd xdot1 = constraints_->getXDot();
  const Eigen::MatrixXd dxdot1 = math::autoDiffToGradientMatrix(xdot1);
  // Cubic interpolation to get xcol and xdotcol.
  const AutoDiffVecXd xcol = 0.5 * (x0 + x1) + h / 8 * (xdot0 - xdot1);
  const AutoDiffVecXd xdotcol = -1.5 * (x0 - x1) / h - .25 * (xdot0 + xdot1);

  constraints_->updateData(xcol, 0.5 * (u0 + u1), vc);
  AutoDiffVecXd g = constraints_->getXDot();
  g.head(num_positions_) += constraints_->getJ().transpose()*vc;
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

DirconKinematicConstraint::DirconKinematicConstraint(const RigidBodyTree<double>& tree, DirconKinematicDataSet<AutoDiffXd>& constraints) :
  DirconKinematicConstraint(tree, constraints, tree.get_num_positions(), tree.get_num_velocities(), tree.get_num_actuators(), constraints.getNumConstraints()) {}

DirconKinematicConstraint::DirconKinematicConstraint(const RigidBodyTree<double>& tree, DirconKinematicDataSet<AutoDiffXd>& constraints,
                                                 int num_positions, int num_velocities, int num_inputs, int num_kinematic_constraints)
    : Constraint(3*num_kinematic_constraints, num_positions+ num_velocities + num_inputs + num_kinematic_constraints,
                 Eigen::VectorXd::Zero(3*num_kinematic_constraints), Eigen::VectorXd::Zero(3*num_kinematic_constraints)),
      num_states_{num_positions+num_velocities}, num_inputs_{num_inputs}, num_kinematic_constraints_{num_kinematic_constraints},
      num_positions_{num_positions}, num_velocities_{num_velocities} {
  tree_ = &tree;
  constraints_ = &constraints;
}


//mposa: what is this function actually doing?
void DirconKinematicConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x,
    Eigen::VectorXd& y) const {
  AutoDiffVecXd y_t;
  Eval(math::initializeAutoDiff(x), y_t);
  y = math::autoDiffToValueMatrix(y_t);
}

void DirconKinematicConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd& y) const {
  DRAKE_ASSERT(x.size() == num_states_ + num_inputs_ + num_kinematic_constraints_);

  // Extract our input variables:
  // h - current time (knot) value
  // x0, x1 state vector at time steps k, k+1
  // u0, u1 input vector at time steps k, k+1
  const auto state = x.segment(0, num_states_);
  const auto input = x.segment(num_states_, num_inputs_);
  const auto force = x.segment(num_states_ + num_inputs_, num_kinematic_constraints_);
  //TODO: properly add in relative constraint decision variables
  constraints_->updateData(state, input, force);
  y = AutoDiffVecXd(3*num_kinematic_constraints_);
  y << constraints_->getC(), constraints_->getCDot(), constraints_->getCDDot();
}

Dircon::Dircon(const RigidBodyTree<double>& tree, int num_time_samples, double minimum_timestep, double maximum_timestep,
    DirconKinematicDataSet<AutoDiffXd>& constraints, DirconOptions options)
    : MultipleShooting(tree.get_num_actuators(), tree.get_num_positions() + tree.get_num_velocities(), num_time_samples, minimum_timestep, maximum_timestep),
      num_kinematic_constraints_{constraints.getNumConstraints()},
      force_vars_(NewContinuousVariables(constraints.getNumConstraints() * num_time_samples, "lambda")),
      collocation_force_vars_(NewContinuousVariables(constraints.getNumConstraints() * (num_time_samples - 1), "lambda_c")),
      collocation_slack_vars_(NewContinuousVariables(constraints.getNumConstraints() * (num_time_samples - 1), "v_c"))
 {
  tree_ = &tree;
  constraints_ = &constraints;
  auto constraint = std::make_shared<DirconDynamicConstraint>(tree, constraints);

  DRAKE_ASSERT(static_cast<int>(constraint->num_constraints()) == num_states());

  // For N-1 timesteps, add a constraint which depends on the knot
  // value along with the state and input vectors at that knot and the
  // next.

  //TODO: To enable caching of constraint calculations, I probably need to make deep copies of constraints (and make another container
  // class that that has double the info for time i and i+1)
  for (int i = 0; i < N() - 1; i++) {
    AddConstraint(constraint,
                  {h_vars().segment<1>(i),
                   x_vars().segment(i * num_states(), num_states() * 2),
                   u_vars().segment(i * num_inputs(), num_inputs() * 2),
                   force_vars().segment(i * num_kinematic_constraints(), num_kinematic_constraints() * 2),
                   collocation_force_vars().segment(i * num_kinematic_constraints(), num_kinematic_constraints()),
                   collocation_slack_vars().segment(i * num_kinematic_constraints(), num_kinematic_constraints())});
  }

  //TODO: add decision variables related to relative constraints
  auto kinematic_constraint = std::make_shared<DirconKinematicConstraint>(tree, constraints);
  for (int i = 0; i < N(); i++) {
    AddConstraint(kinematic_constraint,
                  {x_vars().segment(i * num_states(), num_states()),
                   u_vars().segment(i * num_inputs(), num_inputs()),
                   force_vars().segment(i * num_kinematic_constraints(), num_kinematic_constraints())});
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


PiecewisePolynomial<double>
Dircon::ReconstructStateTrajectory()
    const {
  Eigen::VectorXd times = GetSampleTimes();
  std::vector<double> times_vec(N());
  std::vector<Eigen::MatrixXd> states(N());
  std::vector<Eigen::MatrixXd> inputs(N());
  std::vector<Eigen::MatrixXd> forces(N());
  std::vector<Eigen::MatrixXd> derivatives(N());

  for (int i = 0; i < N(); i++) {
    times_vec[i] = times(i);
    states[i] = GetSolution(state(i));
    inputs[i] = GetSolution(input(i));
    forces[i] = GetSolution(force(i));
    constraints_->updateData(states[i], inputs[i], forces[i]);

    derivatives[i] =   math::autoDiffToValueMatrix(constraints_->getXDot());//Do I need to make a copy here?
  }
  return PiecewisePolynomial<double>::Cubic(times_vec, states, derivatives);
}

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake