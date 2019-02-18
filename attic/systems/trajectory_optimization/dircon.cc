#include "dircon.h"

#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

namespace dairlib {
namespace systems {
namespace trajectory_optimization {

using drake::solvers::Binding;
using drake::solvers::Constraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::VectorXDecisionVariable;
using drake::trajectories::PiecewisePolynomial;
using drake::systems::trajectory_optimization::MultipleShooting;
using drake::AutoDiffXd;
using Eigen::VectorXd;
using Eigen::MatrixXd;

template <typename T>
Dircon<T>::Dircon(const RigidBodyTree<double>& tree, int num_time_samples, double minimum_timestep, double maximum_timestep,
    DirconKinematicDataSet<T>& constraints, DirconOptions options)
    : MultipleShooting(tree.get_num_actuators(), tree.get_num_positions() + tree.get_num_velocities(), num_time_samples, minimum_timestep, maximum_timestep),
      num_kinematic_constraints_{constraints.countConstraints()},
      force_vars_(NewContinuousVariables(constraints.countConstraints() * num_time_samples, "lambda")),
      collocation_force_vars_(NewContinuousVariables(constraints.countConstraints() * (num_time_samples - 1), "lambda_c")),
      collocation_slack_vars_(NewContinuousVariables(constraints.countConstraints() * (num_time_samples - 1), "v_c")),
      offset_vars_(NewContinuousVariables(options.getNumRelative(), "offset")) {
  tree_ = &tree;
  constraints_ = &constraints;
  auto constraint = std::make_shared<DirconDynamicConstraint<T>>(tree, constraints);

  DRAKE_ASSERT(static_cast<int>(constraint->num_constraints()) == num_states());

  // For N-1 timesteps, add a constraint which depends on the knot
  // value along with the state and input vectors at that knot and the
  // next.

  //TODO: To enable caching of constraint calculations, I probably need to make deep copies of constraints (and make another container
  // class that that has double the info for time i and i+1)

  //Adding dynamic constraints
  for (int i = 0; i < N() - 1; i++) {
    AddConstraint(constraint,
                  {h_vars().segment(i,1),
                   x_vars().segment(i * num_states(), num_states() * 2),
                   u_vars().segment(i * num_inputs(), num_inputs() * 2),
                   force_vars().segment(i * num_kinematic_constraints(), num_kinematic_constraints() * 2),
                   collocation_force_vars().segment(i * num_kinematic_constraints(), num_kinematic_constraints()),
                   collocation_slack_vars().segment(i * num_kinematic_constraints(), num_kinematic_constraints())});
  }

  //Adding kinematic constraints
  auto kinematic_constraint = std::make_shared<DirconKinematicConstraint<T>>(tree, constraints, options.getConstraintsRelative());
  for (int i = 1; i < N()-1; i++) {
    AddConstraint(kinematic_constraint,
                  {x_vars().segment(i * num_states(), num_states()),
                   u_vars().segment(i * num_inputs(), num_inputs()),
                   force_vars().segment(i * num_kinematic_constraints(), num_kinematic_constraints()),
                   offset_vars()});
  }

  //special case i=0 and i=N based on options
  auto kinematic_constraint_start = std::make_shared<DirconKinematicConstraint<T>>(tree, constraints, options.getConstraintsRelative(), options.getStartType());
  AddConstraint(kinematic_constraint_start,
                {x_vars().segment(0, num_states()),
                 u_vars().segment(0, num_inputs()),
                 force_vars().segment(0, num_kinematic_constraints()),
                 offset_vars()});


  auto kinematic_constraint_end = std::make_shared<DirconKinematicConstraint<T>>(tree, constraints, options.getConstraintsRelative(), options.getEndType());
  AddConstraint(kinematic_constraint_end,
                {x_vars().segment((N()-1) * num_states(), num_states()),
                 u_vars().segment((N()-1) * num_inputs(), num_inputs()),
                 force_vars().segment((N()-1) * num_kinematic_constraints(), num_kinematic_constraints()),
                 offset_vars()});

  //Add constraints on force variables
  for (int i = 0; i < N() - 1; i++) {
    int start_index = i*num_kinematic_constraints();
    for (int j = 0; j < constraints_->getNumConstraintObjects(); j++) {
      DirconKinematicData<T>* constraint_j = constraints_->getConstraint(j);
      start_index += constraint_j->getLength();
      for (int k = 0; k < constraint_j->numForceConstraints(); k++) {
        AddConstraint(constraint_j->getForceConstraint(k), force_vars().segment(start_index, constraint_j->getLength()));
      }
    }
  }

  //Force cost option
  if (options.getForceCost() != 0) {
    auto A = options.getForceCost()*MatrixXd::Identity(num_kinematic_constraints(),num_kinematic_constraints());
    auto b = MatrixXd::Zero(num_kinematic_constraints(),1);
    for (int i=0; i < N(); i++) {
      AddL2NormCost(A,b,force(i));
    }
  }
}

template <typename T>
void Dircon<T>::DoAddRunningCost(const drake::symbolic::Expression& g) {
  // Trapezoidal integration:
  //    sum_{i=0...N-2} h_i/2.0 * (g_i + g_{i+1}), or
  // g_0*h_0/2.0 + [sum_{i=1...N-2} g_i*(h_{i-1} + h_i)/2.0] +
  // g_{N-1}*h_{N-2}/2.0.

  AddCost(SubstitutePlaceholderVariables(g * h_vars()(0) / 2, 0));
  for (int i = 1; i < N() - 1; i++) {
    AddCost(SubstitutePlaceholderVariables(
        g * (h_vars()(i - 1) + h_vars()(i)) / 2, i));
  }
  AddCost(
          SubstitutePlaceholderVariables(g * h_vars()(N() - 2) / 2, N() - 1));
}

template <typename T>
PiecewisePolynomial<double> Dircon<T>::ReconstructInputTrajectory()
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

template <typename T>
PiecewisePolynomial<double> Dircon<T>::ReconstructStateTrajectory()
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

    derivatives[i] = drake::math::DiscardGradient(constraints_->getXDot());
  }
  return PiecewisePolynomial<double>::Cubic(times_vec, states, derivatives);
}

template <typename T>
void Dircon<T>::SetInitialTrajectory(
    const PiecewisePolynomial<double>& traj_init_u,
    const PiecewisePolynomial<double>& traj_init_x,
    const PiecewisePolynomial<double>& traj_init_l,
    const PiecewisePolynomial<double>& traj_init_lc,
    const PiecewisePolynomial<double>& traj_init_vc) {
  MultipleShooting::SetInitialTrajectory(traj_init_u,traj_init_x);
  double start_time = 0;
  double h;
  if (timesteps_are_decision_variables())
    h = GetInitialGuess(h_vars()[0]);
  else
    h = fixed_timestep();

  VectorXd guess_force(force_vars_.size());
  if (traj_init_l.empty()) {
    guess_force.fill(0);  // Start with 0
  } else {
    for (int i = 0; i < N(); ++i) {
      guess_force.segment(num_kinematic_constraints_ * i, num_kinematic_constraints_) =
          traj_init_l.value(start_time + i * h);
    }
  }
  SetInitialGuess(force_vars_, guess_force);

  VectorXd guess_collocation_force(collocation_force_vars_.size());
  if (traj_init_lc.empty()) {
    guess_collocation_force.fill(0);  // Start with 0
  } else {
    for (int i = 0; i < N()-1; ++i) {
      guess_collocation_force.segment(num_kinematic_constraints_ * i, num_kinematic_constraints_) =
          traj_init_lc.value(start_time + (i + 0.5) * h);
    }
  }
  SetInitialGuess(collocation_force_vars_, guess_collocation_force);

  VectorXd guess_collocation_slack(collocation_slack_vars_.size());
  if (traj_init_vc.empty()) {
    guess_collocation_slack.fill(0);  // Start with 0
  } else {
    for (int i = 0; i < N()-1; ++i) {
      guess_collocation_slack.segment(num_kinematic_constraints_ * i, num_kinematic_constraints_) =
          traj_init_vc.value(start_time + (i + 0.5) * h);
    }
  }
  SetInitialGuess(collocation_slack_vars_, guess_collocation_slack); //call superclass method
}

template class Dircon<double>;
template class Dircon<AutoDiffXd>;

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace dairlib
