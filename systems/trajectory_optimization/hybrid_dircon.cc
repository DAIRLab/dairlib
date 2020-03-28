#include "hybrid_dircon.h"

#include <algorithm>  // std::max
#include <stdexcept>
#include <utility>
#include <vector>

#include "multibody/multibody_utils.h"
#include "drake/math/autodiff.h"
#include "drake/solvers/decision_variable.h"

namespace dairlib {
namespace systems {
namespace trajectory_optimization {

using drake::AutoDiffXd;
using drake::VectorX;
using drake::multibody::MultibodyPlant;
using drake::solvers::Binding;
using drake::solvers::Constraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::VectorXDecisionVariable;
using drake::symbolic::Expression;
using drake::systems::trajectory_optimization::MultipleShooting;
using drake::trajectories::PiecewisePolynomial;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

// HybridDircon constructor
template <typename T>
HybridDircon<T>::HybridDircon(const MultibodyPlant<T>& plant,
                              vector<int> num_time_samples,
                              vector<double> minimum_timestep,
                              vector<double> maximum_timestep,
                              vector<DirconKinematicDataSet<T>*> constraints,
                              vector<DirconOptions> options)
    : MultipleShooting(
          plant.num_actuators(), plant.num_positions() + plant.num_velocities(),
          std::accumulate(num_time_samples.begin(), num_time_samples.end(), 0) -
              num_time_samples.size() + 1,
          1e-8, 1e8),
      plant_(plant),
      constraints_(constraints),
      num_modes_(num_time_samples.size()),
      mode_lengths_(num_time_samples),
      v_post_impact_vars_(NewContinuousVariables(
          plant.num_velocities() * (num_time_samples.size() - 1), "v_p")) {
  DRAKE_ASSERT(minimum_timestep.size() == num_modes_);
  DRAKE_ASSERT(maximum_timestep.size() == num_modes_);
  DRAKE_ASSERT(constraints.size() == num_modes_);
  DRAKE_ASSERT(options.size() == num_modes_);

  bool is_quaternion = multibody::isQuaternion(plant);

  // Initialization is looped over the modes
  int counter = 0;
  for (int i = 0; i < num_modes_; i++) {
    mode_start_.push_back(counter);

    // set timestep bounds
    for (int j = 0; j < mode_lengths_[i] - 1; j++) {
      AddBoundingBoxConstraint(minimum_timestep[i], maximum_timestep[i],
                               timestep(mode_start_[i] + j));
    }
    for (int j = 0; j < mode_lengths_[i] - 2; j++) {
      // all timesteps must be equal
      AddLinearConstraint(timestep(mode_start_[i] + j) ==
                          timestep(mode_start_[i] + j + 1));
    }

    // initialize constraint lengths
    num_kinematic_constraints_wo_skipping_.push_back(
        constraints_[i]->countConstraintsWithoutSkipping());
    num_kinematic_constraints_.push_back(constraints_[i]->countConstraints());

    // initialize decision variables
    // force_vars_, collocation_force_vars_ and collocation_slack_vars_
    force_vars_.push_back(NewContinuousVariables(
        constraints_[i]->countConstraintsWithoutSkipping() *
            num_time_samples[i],
        "lambda[" + std::to_string(i) + "]"));
    collocation_force_vars_.push_back(NewContinuousVariables(
        constraints_[i]->countConstraintsWithoutSkipping() *
            (num_time_samples[i] - 1),
        "lambda_c[" + std::to_string(i) + "]"));
    collocation_slack_vars_.push_back(NewContinuousVariables(
        constraints_[i]->countConstraintsWithoutSkipping() *
            (num_time_samples[i] - 1),
        "v_c[" + std::to_string(i) + "]"));
    // quaternion_slack_vars_ (slack variables used to scale quaternion norm to
    // 1 in the dynamic constraints)
    if (is_quaternion) {
      quaternion_slack_vars_.push_back(NewContinuousVariables(
          num_time_samples[i] - 1, "gamma_" + std::to_string(i)));
    } else {
      quaternion_slack_vars_.push_back(
          NewContinuousVariables(0, "gamma_" + std::to_string(i)));
    }
    // offset_vars_
    offset_vars_.push_back(NewContinuousVariables(
        options[i].getNumRelative(), "offset[" + std::to_string(i) + "]"));
    // impulse_vars_
    if (i > 0) {
      impulse_vars_.push_back(NewContinuousVariables(
          constraints_[i]->countConstraintsWithoutSkipping(),
          "impulse[" + std::to_string(i) + "]"));
    }

    // For N-1 timesteps, add a constraint which depends on the knot
    // value along with the state and input vectors at that knot and the
    // next.

    // Adding quaternion norm constraint
    if (is_quaternion) {
      auto quat_norm_constraint =
          std::make_shared<QuaternionNormConstraint<T>>();
      // If the current mode is not the first mode, start with the first knot.
      // Otherwise, start with the second knot in order to avoid imposing the
      // same constraint twice.
      for (int j = (i == 0) ? 0 : 1; j < mode_lengths_[i]; j++) {
        AddConstraint(quat_norm_constraint, state_vars_by_mode(i, j).head(4));
      }
    }

    // Adding dynamic constraints
    auto dynamic_constraint = std::make_shared<DirconDynamicConstraint<T>>(
        plant_, *constraints_[i], is_quaternion);
    DRAKE_ASSERT(static_cast<int>(dynamic_constraint->num_constraints()) ==
                 num_states());
    dynamic_constraint->SetConstraintScaling(
        options[i].getDynConstraintScaling());
    for (int j = 0; j < mode_lengths_[i] - 1; j++) {
      int time_index = mode_start_[i] + j;
      AddConstraint(
          dynamic_constraint,
          {h_vars().segment(time_index, 1), state_vars_by_mode(i, j),
           state_vars_by_mode(i, j + 1),
           u_vars().segment(time_index * num_inputs(), num_inputs() * 2),
           force_vars(i).segment(j * num_kinematic_constraints_wo_skipping(i),
                                 num_kinematic_constraints_wo_skipping(i) * 2),
           collocation_force_vars(i).segment(
               j * num_kinematic_constraints_wo_skipping(i),
               num_kinematic_constraints_wo_skipping(i)),
           collocation_slack_vars(i).segment(
               j * num_kinematic_constraints_wo_skipping(i),
               num_kinematic_constraints_wo_skipping(i)),
           (is_quaternion) ? quaternion_slack_vars(i).segment(j, 1)
                           : quaternion_slack_vars(i).segment(0, 0)});
    }

    // Adding kinematic constraints (interior nodes of the mode)
    auto kinematic_constraint = std::make_shared<DirconKinematicConstraint<T>>(
        plant_, *constraints_[i], options[i].getConstraintsRelative());
    kinematic_constraint->SetConstraintScaling(
        options[i].getKinConstraintScaling());
    for (int j = 1; j < mode_lengths_[i] - 1; j++) {
      int time_index = mode_start_[i] + j;
      AddConstraint(
          kinematic_constraint,
          {state_vars_by_mode(i, j),
           u_vars().segment(time_index * num_inputs(), num_inputs()),
           force_vars(i).segment(j * num_kinematic_constraints_wo_skipping(i),
                                 num_kinematic_constraints_wo_skipping(i)),
           offset_vars(i)});
    }

    // Adding kinematic constraints (start node of the mode)
    auto kinematic_constraint_start =
        std::make_shared<DirconKinematicConstraint<T>>(
            plant_, *constraints_[i], options[i].getConstraintsRelative(),
            options[i].getStartType());
    kinematic_constraint_start->SetConstraintScaling(
        options[i].getKinConstraintScalingStart());
    AddConstraint(
        kinematic_constraint_start,
        {state_vars_by_mode(i, 0),
         u_vars().segment(mode_start_[i], num_inputs()),
         force_vars(i).segment(0, num_kinematic_constraints_wo_skipping(i)),
         offset_vars(i)});

    // Adding kinematic constraints (end node of the mode)
    // Only add the end constraint if the length inside the mode is greater
    // than 1. (The first and the last timestamp are the same, if the length
    // inside the mode is 1.)
    if (mode_lengths_[i] > 1) {
      auto kinematic_constraint_end =
          std::make_shared<DirconKinematicConstraint<T>>(
              plant_, *constraints_[i], options[i].getConstraintsRelative(),
              options[i].getEndType());
      kinematic_constraint_end->SetConstraintScaling(
          options[i].getKinConstraintScalingEnd());
      AddConstraint(
          kinematic_constraint_end,
          {state_vars_by_mode(i, mode_lengths_[i] - 1),
           u_vars().segment(
               (mode_start_[i] + mode_lengths_[i] - 1) * num_inputs(),
               num_inputs()),
           force_vars(i).segment((mode_lengths_[i] - 1) *
                                     num_kinematic_constraints_wo_skipping(i),
                                 num_kinematic_constraints_wo_skipping(i)),
           offset_vars(i)});
    }

    // Add constraints on force
    for (int l = 0; l < mode_lengths_[i]; l++) {
      int start_index = l * num_kinematic_constraints_wo_skipping(i);
      for (int j = 0; j < constraints_[i]->getNumConstraintObjects(); j++) {
        DirconKinematicData<T>* constraint_j =
            constraints_[i]->getConstraint(j);
        for (int k = 0; k < constraint_j->numForceConstraints(); k++) {
          AddConstraint(
              constraint_j->getForceConstraint(k),
              force_vars(i).segment(start_index, constraint_j->getLength()));
        }
        start_index += constraint_j->getLength();
      }
    }

    // Add force to cost function
    if (options[i].getForceCost() != 0) {
      auto A = options[i].getForceCost() *
               MatrixXd::Identity(num_kinematic_constraints_wo_skipping(i),
                                  num_kinematic_constraints_wo_skipping(i));
      auto b = MatrixXd::Zero(num_kinematic_constraints_wo_skipping(i), 1);
      for (int j = 0; j < mode_lengths_[i]; j++) {
        // Add || Ax - b ||^2
        AddL2NormCost(A, b, force(i, j));
      }
    }

    if (i > 0) {
      if (constraints_[i]->countConstraintsWithoutSkipping() > 0) {
        auto impact_constraint = std::make_shared<DirconImpactConstraint<T>>(
            plant_, *constraints_[i]);
        impact_constraint->SetConstraintScaling(
            options[i].getImpConstraintScaling());
        AddConstraint(impact_constraint,
                      {state_vars_by_mode(i - 1, mode_lengths_[i - 1] - 1),
                       impulse_vars(i - 1), v_post_impact_vars_by_mode(i - 1)});

        // Add constraints on impulse variables
        int start_index = 0;
        for (int j = 0; j < constraints_[i]->getNumConstraintObjects(); j++) {
          DirconKinematicData<T>* constraint_j =
              constraints_[i]->getConstraint(j);
          for (int k = 0; k < constraint_j->numForceConstraints(); k++) {
            AddConstraint(constraint_j->getForceConstraint(k),
                          impulse_vars(i - 1).segment(
                              start_index, constraint_j->getLength()));
          }
          start_index += constraint_j->getLength();
        }

      } else {
        auto x_vars_prev = state_vars_by_mode(i - 1, mode_lengths_[i - 1] - 1);
        AddConstraint(v_post_impact_vars_by_mode(i - 1) ==
                      x_vars_prev.tail(plant.num_velocities()));
      }
    }

    counter += mode_lengths_[i] - 1;
  }
}

template <typename T>
const Eigen::VectorBlock<const VectorXDecisionVariable>
HybridDircon<T>::v_post_impact_vars_by_mode(int mode) const {
  return v_post_impact_vars_.segment(mode * plant_.num_velocities(),
                                     plant_.num_velocities());
}

template <typename T>
VectorX<Expression> HybridDircon<T>::SubstitutePlaceholderVariables(
    const VectorX<Expression>& f, int interval_index) const {
  VectorX<Expression> ret(f.size());
  for (int i = 0; i < f.size(); i++) {
    ret(i) =
        MultipleShooting::SubstitutePlaceholderVariables(f(i), interval_index);
  }
  return ret;
}

// Eigen::VectorBlock<const VectorXDecisionVariable>
// HybridDircon<T>::state_vars_by_mode(int mode, int time_index)  {
template <typename T>
VectorXDecisionVariable HybridDircon<T>::state_vars_by_mode(
    int mode, int time_index) const {
  if (time_index == 0 && mode > 0) {
    VectorXDecisionVariable ret(num_states());
    ret << x_vars().segment((mode_start_[mode] + time_index) * num_states(),
                            plant_.num_positions()),
        v_post_impact_vars_by_mode(mode - 1);
    return ret;
    // return Eigen::VectorBlock<const VectorXDecisionVariable>(ret, 0,
    // num_states());
  } else {
    VectorXDecisionVariable ret(num_states());
    return x_vars().segment((mode_start_[mode] + time_index) * num_states(),
                            num_states());
    // std::cout << Eigen::VectorBlock<VectorXDecisionVariable>(ret, 0,
    // num_states())  << std::endl; return
    // Eigen::VectorBlock<VectorXDecisionVariable>(ret, 0, num_states());
  }
}

// TODO: need to configure this to handle the hybrid discontinuities properly
template <typename T>
void HybridDircon<T>::DoAddRunningCost(const drake::symbolic::Expression& g) {
  // Trapezoidal integration:
  //    sum_{i=0...N-2} h_i/2.0 * (g_i + g_{i+1}), or
  // g_0*h_0/2.0 + [sum_{i=1...N-2} g_i*(h_{i-1} + h_i)/2.0] +
  // g_{N-1}*h_{N-2}/2.0.

  // Here, we add the cost using symbolic expression. The expression is a
  // polynomial of degree 3 which Drake can handle, although the
  // documentation says it only supports up to second order.
  AddCost(MultipleShooting::SubstitutePlaceholderVariables(g, 0) * h_vars()(0) /
          2);
  for (int i = 1; i <= N() - 2; i++) {
    AddCost(MultipleShooting::SubstitutePlaceholderVariables(g, i) *
            (h_vars()(i - 1) + h_vars()(i)) / 2);
  }
  AddCost(MultipleShooting::SubstitutePlaceholderVariables(g, N() - 1) *
          h_vars()(N() - 2) / 2);
}

template <typename T>
PiecewisePolynomial<double> HybridDircon<T>::ReconstructInputTrajectory(
    const MathematicalProgramResult& result) const {
  Eigen::VectorXd times = GetSampleTimes(result);
  vector<double> times_vec(N());
  vector<Eigen::MatrixXd> inputs(N());
  for (int i = 0; i < N(); i++) {
    times_vec[i] = times(i);
    inputs[i] = result.GetSolution(input(i));
  }
  return PiecewisePolynomial<double>::FirstOrderHold(times_vec, inputs);
}

// TODO(mposa)
// need to configure this to handle the hybrid discontinuities properly
template <typename T>
PiecewisePolynomial<double> HybridDircon<T>::ReconstructStateTrajectory(
    const MathematicalProgramResult& result) const {
  VectorXd times_all(GetSampleTimes(result));
  VectorXd times(N() + num_modes_ - 1);

  MatrixXd states(num_states(), N() + num_modes_ - 1);
  MatrixXd inputs(num_inputs(), N() + num_modes_ - 1);
  MatrixXd derivatives(num_states(), N() + num_modes_ - 1);

  for (int i = 0; i < num_modes_; i++) {
    for (int j = 0; j < mode_lengths_[i]; j++) {
      int k = mode_start_[i] + j + i;
      int k_data = mode_start_[i] + j;
      times(k) = times_all(k_data);

      // False timestep to match velocities
      if (i > 0 && j == 0) {
        times(k) += +1e-6;
      }
      VectorX<T> xk = result.GetSolution(state_vars_by_mode(i, j));
      VectorX<T> uk = result.GetSolution(input(k_data));
      states.col(k) = drake::math::DiscardGradient(xk);
      inputs.col(k) = drake::math::DiscardGradient(uk);
      auto context = multibody::createContext(plant_, xk, uk);
      constraints_[i]->updateData(*context, result.GetSolution(force(i, j)));
      derivatives.col(k) =
          drake::math::DiscardGradient(constraints_[i]->getXDot());
    }
  }
  return PiecewisePolynomial<double>::Cubic(times, states, derivatives);
}

template <typename T>
void HybridDircon<T>::SetInitialForceTrajectory(
    int mode, const PiecewisePolynomial<double>& traj_init_l,
    const PiecewisePolynomial<double>& traj_init_lc,
    const PiecewisePolynomial<double>& traj_init_vc) {
  double start_time = 0;
  double h;
  if (timesteps_are_decision_variables())
    h = GetInitialGuess(h_vars()[0]);
  else
    h = fixed_timestep();

  VectorXd guess_force(force_vars_[mode].size());
  if (traj_init_l.empty()) {
    guess_force.fill(0);  // Start with 0
  } else {
    for (int i = 0; i < mode_lengths_[mode]; ++i) {
      guess_force.segment(num_kinematic_constraints_wo_skipping_[mode] * i,
                          num_kinematic_constraints_wo_skipping_[mode]) =
          traj_init_l.value(start_time + i * h);
    }
  }
  SetInitialGuess(force_vars_[mode], guess_force);

  VectorXd guess_collocation_force(collocation_force_vars_[mode].size());
  if (traj_init_lc.empty()) {
    guess_collocation_force.fill(0);  // Start with 0
  } else {
    for (int i = 0; i < mode_lengths_[mode] - 1; ++i) {
      guess_collocation_force.segment(
          num_kinematic_constraints_wo_skipping_[mode] * i,
          num_kinematic_constraints_wo_skipping_[mode]) =
          traj_init_lc.value(start_time + (i + 0.5) * h);
    }
  }
  SetInitialGuess(collocation_force_vars_[mode], guess_collocation_force);

  VectorXd guess_collocation_slack(collocation_slack_vars_[mode].size());
  if (traj_init_vc.empty()) {
    guess_collocation_slack.fill(0);  // Start with 0
  } else {
    for (int i = 0; i < mode_lengths_[mode] - 1; ++i) {
      guess_collocation_slack.segment(
          num_kinematic_constraints_wo_skipping_[mode] * i,
          num_kinematic_constraints_wo_skipping_[mode]) =
          traj_init_vc.value(start_time + (i + 0.5) * h);
    }
  }
  // call superclass method
  SetInitialGuess(collocation_slack_vars_[mode], guess_collocation_slack);
}

template <typename T>
void HybridDircon<T>::ScaleTimeVariables(double scale) {
  for (int i = 0; i < h_vars().size(); i++) {
    this->SetVariableScaling(h_vars()(i), scale);
  }
}
template <typename T>
void HybridDircon<T>::ScaleQuaternionSlackVariables(double scale) {
  DRAKE_DEMAND(multibody::isQuaternion(plant_));
  for (size_t mode = 0; mode < mode_lengths_.size(); mode++) {
    for (int j = 0; j < mode_lengths_[mode] - 1; j++) {
      this->SetVariableScaling(quaternion_slack_vars_[mode](j), scale);
    }
  }
}
template <typename T>
void HybridDircon<T>::ScaleStateVariable(int idx, double scale) {
  int n_x = this->num_states();
  DRAKE_DEMAND((0 <= idx) && (idx < n_x));

  // x_vars_ in MathematicalProgram
  for (int j_knot = 0; j_knot < N(); j_knot++) {
    auto vars = this->state(j_knot);
    this->SetVariableScaling(vars(idx), scale);
  }

  // v_post_impact_vars_
  int n_q = plant_.num_positions();
  if ((idx >= n_q) && (num_modes_ > 1)) {
    idx -= n_q;
    for (int mode = 0; mode < num_modes_ - 1; mode++) {
      auto vars = v_post_impact_vars_by_mode(mode);
      this->SetVariableScaling(vars(idx), scale);
    }
  }
}
template <typename T>
void HybridDircon<T>::ScaleInputVariable(int idx, double scale) {
  int n_u = this->num_inputs();
  DRAKE_DEMAND((0 <= idx) && (idx < n_u));

  // u_vars_ in MathematicalProgram
  for (int j_knot = 0; j_knot < N(); j_knot++) {
    auto vars = this->input(j_knot);
    this->SetVariableScaling(vars(idx), scale);
  }
}
template <typename T>
void HybridDircon<T>::ScaleForceVariable(int mode, int idx, double scale) {
  DRAKE_DEMAND((0 <= mode) && (mode < num_modes_));
  int n_lambda = num_kinematic_constraints_wo_skipping_[mode];
  DRAKE_DEMAND((0 <= idx) && (idx < n_lambda));

  // Force at knot points
  auto vars = force_vars(mode);
  for (int j = 0; j < mode_lengths_[mode]; j++) {
    this->SetVariableScaling(vars(n_lambda * j + idx), scale);
  }
  // Force at collocation pints
  auto vars_2 = collocation_force_vars(mode);
  for (int j = 0; j < mode_lengths_[mode] - 1; j++) {
    this->SetVariableScaling(vars_2(n_lambda * j + idx), scale);
  }
}
template <typename T>
void HybridDircon<T>::ScaleImpulseVariable(int mode, int idx, double scale) {
  DRAKE_DEMAND((0 <= mode) && (mode < num_modes_ - 1));
  int n_impulse = constraints_[mode]->countConstraintsWithoutSkipping();
  DRAKE_DEMAND((0 <= idx) && (idx < n_impulse));

  auto vars = impulse_vars(mode);
  this->SetVariableScaling(vars(idx), scale);
}
template <typename T>
void HybridDircon<T>::ScaleKinConstraintSlackVariable(int mode, int idx,
                                                      double scale) {
  DRAKE_DEMAND((0 <= mode) && (mode < num_modes_ - 1));
  int n_lambda = num_kinematic_constraints_wo_skipping_[mode];
  DRAKE_DEMAND(idx < n_lambda);

  auto vars = collocation_slack_vars(mode);
  for (int j = 0; j < mode_lengths_[mode] - 1; j++) {
    this->SetVariableScaling(vars(n_lambda * j + idx), scale);
  }
}

template <typename T>
void HybridDircon<T>::ScaleStateVariables(std::vector<int> idx_list,
                                          double scale) {
  for (const auto& idx : idx_list) {
    ScaleStateVariable(idx, scale);
  }
}
template <typename T>
void HybridDircon<T>::ScaleInputVariables(std::vector<int> idx_list,
                                          double scale) {
  for (const auto& idx : idx_list) {
    ScaleInputVariable(idx, scale);
  }
}
template <typename T>
void HybridDircon<T>::ScaleForceVariables(int mode, std::vector<int> idx_list,
                                          double scale) {
  for (const auto& idx : idx_list) {
    ScaleForceVariable(mode, idx, scale);
  }
}
template <typename T>
void HybridDircon<T>::ScaleImpulseVariables(int mode, std::vector<int> idx_list,
                                            double scale) {
  for (const auto& idx : idx_list) {
    ScaleImpulseVariable(mode, idx, scale);
  }
}
template <typename T>
void HybridDircon<T>::ScaleKinConstraintSlackVariables(
    int mode, std::vector<int> idx_list, double scale) {
  for (const auto& idx : idx_list) {
    ScaleKinConstraintSlackVariable(mode, idx, scale);
  }
}

template class HybridDircon<double>;
template class HybridDircon<AutoDiffXd>;

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace dairlib
