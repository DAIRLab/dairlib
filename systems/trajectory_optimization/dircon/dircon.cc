#include "systems/trajectory_optimization/dircon/dircon.h"

#include "multibody/kinematic/kinematic_constraints.h"
#include "multibody/multibody_utils.h"
#include "systems/trajectory_optimization/dircon/dircon_opt_constraints.h"

namespace dairlib {
namespace systems {
namespace trajectory_optimization {

using drake::VectorX;
using drake::multibody::MultibodyPlant;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::MatrixXDecisionVariable;
using drake::solvers::VectorXDecisionVariable;
using drake::symbolic::Expression;
using drake::systems::Context;
using drake::trajectories::PiecewisePolynomial;

using Eigen::MatrixXd;
using Eigen::VectorXd;

using multibody::KinematicAccelerationConstraint;
using multibody::KinematicPositionConstraint;
using multibody::KinematicVelocityConstraint;

template <typename T>
Dircon<T>::Dircon(const DirconModeSequence<T>& mode_sequence)
    : Dircon<T>({}, &mode_sequence, mode_sequence.plant(),
                mode_sequence.count_knotpoints()) {}

template <typename T>
Dircon<T>::Dircon(DirconMode<T>* mode)
    : Dircon<T>(std::make_unique<DirconModeSequence<T>>(mode), nullptr,
                mode->plant(), mode->num_knotpoints()) {}

/// Private constructor. Determines which DirconModeSequence was provided,
/// a locally owned unique_ptr or an externally owned const reference
template <typename T>
Dircon<T>::Dircon(std::unique_ptr<DirconModeSequence<T>> my_sequence,
                  const DirconModeSequence<T>* ext_sequence,
                  const MultibodyPlant<T>& plant, int num_knotpoints)
    : drake::planning::trajectory_optimization::MultipleShooting(
    plant.num_actuators(), plant.num_positions() + plant.num_velocities(),
    num_knotpoints, 1e-8, 1e8),
      my_sequence_(std::move(my_sequence)),
      plant_(plant),
      mode_sequence_(ext_sequence ? *ext_sequence : *my_sequence_),
      contexts_(num_modes()),
      mode_start_(num_modes()) {
  // Loop over all modes
  for (int i_mode = 0; i_mode < num_modes(); i_mode++) {
    const auto& mode = get_mode(i_mode);

    // Identify starting index for this mode, accounting for shared knotpoints
    if (i_mode == 0) {
      mode_start_[i_mode] = 0;
    } else {
      mode_start_[i_mode] =
          mode_start_[i_mode - 1] + mode_length(i_mode - 1) - 1;
    }

    //
    // Set constraints on timesteps
    //
    if (mode_length(i_mode) > 1) {
      double min_dt = mode.min_T() / (mode.num_knotpoints() - 1);
      double max_dt = mode.max_T() / (mode.num_knotpoints() - 1);
      prog().AddBoundingBoxConstraint(min_dt, max_dt, time_step(mode_start_[i_mode]));
      for (int j = 0; j < mode.num_knotpoints() - 2; j++) {
        // all timesteps must be equal
        prog().AddLinearConstraint(time_step(mode_start_[i_mode] + j) ==
            time_step(mode_start_[i_mode] + j + 1));
      }
    }

    //
    // Create new decision variables
    //
    force_vars_.push_back(prog().NewContinuousVariables(
        mode.evaluators().count_full() * mode.num_knotpoints(),
        "lambda[" + std::to_string(i_mode) + "]"));
    collocation_force_vars_.push_back(prog().NewContinuousVariables(
        mode.evaluators().count_full() * (mode.num_knotpoints() - 1),
        "lambda_c[" + std::to_string(i_mode) + "]"));
    collocation_slack_vars_.push_back(prog().NewContinuousVariables(
        mode.evaluators().count_full() * (mode.num_knotpoints() - 1),
        "gamma[" + std::to_string(i_mode) + "]"));

    // quaternion_slack_vars_ (slack variables used to scale quaternion norm to
    // 1 in the dynamic constraints)
    int num_quat = multibody::QuaternionStartIndices(plant_).size();
    quaternion_slack_vars_.push_back(
        prog().NewContinuousVariables(num_quat * (mode.num_knotpoints() - 1),
                               "quat_slack[" + std::to_string(i_mode) + "]"));

    // Bound quaternion slack variables to avoid false full rotations
    double slack_bound = 1;
    prog().AddBoundingBoxConstraint(-slack_bound, slack_bound,
                             quaternion_slack_vars_.at(i_mode));

    //  Post-impact variables. Note: impulse variables declared below.
    if (i_mode > 0) {
      v_post_impact_vars_.push_back(prog().NewContinuousVariables(
          plant_.num_velocities(), "v_p[" + std::to_string(i_mode) + "]"));
      v_post_impact_vars_substitute_.push_back(
          {state().tail(plant_.num_velocities()),
           Eigen::Map<MatrixXDecisionVariable>(
               v_post_impact_vars_.back().data(), plant_.num_velocities(), 1)
               .cast<Expression>()});
    }

    // Constraint offset variables for relative constraints
    offset_vars_.push_back(
        prog().NewContinuousVariables(mode.num_relative_constraints(),
                               "rel_offset[" + std::to_string(i_mode) + "]"));

    //
    // Create context elements for knot points
    //
    for (int j = 0; j < mode.num_knotpoints(); j++) {
      contexts_[i_mode].push_back(std::move(plant_.CreateDefaultContext()));
    }

    // We only need an impact if the post-impact constraint set contains
    // anything not already in the pre-impact constraint set.
    bool is_impact = false;
    if (i_mode > 0) {
      auto prev_evaluators = get_mode(i_mode - 1).evaluators().get_evaluators();
      for (auto e : mode.evaluators().get_evaluators()) {
        if (std::find(prev_evaluators.begin(), prev_evaluators.end(), e) ==
            prev_evaluators.end()) {
          is_impact = true;
          break;
        }
      }
    }

    //
    // Create and add collocation constraints
    //

    // Want to set cache_size > number of decision variables. While we have not
    // declared every decision variable yet (see impulse variables below), the
    // impulse variables do not enter into any dynamics evaluations, so we are
    // safe. Add a small factor (10%) just for safety margin.
    int cache_size = 1.1 * prog().num_vars();
    cache_.push_back(
        std::make_unique<DynamicsCache<T>>(mode.evaluators(), cache_size));
    for (int j = 0; j < mode.num_knotpoints() - 1; j++) {
      auto constraint = std::make_shared<DirconCollocationConstraint<T>>(
          plant_, mode.evaluators(), contexts_[i_mode].at(j).get(),
          contexts_[i_mode].at(j + 1).get(), i_mode, j, cache_[i_mode].get());
      constraint->SetConstraintScaling(mode.GetDynamicsScale());
      prog().AddConstraint(
          constraint,
          {time_step(mode_start_[i_mode] + j), state_vars(i_mode, j),
           state_vars(i_mode, j + 1), input_vars(i_mode, j),
           input_vars(i_mode, j + 1), force_vars(i_mode, j),
           force_vars(i_mode, j + 1), collocation_force_vars(i_mode, j),
           collocation_slack_vars(i_mode, j),
           quaternion_slack_vars(i_mode, j)});
    }

    //
    // Create and add kinematic constraints
    //
    for (int j = 0; j < mode.num_knotpoints(); j++) {
      // Position constraints if type is All
      if (mode.get_constraint_type(j) == KinematicConstraintType::kAll) {
        VectorXd lb = VectorXd::Zero(mode.evaluators().count_active());
        VectorXd ub = VectorXd::Zero(mode.evaluators().count_active());

        // If we are in the first knotpoint, do not constrain the position of
        // any constraints that overlap with the previous mode, as such a
        // constraint would be redundant. We do this here by setting bounds to
        // +/-inf.
        if (j == 0 && i_mode > 0) {
          std::vector<int> row_indices =
              get_mode(i_mode - 1)
                  .evaluators()
                  .FindActiveIndicesUnion(mode.evaluators());
          for (const int row_index : row_indices) {
            lb(row_index) = -std::numeric_limits<double>::infinity();
            ub(row_index) = std::numeric_limits<double>::infinity();
          }
        }

        auto pos_constraint = std::make_shared<KinematicPositionConstraint<T>>(
            plant_, mode.evaluators(), lb, ub, mode.relative_constraints(),
            contexts_[i_mode].at(j).get(),
            "kinematic_position[" + std::to_string(i_mode) + "][" +
                std::to_string(j) + "]");
        pos_constraint->SetConstraintScaling(mode.GetKinPositionScale());
        prog().AddConstraint(pos_constraint,
                      {state_vars(i_mode, j).head(plant_.num_positions()),
                       offset_vars(i_mode)});
      }

      // Velocity constraints if type is not acceleration only. Also skip if
      // this is the first knotpoint following an impact-free mode transition.
      if (mode.get_constraint_type(j) != KinematicConstraintType::kAccelOnly) {
        // Skip if i_mode > 0 and j == 0 and no impact
        if (i_mode == 0 || j > 0 || is_impact) {
          auto vel_constraint =
              std::make_shared<KinematicVelocityConstraint<T>>(
                  plant_, mode.evaluators(),
                  VectorXd::Zero(mode.evaluators().count_active()),
                  VectorXd::Zero(mode.evaluators().count_active()),
                  contexts_[i_mode].at(j).get(),
                  "kinematic_velocity[" + std::to_string(i_mode) + "][" +
                      std::to_string(j) + "]");
          vel_constraint->SetConstraintScaling(mode.GetKinVelocityScale());
          prog().AddConstraint(vel_constraint, state_vars(i_mode, j));
        }
      }

      // Acceleration constraints (always)
      auto accel_constraint = std::make_shared<CachedAccelerationConstraint<T>>(
          plant_, mode.evaluators(), contexts_[i_mode].at(j).get(),
          "kinematic_acceleration[" + std::to_string(i_mode) + "][" +
              std::to_string(j) + "]",
          cache_[i_mode].get());
      accel_constraint->SetConstraintScaling(mode.GetKinAccelerationScale());
      prog().AddConstraint(accel_constraint,
                    {state_vars(i_mode, j), input_vars(i_mode, j),
                     force_vars(i_mode, j)});
    }

    //
    // Create and add impact constraints
    //
    if (i_mode > 0) {
      int pre_impact_index = mode_length(i_mode - 1) - 1;
      if (is_impact) {
        impulse_vars_.push_back(
            prog().NewContinuousVariables(mode.evaluators().count_full(),
                                   "impulse[" + std::to_string(i_mode) + "]"));

        // Use pre-impact context
        auto impact_constraint = std::make_shared<ImpactConstraint<T>>(
            plant_, mode.evaluators(), contexts_[i_mode - 1].back().get(),
            "impact[" + std::to_string(i_mode) + "]");
        impact_constraint->SetConstraintScaling(mode.GetImpactScale());

        prog().AddConstraint(
            impact_constraint,
            {state_vars(i_mode - 1, pre_impact_index), impulse_vars(i_mode - 1),
             post_impact_velocity_vars(i_mode - 1)});
      } else {
        // Add empty decision variables
        impulse_vars_.push_back(prog().NewContinuousVariables(0, ""));

        // Linear equality constraint on velocity variables
        VectorXDecisionVariable pre_impact_velocity =
            state_vars(i_mode - 1, pre_impact_index)
                .tail(plant_.num_velocities());
        prog().AddLinearConstraint(pre_impact_velocity ==
            post_impact_velocity_vars(i_mode - 1));
      }
    }

    //
    // Create and add quaternion constraints
    //
    auto quaternion_constraint = std::make_shared<QuaternionConstraint<T>>();
    for (int j = 0; j < mode.num_knotpoints(); j++) {
      if (!mode.IsSkipQuaternionConstraint(j)) {
        auto start_indices = multibody::QuaternionStartIndices(plant_);
        for (auto start_index : start_indices) {
          prog().AddConstraint(quaternion_constraint,
                        state_vars(i_mode, j).segment(start_index, 4));
        }
      }
    }

    ///
    /// Add friction cone constraints to force variables
    ///
    /// TODO: hard-coding number of frictional faces, but this could be an
    /// option, along with a choice of which friction cone constraint to use.
    int num_faces = 4;
    for (int k = 0; k < mode.evaluators().num_evaluators(); k++) {
      const auto& e = mode.evaluators().get_evaluator(k);
      auto force_constraints_vec = e.CreateLinearFrictionConstraints(num_faces);
      for (auto force_constraint : force_constraints_vec) {
        // Add to knot point forces
        for (int j = 0; j < mode.num_knotpoints(); j++) {
          prog().AddConstraint(
              force_constraint,
              force_vars(i_mode, j).segment(
                  mode.evaluators().evaluator_full_start(k), e.num_full()));
        }

        if (i_mode > 0 && is_impact) {
          // Add to impulse variables
          prog().AddConstraint(force_constraint,
                        impulse_vars(i_mode - 1)
                            .segment(mode.evaluators().evaluator_full_start(k),
                                     e.num_full()));
        }
      }
    }

    if (mode.get_force_regularization() != 0) {
      // Add regularization cost on forcese
      {
        int size = force_vars_.at(i_mode).size();
        prog().AddQuadraticCost(
            mode.get_force_regularization() * MatrixXd::Identity(size, size),
            VectorXd::Zero(size), force_vars_.at(i_mode));
      }

      {
        int size = collocation_force_vars_.at(i_mode).size();
        prog().AddQuadraticCost(
            mode.get_force_regularization() * MatrixXd::Identity(size, size),
            VectorXd::Zero(size), collocation_force_vars_.at(i_mode));
      }

      {
        int size = collocation_slack_vars_.at(i_mode).size();
        prog().AddQuadraticCost(
            mode.get_force_regularization() * MatrixXd::Identity(size, size),
            VectorXd::Zero(size), collocation_slack_vars_.at(i_mode));
      }

      {
        int size = quaternion_slack_vars_.at(i_mode).size();
        prog().AddQuadraticCost(
            mode.get_force_regularization() * MatrixXd::Identity(size, size),
            VectorXd::Zero(size), quaternion_slack_vars_.at(i_mode));
      }
    }
  }
}

///
/// Getters for decision variables
///
template <typename T>
const VectorXDecisionVariable Dircon<T>::force_vars(int mode_index,
                                                    int knotpoint_index) const {
  const auto& mode = get_mode(mode_index);
  return force_vars_.at(mode_index)
      .segment(knotpoint_index * mode.evaluators().count_full(),
               mode.evaluators().count_full());
}

template <typename T>
const VectorXDecisionVariable Dircon<T>::collocation_force_vars(
    int mode_index, int collocation_index) const {
  const auto& mode = get_mode(mode_index);
  return collocation_force_vars_.at(mode_index)
      .segment(collocation_index * mode.evaluators().count_full(),
               mode.evaluators().count_full());
}

template <typename T>
const VectorXDecisionVariable Dircon<T>::collocation_slack_vars(
    int mode_index, int collocation_index) const {
  const auto& mode = get_mode(mode_index);
  return collocation_slack_vars_.at(mode_index)
      .segment(collocation_index * mode.evaluators().count_full(),
               mode.evaluators().count_full());
}

template <typename T>
const VectorXDecisionVariable Dircon<T>::state_vars(int mode_index,
                                                    int knotpoint_index) const {
  // If first knot of a mode after the first, use post impact velocity variables
  if (knotpoint_index == 0 && mode_index > 0) {
    VectorXDecisionVariable ret(plant_.num_positions() +
        plant_.num_velocities());
    ret << x_vars().segment(mode_start_[mode_index] * (plant_.num_positions() +
                                plant_.num_velocities()),
                            plant_.num_positions()),
        post_impact_velocity_vars(mode_index - 1);
    return ret;
  } else {
    return x_vars().segment(
        (mode_start_[mode_index] + knotpoint_index) *
            (plant_.num_positions() + plant_.num_velocities()),
        plant_.num_positions() + plant_.num_velocities());
  }
}

template <typename T>
const VectorXDecisionVariable Dircon<T>::input_vars(int mode_index,
                                                    int knotpoint_index) const {
  return u_vars().segment(
      (mode_start_[mode_index] + knotpoint_index) * plant_.num_actuators(),
      plant_.num_actuators());
}

template <typename T>
const VectorXDecisionVariable Dircon<T>::quaternion_slack_vars(
    int mode_index, int collocation_index) const {
  int num_quat = multibody::QuaternionStartIndices(plant_).size();
  return quaternion_slack_vars_.at(mode_index)
      .segment(num_quat * collocation_index, num_quat);
}

template <typename T>
const VectorXDecisionVariable Dircon<T>::offset_vars(int mode_index) const {
  return offset_vars_.at(mode_index);
}

template <typename T>
const VectorXDecisionVariable Dircon<T>::post_impact_velocity_vars(
    int mode_transition_index) const {
  return v_post_impact_vars_.at(mode_transition_index);
}

template <typename T>
const VectorXDecisionVariable Dircon<T>::impulse_vars(
    int mode_transition_index) const {
  return impulse_vars_.at(mode_transition_index);
}

template <typename T>
void Dircon<T>::CreateVisualizationCallback(
    std::string model_file, std::vector<unsigned int> poses_per_mode,
    double alpha, std::string weld_frame_to_world) {
  DRAKE_DEMAND(!callback_visualizer_);  // Cannot be set twice
  DRAKE_DEMAND(poses_per_mode.size() == (uint)num_modes());

  // Count number of total poses, start and finish of every mode
  int num_poses = num_modes() + 1;
  for (int i = 0; i < num_modes(); i++) {
    DRAKE_DEMAND(poses_per_mode.at(i) == 0 ||
        (poses_per_mode.at(i) + 2 <= (uint)mode_length(i)));
    num_poses += poses_per_mode.at(i);
  }

  // Assemble variable list
  drake::solvers::VectorXDecisionVariable vars(num_poses *
      plant_.num_positions());

  int index = 0;
  for (int i = 0; i < num_modes(); i++) {
    // Set variable block, extracting positions only
    vars.segment(index * plant_.num_positions(), plant_.num_positions()) =
        state_vars(i, 0).head(plant_.num_positions());
    index++;

    for (uint j = 0; j < poses_per_mode.at(i); j++) {
      // The jth element in mode i, counting in between the start/end poses
      int modei_index =
          (j + 1) * (mode_length(i) - 1) / (poses_per_mode.at(i) + 1);

      vars.segment(index * plant_.num_positions(), plant_.num_positions()) =
          state_vars(i, modei_index).head(plant_.num_positions());
      index++;
    }
  }

  // Final state
  auto last_mode = get_mode(num_modes() - 1);
  vars.segment(index * plant_.num_positions(), plant_.num_positions()) =
      state_vars(num_modes() - 1, last_mode.num_knotpoints() - 1)
          .head(plant_.num_positions());

  VectorXd alpha_vec = VectorXd::Constant(num_poses, alpha);
  alpha_vec(0) = 1;
  alpha_vec(num_poses - 1) = 1;

  // Create visualizer
  callback_visualizer_ = std::make_unique<multibody::MultiposeVisualizer>(
      model_file, num_poses, alpha_vec, weld_frame_to_world);

  // Callback lambda function
  auto my_callback = [this, num_poses](const Eigen::Ref<const VectorXd>& vars) {
    VectorXd vars_copy = vars;
    Eigen::Map<MatrixXd> states(vars_copy.data(), this->plant_.num_positions(),
                                num_poses);

    this->callback_visualizer_->DrawPoses(states);
  };

  prog().AddVisualizationCallback(my_callback, vars);
}

template <typename T>
void Dircon<T>::CreateVisualizationCallback(std::string model_file,
                                            unsigned int num_poses,
                                            double alpha,
                                            std::string weld_frame_to_world) {
  // Check that there is an appropriate number of poses
  DRAKE_DEMAND(num_poses >= (uint)num_modes() + 1);
  DRAKE_DEMAND(num_poses <= (uint)N());

  // sum of mode lengths, excluding ends
  int mode_sum = 0;
  for (int i_mode = 0; i_mode < num_modes(); i_mode++) {
    auto mode = get_mode(i_mode);
    if (mode.num_knotpoints() > 2) {
      mode_sum += mode.num_knotpoints() - 2;
    }
  }

  // Split up num_poses per modes, rounding down
  // If NP is the total number of  poses to visualize, excluding ends (interior)
  //   S is mode_sum, the total number of the interior knot points
  //   and N the number of interior knot points for a particular mode, then
  //
  //   poses = (NP * N )/S
  unsigned int num_poses_without_ends = num_poses - num_modes() - 1;
  unsigned int assigned_sum = 0;
  std::vector<unsigned int> num_poses_per_mode;
  for (int i_mode = 0; i_mode < num_modes(); i_mode++) {
    if (mode_length(i_mode) > 2) {
      unsigned int mode_count =
          (num_poses_without_ends * (mode_length(i_mode) - 2)) / mode_sum;
      num_poses_per_mode.push_back(mode_count);
      assigned_sum += mode_count;
    } else {
      num_poses_per_mode.push_back(0);
    }
  }

  // Need to add back in the fractional bits, using the largest fractions
  while (assigned_sum < num_poses_without_ends) {
    int largest_mode = -1;
    double value = 0;
    for (int i_mode = 0; i_mode < num_modes(); i_mode++) {
      double fractional_value =
          num_poses_without_ends * (mode_length(i_mode) - 2) -
              num_poses_per_mode.at(i_mode) * mode_sum;

      if (fractional_value > value) {
        value = fractional_value;
        largest_mode = i_mode;
      }
    }

    num_poses_per_mode.at(largest_mode)++;
    assigned_sum++;
  }

  CreateVisualizationCallback(model_file, num_poses_per_mode, alpha,
                              weld_frame_to_world);
}

template <typename T>
void Dircon<T>::CreateVisualizationCallback(std::string model_file,
                                            double alpha,
                                            std::string weld_frame_to_world) {
  CreateVisualizationCallback(model_file, N(), alpha, weld_frame_to_world);
}

template <typename T>
VectorX<Expression> Dircon<T>::SubstitutePlaceholderVariables(
    const VectorX<Expression>& f, int interval_index) const {
  VectorX<Expression> ret(f.size());
  for (int i = 0; i < f.size(); i++) {
    ret(i) =
        MultipleShooting::SubstitutePlaceholderVariables(f(i), interval_index);
  }
  return ret;
}

template <typename T>
Expression Dircon<T>::SubstitutePostImpactVelocityVariables(
    const drake::symbolic::Expression& e, int mode) const {
  DRAKE_DEMAND(mode > 0);
  drake::symbolic::Substitution s;
  for (int i = 0; i < v_post_impact_vars_substitute_[mode - 1].first.size();
       ++i) {
    s.emplace(v_post_impact_vars_substitute_[mode - 1].first[i],
              v_post_impact_vars_substitute_[mode - 1].second[i]);
  }
  return e.Substitute(s);
}

template <typename T>
void Dircon<T>::DoAddRunningCost(const drake::symbolic::Expression& g) {
  // Trapezoidal integration:
  //    sum_{i=0...N-2} h_i/2.0 * (g_i + g_{i+1}), or
  // g_0*h_0/2.0 + [sum_{i=1...N-2} g_i*(h_{i-1} + h_i)/2.0] +
  // g_{N-1}*h_{N-2}/2.0.

  // Here, we add the cost using symbolic expression. The expression is a
  // polynomial of degree 3 which Drake can handle, although the
  // documentation says it only supports up to second order.

  for (int mode = 0; mode < num_modes(); ++mode) {
    int mode_start = mode_start_[mode];
    int mode_end = mode_start_[mode] + mode_length(mode);
    // Substitute the velocity vars with the correct post-impact velocity vars
    // before substituting the rest of the expression
    if (mode > 0) {
      prog().AddCost(MultipleShooting::SubstitutePlaceholderVariables(
          SubstitutePostImpactVelocityVariables(g, mode), mode_start) *
          (h_vars()(mode_start) / 2));
    } else {
      prog().AddCost(MultipleShooting::SubstitutePlaceholderVariables(g, mode_start) *
          (h_vars()(mode_start) / 2));
    }
    for (int i = mode_start + 1; i < mode_end - 1; ++i) {
      prog().AddCost(MultipleShooting::SubstitutePlaceholderVariables(g, i) *
          (h_vars()(i - 1) + h_vars()(i)) / 2);
    }
    prog().AddCost(MultipleShooting::SubstitutePlaceholderVariables(g, mode_end - 1) *
        h_vars()(mode_end - 2) / 2);
  }
}

template <typename T>
void Dircon<T>::GetStateAndDerivativeSamples(
    const drake::solvers::MathematicalProgramResult& result,
    std::vector<Eigen::MatrixXd>* state_samples,
    std::vector<Eigen::MatrixXd>* derivative_samples,
    std::vector<Eigen::VectorXd>* state_breaks) const {
  DRAKE_ASSERT(state_samples->empty());
  DRAKE_ASSERT(derivative_samples->empty());
  DRAKE_ASSERT(state_breaks->empty());

  VectorXd times(GetSampleTimes(result));

  for (int mode = 0; mode < num_modes(); mode++) {
    MatrixXd states_i(num_states(), mode_length(mode));
    MatrixXd derivatives_i(num_states(), mode_length(mode));
    VectorXd times_i(mode_length(mode));
    for (int j = 0; j < mode_length(mode); j++) {
      int k = mode_start_[mode] + j;

      VectorX<T> xk = result.GetSolution(state_vars(mode, j));
      VectorX<T> uk = result.GetSolution(input_vars(mode, j));
      auto context = multibody::CreateContext<T>(plant_, xk, uk);

      states_i.col(j) = drake::math::DiscardGradient(xk);
      auto xdot = get_mode(mode).evaluators().CalcTimeDerivativesWithForce(
          context.get(), result.GetSolution(force_vars(mode, j)));
      derivatives_i.col(j) = drake::math::DiscardGradient(xdot);
      times_i(j) = times(k);
    }
    state_samples->push_back(states_i);
    derivative_samples->push_back(derivatives_i);
    state_breaks->push_back(times_i);
  }
}

template <typename T>
PiecewisePolynomial<double> Dircon<T>::ReconstructInputTrajectory(
    const MathematicalProgramResult& result) const {
  return PiecewisePolynomial<double>::FirstOrderHold(GetSampleTimes(result),
                                                     GetInputSamples(result));
}

template <typename T>
PiecewisePolynomial<double> Dircon<T>::ReconstructStateTrajectory(
    const MathematicalProgramResult& result) const {
  std::vector<MatrixXd> states;
  std::vector<MatrixXd> derivatives;
  std::vector<VectorXd> times;
  GetStateAndDerivativeSamples(result, &states, &derivatives, &times);
  PiecewisePolynomial<double> state_traj =
      PiecewisePolynomial<double>::CubicHermite(times[0], states[0],
                                                derivatives[0]);
  for (int mode = 1; mode < num_modes(); ++mode) {
    // Cannot form trajectory with only a single break
    if (mode_length(mode) < 2) {
      continue;
    }
    state_traj.ConcatenateInTime(PiecewisePolynomial<double>::CubicHermite(
        times[mode], states[mode], derivatives[mode]));
  }
  return state_traj;
}

template <typename T>
void Dircon<T>::SetInitialForceTrajectory(
    int mode_index, const PiecewisePolynomial<double>& traj_init_l,
    const PiecewisePolynomial<double>& traj_init_lc,
    const PiecewisePolynomial<double>& traj_init_vc) {
  const auto& mode = get_mode(mode_index);
  double start_time = 0;
  double h;
  if (time_steps_are_decision_variables ())
    h = prog().GetInitialGuess(h_vars()[0]);
  else
    h = fixed_time_step();

  VectorXd guess_force(force_vars_[mode_index].size());
  if (traj_init_l.empty()) {
    guess_force.fill(0);  // Start with 0
  } else {
    for (int i = 0; i < mode.num_knotpoints(); ++i) {
      guess_force.segment(mode.evaluators().count_full() * i,
                          mode.evaluators().count_full()) =
          traj_init_l.value(start_time + i * h);
    }
  }
  prog().SetInitialGuess(force_vars_[mode_index], guess_force);

  VectorXd guess_collocation_force(collocation_force_vars_[mode_index].size());
  if (traj_init_lc.empty()) {
    guess_collocation_force.fill(0);  // Start with 0
  } else {
    for (int i = 0; i < mode.num_knotpoints() - 1; ++i) {
      guess_collocation_force.segment(mode.evaluators().count_full() * i,
                                      mode.evaluators().count_full()) =
          traj_init_lc.value(start_time + (i + 0.5) * h);
    }
  }

  prog().SetInitialGuess(collocation_force_vars_[mode_index], guess_collocation_force);

  VectorXd guess_collocation_slack(collocation_slack_vars_[mode_index].size());
  if (traj_init_vc.empty()) {
    guess_collocation_slack.fill(0);  // Start with 0
  } else {
    for (int i = 0; i < mode.num_knotpoints() - 1; ++i) {
      guess_collocation_slack.segment(mode.evaluators().count_full() * i,
                                      mode.evaluators().count_full()) =
          traj_init_vc.value(start_time + (i + 0.5) * h);
    }
  }

  // call superclass method
  prog().SetInitialGuess(collocation_slack_vars_[mode_index], guess_collocation_slack);
}

template <typename T>
void Dircon<T>::SetInitialForceTrajectory(
    int mode_index, const PiecewisePolynomial<double>& traj_init_l) {
  const auto& mode = get_mode(mode_index);
  double start_time = 0;
  double h;
  if (time_steps_are_decision_variables())
    h = prog().GetInitialGuess(h_vars()[0]);
  else
    h = fixed_time_step();

  VectorXd guess_force(force_vars_[mode_index].size());
  VectorXd guess_collocation_force(collocation_force_vars_[mode_index].size());
  if (traj_init_l.empty()) {
    guess_force.fill(0);  // Start with 0
  } else {
    for (int i = 0; i < mode.num_knotpoints(); ++i) {
      guess_force.segment(mode.evaluators().count_full() * i,
                          mode.evaluators().count_full()) =
          traj_init_l.value(start_time + i * h);
    }
    for (int i = 0; i < mode.num_knotpoints() - 1; ++i) {
      guess_collocation_force.segment(mode.evaluators().count_full() * i,
                                      mode.evaluators().count_full()) =
          traj_init_l.value(start_time + (i + 0.5) * h);
    }
  }
  prog().SetInitialGuess(force_vars_[mode_index], guess_force);
  prog().SetInitialGuess(collocation_force_vars_[mode_index], guess_collocation_force);
}

template <typename T>
int Dircon<T>::num_modes() const {
  return mode_sequence_.num_modes();
}

template <typename T>
int Dircon<T>::mode_length(int mode_index) const {
  return get_mode(mode_index).num_knotpoints();
}

template <typename T>
void Dircon<T>::ScaleTimeVariables(double scale) {
  for (int i = 0; i < h_vars().size(); i++) {
    prog().SetVariableScaling(h_vars()(i), scale);
  }
}

template <typename T>
void Dircon<T>::ScaleQuaternionSlackVariables(double scale) {
  DRAKE_DEMAND(multibody::HasQuaternion(plant_));
  for (int i_mode = 0; i_mode < num_modes(); i_mode++) {
    for (int j = 0; j < mode_length(i_mode) - 1; j++) {
      const auto& vars = quaternion_slack_vars(i_mode, j);
      for (int k = 0; k < vars.size(); k++) {
        prog().SetVariableScaling(vars(k), scale);
      }
    }
  }
}

template <typename T>
void Dircon<T>::ScaleStateVariable(int state_index, double scale) {
  DRAKE_DEMAND(0 <= state_index &&
      state_index < plant_.num_positions() + plant_.num_velocities());

  // x_vars_ in MathematicalProgram
  for (int j_knot = 0; j_knot < N(); j_knot++) {
    auto vars = this->state(j_knot);
    prog().SetVariableScaling(vars(state_index), scale);
  }

  // v_post_impact_vars_
  if (state_index > plant_.num_positions()) {
    for (int mode = 0; mode < num_modes() - 1; mode++) {
      auto vars = post_impact_velocity_vars(mode);
      prog().SetVariableScaling(vars(state_index - plant_.num_positions()),
                               scale);
    }
  }
}

template <typename T>
void Dircon<T>::ScaleInputVariable(int input_index, double scale) {
  DRAKE_DEMAND((0 <= input_index) && (input_index < plant_.num_actuators()));

  // u_vars_ in MathematicalProgram
  for (int j_knot = 0; j_knot < N(); j_knot++) {
    auto vars = this->input(j_knot);
    prog().SetVariableScaling(vars(input_index), scale);
  }
}

template <typename T>
void Dircon<T>::ScaleForceVariable(int mode_index, int force_index,
                                   double scale) {
  DRAKE_DEMAND((0 <= mode_index) && (mode_index < num_modes()));
  int n_lambda = get_mode(mode_index).evaluators().count_full();
  DRAKE_DEMAND((0 <= force_index) && (force_index < n_lambda));

  // Force at knot points
  for (int j = 0; j < mode_length(mode_index); j++) {
    prog().SetVariableScaling(force_vars(mode_index, j)(force_index), scale);
  }
  // Force at collocation pints
  for (int j = 0; j < mode_length(mode_index) - 1; j++) {
    prog().SetVariableScaling(collocation_force_vars(mode_index, j)(force_index),
                             scale);
  }
}

template <typename T>
void Dircon<T>::ScaleImpulseVariable(int mode_index, int impulse_index,
                                     double scale) {
  DRAKE_DEMAND((0 <= mode_index) && (mode_index < num_modes() - 1));
  int n_impulse = get_mode(mode_index).evaluators().count_full();
  DRAKE_DEMAND((0 <= impulse_index) && (impulse_index < n_impulse));

  prog().SetVariableScaling(impulse_vars(mode_index)(impulse_index), scale);
}

template <typename T>
void Dircon<T>::ScaleKinConstraintSlackVariable(int mode_index, int slack_index,
                                                double scale) {
  DRAKE_DEMAND((0 <= mode_index) && (mode_index < num_modes() - 1));
  int n_lambda = get_mode(mode_index).evaluators().count_full();
  DRAKE_DEMAND(slack_index < n_lambda);

  for (int j = 0; j < mode_length(mode_index) - 1; j++) {
    prog().SetVariableScaling(collocation_slack_vars(mode_index, j)(slack_index),
                             scale);
  }
}

template <typename T>
void Dircon<T>::ScaleStateVariables(std::vector<int> index_list, double scale) {
  for (const auto& idx : index_list) {
    ScaleStateVariable(idx, scale);
  }
}

template <typename T>
void Dircon<T>::ScaleInputVariables(std::vector<int> index_list, double scale) {
  for (const auto& idx : index_list) {
    ScaleInputVariable(idx, scale);
  }
}

template <typename T>
void Dircon<T>::ScaleForceVariables(int mode, std::vector<int> index_list,
                                    double scale) {
  for (const auto& idx : index_list) {
    ScaleForceVariable(mode, idx, scale);
  }
}

template <typename T>
void Dircon<T>::ScaleImpulseVariables(int mode, std::vector<int> index_list,
                                      double scale) {
  for (const auto& idx : index_list) {
    ScaleImpulseVariable(mode, idx, scale);
  }
}

template <typename T>
void Dircon<T>::ScaleKinConstraintSlackVariables(int mode,
                                                 std::vector<int> index_list,
                                                 double scale) {
  for (const auto& idx : index_list) {
    ScaleKinConstraintSlackVariable(mode, idx, scale);
  }
}

template <typename T>
Eigen::MatrixXd Dircon<T>::GetStateSamplesByMode(
    const MathematicalProgramResult& result, int mode) const {
  Eigen::MatrixXd states(num_states(), mode_length(mode));
  for (int i = 0; i < mode_length(mode); i++) {
    states.col(i) = result.GetSolution(state_vars(mode, i));
  }
  return states;
}

template <typename T>
Eigen::MatrixXd Dircon<T>::GetInputSamplesByMode(
    const MathematicalProgramResult& result, int mode) const {
  Eigen::MatrixXd inputs(num_inputs(), mode_length(mode));
  for (int i = 0; i < mode_length(mode); i++) {
    inputs.col(i) = result.GetSolution(input_vars(mode, i));
  }
  return inputs;
}

template <typename T>
Eigen::MatrixXd Dircon<T>::GetForceSamplesByMode(
    const MathematicalProgramResult& result, int mode) const {
  Eigen::MatrixXd forces(get_mode(mode).evaluators().count_full(),
                         mode_length(mode));
  for (int i = 0; i < mode_length(mode); i++) {
    forces.col(i) = result.GetSolution(force_vars(mode, i));
  }
  return forces;
}

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace dairlib

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::systems::trajectory_optimization::Dircon)
