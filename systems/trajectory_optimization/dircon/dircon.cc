#include "systems/trajectory_optimization/dircon/dircon.h"

#include "multibody/kinematic/kinematic_constraints.h"
#include "systems/trajectory_optimization/dircon/dircon_opt_constraints.h"

#include "multibody/multibody_utils.h"

namespace dairlib {
namespace systems {
namespace trajectory_optimization {

using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::solvers::VectorXDecisionVariable;

using Eigen::MatrixXd;
using Eigen::VectorXd;

using multibody::KinematicPositionConstraint;
using multibody::KinematicVelocityConstraint;
using multibody::KinematicAccelerationConstraint;

template <typename T>
Dircon<T>::Dircon(const DirconModeSequence<T>& mode_sequence)
  : Dircon<T>({}, &mode_sequence, mode_sequence.plant(),
        mode_sequence.count_knotpoints()) {}

template <typename T>
Dircon<T>::Dircon(DirconMode<T>* mode)
  : Dircon<T>(std::make_unique<DirconModeSequence<T>>(mode),
        nullptr,
        mode->plant(),
        mode->num_knotpoints()) {}

/// Private constructor. Determines which DirconModeSequence was provided,
/// a locally owned unique_ptr or an externally owned const reference
template <typename T>
Dircon<T>::Dircon(std::unique_ptr<DirconModeSequence<T>> my_sequence,
    const DirconModeSequence<T>* ext_sequence,
    const MultibodyPlant<T>& plant,
    int num_knotpoints)
    : drake::systems::trajectory_optimization::MultipleShooting(
          plant.num_actuators(),
          plant.num_positions() + plant.num_velocities(),
          num_knotpoints, false),
      my_sequence_(std::move(my_sequence_)),
      plant_(plant),
      mode_sequence_(ext_sequence ? *ext_sequence : *my_sequence),
      contexts_(mode_sequence_.num_modes()) {
  // Loop over all modes
  for (int i_mode = 0; i_mode < mode_sequence_.num_modes(); i_mode++) {    
    const auto& mode = mode_sequence_.mode(i_mode);

    // Identify starting index for this mode, accounting for shared knotpoints
    if (i_mode == 0) {
      mode_start_[i_mode] = 0;
    } else {
      mode_start_[i_mode] = mode_start_[i_mode-1] +
          mode_sequence_.mode(i_mode - 1).num_knotpoints() - 1;
    }
    
    //
    // Set constraints on timesteps
    //
    double min_dt = mode.min_T() / (mode.num_knotpoints() - 1);
    double max_dt = mode.max_T() / (mode.num_knotpoints() - 1);
    // set timestep bounds
    for (int j = 0; j < mode.num_knotpoints() - 1; j++) {
      AddBoundingBoxConstraint(min_dt, max_dt,
          timestep(mode_start_[i_mode] + j));
    }
    for (int j = 0; j < mode.num_knotpoints() - 2; j++) {
      // all timesteps must be equal
      AddLinearConstraint(timestep(mode_start_[i_mode] + j) ==
                          timestep(mode_start_[i_mode] + j + 1));
    }

    //
    // Create new decision variables
    //
    force_vars_.push_back(NewContinuousVariables(
        mode.evaluators().count_full() * mode.num_knotpoints(),
        "lambda[" + std::to_string(i_mode) + "]"));
    collocation_force_vars_.push_back(NewContinuousVariables(
        mode.evaluators().count_full() * (mode.num_knotpoints() - 1),
        "lambda_c[" + std::to_string(i_mode) + "]"));
    collocation_slack_vars_.push_back(NewContinuousVariables(
        mode.evaluators().count_full() * (mode.num_knotpoints() - 1),
        "gamma[" + std::to_string(i_mode) + "]"));

    // quaternion_slack_vars_ (slack variables used to scale quaternion norm to
    // 1 in the dynamic constraints)
    int num_quat = multibody::QuaternionStartIndices(plant_).size();
    quaternion_slack_vars_.push_back(NewContinuousVariables(
        num_quat * mode.num_knotpoints(),
        "quat_slack[" + std::to_string(i_mode) + "]"));

    // Impulse and post-impact variables
    if (i_mode > 0) {
      impulse_vars_.push_back(NewContinuousVariables(
          mode.evaluators().count_full(),
          "impulse[" + std::to_string(i_mode) + "]"));
      v_post_impact_vars_.push_back(NewContinuousVariables(
          plant_.num_velocities(),
          "v_p[" + std::to_string(i_mode) + "]"));
    }

    // Constraint offset variables for relative constraints
    offset_vars_.push_back(NewContinuousVariables(
        mode.num_relative_constraints(),
        "rel_offset[" + std::to_string(i_mode) + "]"));

    //
    // Create context elements for knot points
    //
    for (int j = 0; j < mode.num_knotpoints(); j++) {
      contexts_[i_mode].push_back(std::move(plant_.CreateDefaultContext()));
    }

    //
    // Create and add collocation constraints
    //
    for (int j = 0; j < mode.num_knotpoints() - 1; j++) {
      auto constraint = std::make_shared<DirconCollocationConstraint<T>>(
          plant_, mode.evaluators(), contexts_[i_mode].at(j).get(),
          contexts_[i_mode].at(j+1).get(), i_mode, j);
      constraint->SetConstraintScaling(mode.GetDynamicsScale());
      AddConstraint(constraint,
          { timestep(mode_start_[i_mode] + j),
            state_vars(i_mode, j),
            state_vars(i_mode, j + 1),
            input_vars(i_mode, j),
            input_vars(i_mode, j + 1),
            force_vars(i_mode, j),
            force_vars(i_mode, j + 1),
            collocation_force_vars(i_mode, j),
            collocation_slack_vars(i_mode, j),
            quaternion_slack_vars(i_mode, j) });
    }

    //
    // Create and add kinematic constraints
    //
    for (int j = 0; j < mode.num_knotpoints(); j++) {
      // Position constraints if type is All
      if (mode.get_constraint_type(j) == DirconKinConstraintType::kAll) {
        auto pos_constraint = std::make_shared<KinematicPositionConstraint<T>>(
            plant_, mode.evaluators(),
            VectorXd::Zero(mode.evaluators().count_active()),
            VectorXd::Zero(mode.evaluators().count_active()),
            mode.relative_constraints(),
            contexts_[i_mode].at(j).get(),
            "kinematic_position[" + std::to_string(i_mode) + "]["
                + std::to_string(j) + "]");
        pos_constraint->SetConstraintScaling(mode.GetKinPositionScale());
        AddConstraint(pos_constraint,
            { state_vars(i_mode, j).head(plant_.num_positions()),
              offset_vars(i_mode) });
      }

      // Velocity constraints if type is not acceleration only
      if (mode.get_constraint_type(j) != DirconKinConstraintType::kAccelOnly) {
        auto vel_constraint = std::make_shared<KinematicVelocityConstraint<T>>(
            plant_, mode.evaluators(),
            VectorXd::Zero(mode.evaluators().count_active()),
            VectorXd::Zero(mode.evaluators().count_active()),
            contexts_[i_mode].at(j).get(),
            "kinematic_velocity[" + std::to_string(i_mode) + "]["
                + std::to_string(j) + "]");
        vel_constraint->SetConstraintScaling(mode.GetKinVelocityScale());
        AddConstraint(vel_constraint, state_vars(i_mode, j));
      }

      // Acceleration constraints (always)
      auto accel_constraint =
          std::make_shared<KinematicAccelerationConstraint<T>>(
            plant_, mode.evaluators(),
            VectorXd::Zero(mode.evaluators().count_active()),
            VectorXd::Zero(mode.evaluators().count_active()),
            contexts_[i_mode].at(j).get(),
            "kinematic_acceleration[" + std::to_string(i_mode) + "]["
                + std::to_string(j) + "]");
      accel_constraint->SetConstraintScaling(mode.GetKinAccelerationScale());
      AddConstraint(accel_constraint,
          { state_vars(i_mode, j),
            input_vars(i_mode, j),
            force_vars(i_mode, j) });
    }

    //
    // Create and add impact constraints
    //
    if (i_mode > 0) {
      // Use pre-impact context
      int pre_impact_index = mode_sequence_.mode(i_mode-1).num_knotpoints()-1;
      auto impact_constraint = 
          std::make_shared<DirconImpactConstraint<T>>(
            plant_, mode.evaluators(),
            contexts_[i_mode - 1].back().get(),
            "impact[" + std::to_string(i_mode) + "]");
      impact_constraint->SetConstraintScaling(mode.GetImpactScale());
      AddConstraint(impact_constraint,
          { state_vars(i_mode - 1, pre_impact_index),
            impulse_vars(i_mode - 1),
            post_impact_velocity_vars(i_mode - 1) });
    }

    //
    // Create and add quaternion constraints
    //
    auto quaternion_constraint = 
      std::make_shared<QuaternionNormConstraint<T>>();
    for (int j = 0; j < mode.num_knotpoints(); j++) {
      if (!mode.IsSkipQuaternion(j)) {
        auto start_indices = multibody::QuaternionStartIndices(plant_);
        for (auto start_index : start_indices) {
          AddConstraint(quaternion_constraint,
              state_vars(i_mode, j).segment(start_index, 4));
        }
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
  const auto& mode = mode_sequence_.mode(mode_index);
  return force_vars_.at(mode_index).segment(
      knotpoint_index * mode.evaluators().count_full(),
      mode.evaluators().count_full());
}

template <typename T>
const VectorXDecisionVariable Dircon<T>::collocation_force_vars(int mode_index,
    int knotpoint_index) const {
  const auto& mode = mode_sequence_.mode(mode_index);
  return collocation_force_vars_.at(mode_index).segment(
      knotpoint_index * mode.evaluators().count_full(),
      mode.evaluators().count_full());
}

template <typename T>
const VectorXDecisionVariable Dircon<T>::state_vars(int mode_index,
    int knotpoint_index) const {
  // If first knot of a mode after the first, use post impact velocity variables
  if (knotpoint_index == 0 && mode_index > 0) {
    VectorXDecisionVariable
        ret(plant_.num_positions() + plant_.num_velocities());
    ret << x_vars().segment(mode_start_[mode_index] 
        * (plant_.num_positions() + plant_.num_velocities()),
           plant_.num_positions()),
        post_impact_velocity_vars(mode_index - 1);
    return ret;
  } else {
    return x_vars().segment(mode_start_[mode_index]
            * (plant_.num_positions() + plant_.num_velocities()),
        plant_.num_positions() + plant_.num_velocities());
  }
}

template <typename T>
const VectorXDecisionVariable Dircon<T>::input_vars(int mode_index,
    int knotpoint_index) const {
  return u_vars().segment(mode_start_[mode_index] * plant_.num_actuators(),
      plant_.num_actuators());
}

template <typename T>
const VectorXDecisionVariable Dircon<T>::quaternion_slack_vars(int mode_index,
    int knotpoint_index) const {
  int num_quat = multibody::QuaternionStartIndices(plant_).size();
  return quaternion_slack_vars_.at(mode_index).segment(
      num_quat * knotpoint_index, num_quat);
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
void Dircon<T>::CreateVisualizationCallback(std::string model_file,
    std::vector<unsigned int> poses_per_mode, std::string weld_frame_to_world) {
  DRAKE_DEMAND(!callback_visualizer_);  // Cannot be set twice
  DRAKE_DEMAND(poses_per_mode.size() == (uint) mode_sequence_.num_modes());

  // Count number of total poses, start and finish of every mode
  int num_poses = mode_sequence_.num_modes() + 1;
  for (int i = 0; i < mode_sequence_.num_modes(); i++) {
    DRAKE_DEMAND(poses_per_mode.at(i) == 0 ||  (poses_per_mode.at(i) + 2
        <= (uint) mode_sequence_.mode(i).num_knotpoints()));
    num_poses += poses_per_mode.at(i);
  }

  // Assemble variable list
  drake::solvers::VectorXDecisionVariable vars(
      num_poses * plant_.num_positions());

  int index = 0;
  for (int i = 0; i < mode_sequence_.num_modes(); i++) {
    // Set variable block, extracting positions only
    vars.segment(index * plant_.num_positions(), plant_.num_positions()) = 
        state_vars(i, 0).head(plant_.num_positions());
    index++;

    for (uint j = 0; j < poses_per_mode.at(i); j++) {
      // The jth element in mode i, counting in between the start/end poses
      int modei_index = (j + 1) * (mode_sequence_.mode(i).num_knotpoints() - 1)
          /(poses_per_mode.at(i) + 1);

      vars.segment(index * plant_.num_positions(), plant_.num_positions()) = 
          state_vars(i, modei_index).head(plant_.num_positions());
      index++;
    } 
  }

  // Final state
  auto last_mode = mode_sequence_.mode(mode_sequence_.num_modes() - 1);
  vars.segment(index * plant_.num_positions(), plant_.num_positions()) = 
      state_vars(mode_sequence_.num_modes() - 1,
          last_mode.num_knotpoints() - 1).head(plant_.num_positions());

  // Create visualizer
  callback_visualizer_ = std::make_unique<multibody::MultiposeVisualizer>(
    model_file, num_poses, weld_frame_to_world);

  // Callback lambda function
  auto my_callback = [this, num_poses](const Eigen::Ref<const VectorXd>& vars) {
    VectorXd vars_copy = vars;
    Eigen::Map<MatrixXd> states(vars_copy.data(), this->plant_.num_positions(),
        num_poses);

    this->callback_visualizer_->DrawPoses(states);
  };

  AddVisualizationCallback(my_callback, vars);
}

template <typename T>
void Dircon<T>::CreateVisualizationCallback(std::string model_file,
      unsigned int num_poses, std::string weld_frame_to_world) {
  // Check that there is an appropriate number of poses
  DRAKE_DEMAND(num_poses >= (uint) mode_sequence_.num_modes() + 1);
  DRAKE_DEMAND(num_poses <= (uint) N());

  // sum of mode lengths, excluding ends
  int mode_sum = 0;
  for (int i_mode = 0; i_mode < mode_sequence_.num_modes(); i_mode++) {
    auto mode = mode_sequence_.mode(i_mode);
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
  unsigned int num_poses_without_ends =
      num_poses - mode_sequence_.num_modes() - 1;
  unsigned int assigned_sum = 0;
  std::vector<unsigned int> num_poses_per_mode;
  for (int i_mode = 0; i_mode < mode_sequence_.num_modes(); i_mode++) {
    if (mode_sequence_.mode(i_mode).num_knotpoints() > 2) {
      unsigned int mode_count = (num_poses_without_ends
            * (mode_sequence_.mode(i_mode).num_knotpoints() - 2)) / mode_sum;
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
    for (int i_mode = 0; i_mode < mode_sequence_.num_modes(); i_mode++) {
      double fractional_value = num_poses_without_ends
          * (mode_sequence_.mode(i_mode).num_knotpoints() - 2)
          - num_poses_per_mode.at(i_mode) * mode_sum;

      if (fractional_value > value) {
        value = fractional_value;
        largest_mode = i_mode;
      }
    }

    num_poses_per_mode.at(largest_mode)++;
    assigned_sum++;
  }

  for (auto& n : num_poses_per_mode)
    std::cout << n << std::endl;
  CreateVisualizationCallback(model_file, num_poses_per_mode,
      weld_frame_to_world);
}

template <typename T>
void Dircon<T>::CreateVisualizationCallback(std::string model_file,
      std::string weld_frame_to_world) {
  CreateVisualizationCallback(model_file, N(), weld_frame_to_world);
}

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace dairlib

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::systems::trajectory_optimization::Dircon)