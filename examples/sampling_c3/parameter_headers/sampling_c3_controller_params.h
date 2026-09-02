#pragma once

#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

#include <drake/common/yaml/yaml_io.h>
#include <optional>

#include "c3/core/solver_options_io.h"
#include "examples/sampling_c3/parameter_headers/goal_params.h"
#include "examples/sampling_c3/parameter_headers/progress_params.h"
#include "examples/sampling_c3/parameter_headers/reposition_params.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_options.h"
#include "examples/sampling_c3/parameter_headers/sampling_params.h"

#include "drake/common/yaml/yaml_read_archive.h"

struct SamplingC3ControllerParams {
  std::string sampling_c3_options_file;  // C3 mode params
  std::string reposition_params_file;    // repositioning mode params
  std::string progress_params_file;      // mode switching params
  std::string sampling_params_file;      // sampling params
  std::string goal_params_file;          // goal checking/setting params

  std::string sim_params_file;  // simulation params
  std::string vis_params_file;  // visualization params
  std::string osc_params_file;  // OSC params
  std::string osqp_settings_file;
  std::string osc_qp_settings_file;
  std::string franka_driver_channels_file;
  std::string lcm_channels_hardware_file;
  std::string lcm_channels_simulation_file;

  std::vector<std::string> object_models;
  std::vector<std::string> base_names;

  /// Optional per-goal-step keep-out geometry.  One entry per goal-sequence
  /// step; an empty string means "no keep-out regions for that step".  Each
  /// non-empty entry is a URDF path of world-locked obstacle geometry that
  /// samples must avoid while that goal is active.
  std::optional<std::vector<std::string>> keep_out_model_sequence;

  bool include_end_effector_orientation;
  int control_loop_delay_ms;

  int num_objects;

  /// Store sub-parameter classes internally.
  SamplingC3Options sampling_c3_options;
  SamplingC3RepositionParams reposition_params;
  SamplingC3ProgressParams progress_params;
  SamplingParams sampling_params;
  SamplingC3GoalParams goal_params;
  c3::SolverOptionsFromYaml osqp_settings;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(sampling_c3_options_file));
    a->Visit(DRAKE_NVP(reposition_params_file));
    a->Visit(DRAKE_NVP(progress_params_file));
    a->Visit(DRAKE_NVP(sampling_params_file));
    a->Visit(DRAKE_NVP(goal_params_file));
    a->Visit(DRAKE_NVP(sim_params_file));
    a->Visit(DRAKE_NVP(vis_params_file));
    a->Visit(DRAKE_NVP(osc_params_file));
    a->Visit(DRAKE_NVP(osqp_settings_file));
    a->Visit(DRAKE_NVP(osc_qp_settings_file));
    a->Visit(DRAKE_NVP(franka_driver_channels_file));
    a->Visit(DRAKE_NVP(lcm_channels_hardware_file));
    a->Visit(DRAKE_NVP(lcm_channels_simulation_file));
    a->Visit(DRAKE_NVP(include_end_effector_orientation));
    a->Visit(DRAKE_NVP(control_loop_delay_ms));

    a->Visit(DRAKE_NVP(base_names));
    a->Visit(DRAKE_NVP(object_models));
    a->Visit(DRAKE_NVP(keep_out_model_sequence));

    /// Store individual parameter classes internally.
    sampling_c3_options =
        drake::yaml::LoadYamlFile<SamplingC3Options>(sampling_c3_options_file);
    reposition_params = drake::yaml::LoadYamlFile<SamplingC3RepositionParams>(
        reposition_params_file);
    progress_params = drake::yaml::LoadYamlFile<SamplingC3ProgressParams>(
        progress_params_file);
    sampling_params =
        drake::yaml::LoadYamlFile<SamplingParams>(sampling_params_file);
    goal_params =
        drake::yaml::LoadYamlFile<SamplingC3GoalParams>(goal_params_file);
    osqp_settings = drake::yaml::LoadYamlFile<c3::SolverOptionsFromYaml>(
        osqp_settings_file);

    num_objects = base_names.size();

    ValidatePerGoalSequences();
  }

 private:
  /// Every optional per-goal-step "_sequence" parameter must have exactly one
  /// entry per goal-sequence step, and only makes sense when the goal generator
  /// is stepping through a fixed goal sequence.  Mirrors
  /// SamplingC3GoalParams::ValidateFixedGoalSequence().
  void ValidatePerGoalSequences() const {
    const int num_goal_steps =
        static_cast<int>(goal_params.fixed_target_position_sequence.size());
    const bool is_goal_sequence =
        goal_params.goal_mode == GoalMode::kFixedGoalSequence;

    auto check = [&](const char* field_name, std::size_t actual_size) {
      if (!is_goal_sequence) {
        throw std::runtime_error(
            std::string(field_name) +
            " is set, but goal_mode is not kFixedGoalSequence (3); a per-goal "
            "sequence only makes sense with a goal sequence.");
      }
      if (static_cast<int>(actual_size) != num_goal_steps) {
        throw std::runtime_error(
            std::string(field_name) + " has " + std::to_string(actual_size) +
            " entries but there are " + std::to_string(num_goal_steps) +
            " goal-sequence steps (fixed_target_position_sequence); the "
            "lengths "
            "must match.");
      }
    };

    if (progress_params.cost_switching_threshold_distance_sequence
            .has_value()) {
      check("cost_switching_threshold_distance_sequence",
            progress_params.cost_switching_threshold_distance_sequence->size());
    }
    if (keep_out_model_sequence.has_value()) {
      check("keep_out_model_sequence", keep_out_model_sequence->size());
    }
  }
};
