#include "systems/controllers/elastoplastic_sampling_c3_controller.h"

#include "c3/core/c3_miqp.h"
#include "c3/core/c3_plus.h"
#include "c3/core/c3_qp.h"
#include "c3/core/traj_eval.h"
#include "dairlib/lcmt_radio_out.hpp"
#include "examples/sampling_c3/generate_samples.h"

#include "drake/common/drake_assert.h"

namespace dairlib {

using c3::C3;
using c3::C3MIQP;
using c3::C3Plus;
using c3::C3QP;
using c3::ElastoPlasticLCSFactoryOptions;
using c3::LCS;
using c3::multibody::ElastoPlasticLCSFactory;
using c3::multibody::LCSFactory;
using c3::traj_eval::TrajectoryEvaluator;
using drake::SortedPair;
using drake::geometry::FrameId;
using drake::geometry::GeometryId;
using drake::geometry::QueryObject;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using Eigen::MatrixXd;
using Eigen::RowVectorXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::vector;

namespace systems {

ElastoPlasticSC3Controller::ElastoPlasticSC3Controller(
    drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::systems::Context<drake::AutoDiffXd>* context_ad,
    const vector<vector<SortedPair<GeometryId>>>& external_contact_pair_lists,
    const vector<GeometryId>& internal_contact_geometries,
    DeformControllerParams controller_params, bool verbose)
    : SamplingC3Controller(
          plant, context, plant_ad, context_ad, external_contact_pair_lists,
          std::move(controller_params.sampling_c3_controller_params), verbose,
          3 * controller_params.elastoplastic_sc3_options.num_internal_contacts,
          /*declare_actor_object_plan_ports=*/false),
      goal_params_(controller_params.elastoplastic_goal_params),
      internal_contact_geometries_(internal_contact_geometries),
      n_nodes_(internal_contact_geometries.size()),
      elastoplastic_sc3_options_(controller_params.elastoplastic_sc3_options) {
  this->set_name("elastoplastic_sc3_controller");

  // Edit some of the work of the parent constructor.
  dt_ = elastoplastic_sc3_options_.planning_dt_pose;  // Always in pose mode.
  // Use GetC3Options() (i.e. c3_options_pose) rather than the inherited
  // C3Options::G/U, which C3Options::Serialize builds incorrectly for the
  // elastoplastic case: it bakes g_internal_slack/sigma into g_lambda before
  // g_u and g_eta are separated, producing a 165x165 matrix instead of 205x205.
  G_ = vector<MatrixXd>(N_, elastoplastic_sc3_options_.GetC3Options().G);
  U_ = vector<MatrixXd>(N_, elastoplastic_sc3_options_.GetC3Options().U);
  // Need to rebuild C3 plan objects since they use differently sized LCS.
  auto lcs_placeholder = LCS::CreatePlaceholderLCS(
      n_x_, n_u_, n_lambda_ + n_lambda_internal_, N_, dt_);
  auto x_desired_placeholder = vector<VectorXd>(N_ + 1, VectorXd::Zero(n_x_));
  C3Options c3_options = elastoplastic_sc3_options_.GetC3Options();
  if (elastoplastic_sc3_options_.projection_type == "MIQP") {
    c3_curr_plan_ = std::make_unique<C3MIQP>(lcs_placeholder,
                                             C3::CostMatrices(Q_, R_, G_, U_),
                                             x_desired_placeholder, c3_options);
    c3_best_plan_ = std::make_unique<C3MIQP>(lcs_placeholder,
                                             C3::CostMatrices(Q_, R_, G_, U_),
                                             x_desired_placeholder, c3_options);
    c3_buffer_plan_ = std::make_unique<C3MIQP>(
        lcs_placeholder, C3::CostMatrices(Q_, R_, G_, U_),
        x_desired_placeholder, c3_options);
  } else if (elastoplastic_sc3_options_.projection_type == "QP") {
    c3_curr_plan_ = std::make_unique<C3QP>(lcs_placeholder,
                                           C3::CostMatrices(Q_, R_, G_, U_),
                                           x_desired_placeholder, c3_options);
    c3_best_plan_ = std::make_unique<C3QP>(lcs_placeholder,
                                           C3::CostMatrices(Q_, R_, G_, U_),
                                           x_desired_placeholder, c3_options);
    c3_buffer_plan_ = std::make_unique<C3QP>(lcs_placeholder,
                                             C3::CostMatrices(Q_, R_, G_, U_),
                                             x_desired_placeholder, c3_options);
  } else if (elastoplastic_sc3_options_.projection_type == "C3+") {
    c3_curr_plan_ = std::make_unique<C3Plus>(lcs_placeholder,
                                             C3::CostMatrices(Q_, R_, G_, U_),
                                             x_desired_placeholder, c3_options);
    c3_best_plan_ = std::make_unique<C3Plus>(lcs_placeholder,
                                             C3::CostMatrices(Q_, R_, G_, U_),
                                             x_desired_placeholder, c3_options);
    c3_buffer_plan_ = std::make_unique<C3Plus>(
        lcs_placeholder, C3::CostMatrices(Q_, R_, G_, U_),
        x_desired_placeholder, c3_options);
  }  // No need to throw error otherwise since done by parent constructor.
  n_z_ = c3_curr_plan_->GetZSize();

  // Look up each node's BodyIndex once (topology is fixed; only mass changes).
  // The same BodyIndex numbering is shared between plant_ and plant_ad_.
  const auto& query_object =
      plant_.get_geometry_query_input_port().template Eval<QueryObject<double>>(
          *context_);
  for (const auto& geom_id : internal_contact_geometries_) {
    FrameId frame_id = query_object.inspector().GetFrameId(geom_id);
    node_body_indices_.push_back(plant_.GetBodyFromFrameId(frame_id)->index());
  }

  // New input port for elastoplastic network.
  elastoplastic_input_port_ =
      this->DeclareAbstractInputPort(
              "elastoplastic_network",
              drake::Value<dairlib::lcmt_elastoplastic_network>{})
          .get_index();

  DeclareForcedDiscreteUpdateEvent(&ElastoPlasticSC3Controller::ComputePlan);

  // Use sampling and repositioning strategies compatible with the state
  // representation of a deformable object.
  DRAKE_DEMAND(sampling_params_.sampling_strategy ==
                   SamplingStrategy::kRandomOnSphereAroundDeformable ||
               sampling_params_.sampling_strategy ==
                   SamplingStrategy::kRandomAroundDeformableFixedDistance);
  DRAKE_DEMAND(reposition_params_.traj_type ==
               RepositioningTrajectoryType::kPiecewiseLinear);

  std::cout << "Initialized ElastoPlasticSC3Controller" << std::endl;
}

drake::systems::EventStatus ElastoPlasticSC3Controller::ComputePlan(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  auto start = std::chrono::high_resolution_clock::now();

  // Evaluate input ports.
  const auto& radio_out =
      this->EvalInputValue<dairlib::lcmt_radio_out>(context, radio_port_);
  const BasicVector<double>& x_lcs_des =
      *this->template EvalVectorInput<BasicVector>(context, target_input_port_);
  const BasicVector<double>& x_lcs_final_des =
      *this->template EvalVectorInput<BasicVector>(context,
                                                   final_target_input_port_);
  const auto& elastoplastic_network_lcmt =
      this->EvalInputValue<dairlib::lcmt_elastoplastic_network>(
          context, elastoplastic_input_port_);
  const TimestampedVector<double>* lcs_x_curr =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        lcs_state_input_port_);
  // Store the current LCS state.
  drake::VectorX<double> x_lcs_curr = lcs_x_curr->get_data();
  ee_position_curr_ = x_lcs_curr.segment(0, 3);

  // Use a predicted EE state, if desired.
  bool is_teleop = radio_out->channel[14];
  ResolvePredictedEEState(is_teleop, x_lcs_curr);

  discrete_state->get_mutable_value(plan_start_time_index_)[0] =
      lcs_x_curr->get_timestamp();

  // Check for workspace limit violations; if any, the controller stops.
  CheckForWorkspaceLimitViolations(lcs_x_curr);

  // Update the node masses in the plant to reflect the current state of the
  // deformable network.
  SetNodeMasses(*elastoplastic_network_lcmt);

  // Compute the current deformable graph node position errors.
  current_node_error_ = (x_lcs_curr.segment(3, n_q_ - 3) -
                         x_lcs_final_des.get_value().segment(3, n_q_ - 3))
                            .norm();

  // Detect if the final target has changed, in which case return to caring only
  // about position until the switching threshold has been crossed again.
  // Exclude the EE goal from the comparison, since that always changes to be
  // above the current object location.
  if (!x_final_target_.segment(3, n_x_ - 3)
           .isApprox(x_lcs_final_des.value().segment(3, n_x_ - 3), 1e-5)) {
    std::cout << "Detected goal change!" << std::endl;
    x_final_target_ = x_lcs_final_des.value();
    detected_goal_changes_++;

    // Reset the sample buffers now that the costs have changed.
    ResetSampleBuffers();
  }

  // TODO @bibit:  does this need to be ElastoPlasticLCSFactoryOptions?
  // Build C3Options and LCSFactoryOptions.
  C3Options c3_options = elastoplastic_sc3_options_.GetC3Options();
  LCSFactoryOptions lcs_factory_options =
      elastoplastic_sc3_options_.GetLCSFactoryOptions();

  // Update the cost matrices:  Q_, R_, G_, and U_.
  UpdateCostMatrices(x_lcs_curr, x_lcs_des, c3_options);

  // Generate states, differing from the current state only by EE sample
  // locations.
  vector<bool> node_on_target;
  bool all_reached = true;
  for (int i = 0; i < n_nodes_; i++) {
    double node_position_error = (x_lcs_curr.segment(3 + 3 * i, 3) -
                                  x_lcs_des.get_value().segment(3 + 3 * i, 3))
                                     .norm();
    node_on_target.push_back(
        (node_position_error < goal_params_.node_success_threshold));
    all_reached = all_reached && node_on_target[i];
  }

  // Used fixed samples if fixed goal and all objects on target.
  vector<VectorXd> candidate_states;
  if (achieved_fixed_goal_ ||
      (all_reached && goal_params_.goal_mode == GoalMode::kFixedGoal)) {
    achieved_fixed_goal_ = true;
    int num_samples = is_doing_c3_
                          ? sampling_params_.num_additional_samples_c3
                          : sampling_params_.num_additional_samples_repos;
    for (int i = 0; i < num_samples; i++) {
      candidate_states.push_back(x_lcs_curr);
      candidate_states[i].head(3) << 0.3, 0.4, 0.1;  // TODO: fixed EE end spot
    }
  }
  // Generate new samples according to sampling strategy.
  else {
    candidate_states = GenerateSampleStates(
        n_q_, n_v_, n_u_, x_lcs_curr, is_doing_c3_, sampling_params_,
        elastoplastic_sc3_options_, plant_, context_, plant_ad_, context_ad_,
        contact_pairs_, faces_, face_bins_, faces_per_object_,
        face_bins_per_object_, total_area_per_object_, node_on_target,
        unsuccessful_sample_buffer_, internal_contact_geometries_);
  }

  // Add the previous best repositioning target to the candidate states at the
  // index 1 always. (Index 0 will become the current state.)
  if (!is_doing_c3_) {
    VectorXd repositioning_target_state = x_lcs_curr;
    repositioning_target_state.head(3) = prev_repositioning_target_;
    candidate_states.insert(candidate_states.begin(),
                            repositioning_target_state);
  }
  // Insert the current location at the beginning of the candidate states.
  candidate_states.insert(candidate_states.begin(), x_lcs_curr);
  int num_total_samples = candidate_states.size();

  // Update the set of sample locations under consideration.
  all_sample_locations_.clear();
  for (int i = 0; i < num_total_samples; i++) {
    all_sample_locations_.push_back(candidate_states[i].head(3));
  }
  // Make LCS objects for each sample.
  auto [internal_contact_pairs, yield_forces] =
      GetCurrentElastoPlasticProperties(*elastoplastic_network_lcmt);
  auto lcs_pair = ElastoPlasticSC3Controller::CreateLCSObjectsForSamples(
      candidate_states, x_lcs_curr, internal_contact_pairs, yield_forces,
      elastoplastic_sc3_options_.ep_lcs_factory_options);
  vector<LCS> lcs_candidates = lcs_pair.first;
  vector<LCS> lcs_candidates_for_cost = lcs_pair.second;

  // Prepare variables that will get used or filled in by parallelization.
  all_sample_costs_ = vector<double>(num_total_samples, -1);
  all_sample_dynamically_feasible_plans_ = vector<vector<VectorXd>>(
      num_total_samples, vector<VectorXd>(N_ + 1, VectorXd::Zero(n_x_)));
  vector<std::shared_ptr<C3>> c3_objects(num_total_samples, nullptr);
  bool force_tracking_disabled = radio_out->channel[11];
  C3CostComputationType cost_type = progress_params_.cost_type;

  // Parallelize over computing C3 costs for each sample.
  auto c3_start = std::chrono::high_resolution_clock::now();
#pragma omp parallel for num_threads(num_threads_to_use_)
  for (int i = 0; i < num_total_samples; i++) {
    bool print_cost_breakdown =
        radio_out->channel[7] && (i == SampleIndex::kCurrentLocation);

    // Get the candidate state and its LCS representation.
    VectorXd test_state = candidate_states.at(i);
    LCS test_system = lcs_candidates.at(i);

    // Set up C3 with proper projection type and post-solve cost matrices.
    std::shared_ptr<C3> test_c3_object;
    vector<VectorXd> x_desired(N_ + 1, x_lcs_des.value());

    C3::CostMatrices c3_costmat(Q_, R_, G_, U_);
    if (elastoplastic_sc3_options_.projection_type == "MIQP") {
      test_c3_object = std::make_shared<C3MIQP>(test_system, c3_costmat,
                                                x_desired, c3_options);
    } else if (elastoplastic_sc3_options_.projection_type == "QP") {
      test_c3_object = std::make_shared<C3QP>(test_system, c3_costmat,
                                              x_desired, c3_options);
    } else if (elastoplastic_sc3_options_.projection_type == "C3+") {
      test_c3_object = std::make_shared<C3Plus>(test_system, c3_costmat,
                                                x_desired, c3_options);
    }  // Unknown projection types are rejected in the initialization.

    if (!elastoplastic_sc3_options_.include_walls) {
      // Set actor bounds.
      for (int i = 0; i < elastoplastic_sc3_options_.workspace_limits.size();
           ++i) {
        RowVectorXd A = VectorXd::Zero(n_x_);
        A.segment(0, 3) =
            elastoplastic_sc3_options_.workspace_limits[i].segment(0, 3);
        test_c3_object->AddLinearConstraint(
            A,
            elastoplastic_sc3_options_.workspace_limits[i][3] -
                elastoplastic_sc3_options_.workspace_margins,
            elastoplastic_sc3_options_.workspace_limits[i][4] +
                elastoplastic_sc3_options_.workspace_margins,
            c3::ConstraintVariable::STATE);
      }
      // Set object bounds
      for (int i = 0; i < elastoplastic_sc3_options_.workspace_limits.size();
           ++i) {
        for (int j = 0; j < controller_params_.num_objects; j++) {
          RowVectorXd A = VectorXd::Zero(n_x_);
          A.segment(7 + 7 * j, 3) =
              elastoplastic_sc3_options_.workspace_limits[i].segment(0, 3);
          test_c3_object->AddLinearConstraint(
              A,
              elastoplastic_sc3_options_.workspace_limits[i][3] -
                  elastoplastic_sc3_options_.workspace_margins,
              elastoplastic_sc3_options_.workspace_limits[i][4] +
                  elastoplastic_sc3_options_.workspace_margins,
              c3::ConstraintVariable::STATE);
        }
      }
    }

    // Add constraint on end-effector velocities
    for (int i : vector<int>({0, 1, 2})) {
      RowVectorXd A = VectorXd::Zero(n_x_);
      A(n_q_ + i) = 1.0;
      test_c3_object->AddLinearConstraint(
          A, elastoplastic_sc3_options_.ee_velocity_limits[0],
          elastoplastic_sc3_options_.ee_velocity_limits[1],
          c3::ConstraintVariable::STATE);
    }

    // Add input constraints.
    for (int i : vector<int>({0, 1})) {
      RowVectorXd A = VectorXd::Zero(n_u_);
      A(i) = 1.0;
      test_c3_object->AddLinearConstraint(
          A, elastoplastic_sc3_options_.u_horizontal_limits[0],
          elastoplastic_sc3_options_.u_horizontal_limits[1],
          c3::ConstraintVariable::INPUT);
    }
    for (int i : vector<int>({2})) {
      RowVectorXd A = VectorXd::Zero(n_u_);
      A(i) = 1.0;
      test_c3_object->AddLinearConstraint(
          A, elastoplastic_sc3_options_.u_vertical_limits[0],
          elastoplastic_sc3_options_.u_vertical_limits[1],
          c3::ConstraintVariable::INPUT);
    }

    // Solve C3, store resulting object and cost.
    test_c3_object->SetSolverOptions(solver_options_);
    test_c3_object->Solve(test_state);

    std::pair<double, vector<VectorXd>> cost_trajectory_pair = CalcCost(
        cost_type, lcs_candidates_for_cost.at(i), c3_costmat, test_c3_object,
        force_tracking_disabled, print_cost_breakdown || verbose_);

    double c3_cost = cost_trajectory_pair.first;
    all_sample_dynamically_feasible_plans_.at(i) = cost_trajectory_pair.second;

#pragma omp critical
    {
      c3_objects.at(i) = test_c3_object;
    }
    // Add travel cost (just looking at xy displacement, not also z).
    double xy_travel_distance =
        (test_state.head(2) - x_lcs_curr.head(2)).norm();
    all_sample_costs_[i] =
        c3_cost + progress_params_.travel_cost_per_meter * xy_travel_distance;

    // Add additional costs based on repositioning progress.
    if ((i == SampleIndex::kCurrentReposTarget) && finished_reposition_flag_) {
      all_sample_costs_[i] += progress_params_.finished_reposition_cost;
      finished_reposition_flag_ = false;
    }
  }
  auto c3_end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> duration_ms = c3_end - c3_start;
  // End of parallelization

  // Update the sample buffer.  Do this before switching modes since 1) if in
  // repositioning mode, don't add the repositioning target over and over again,
  // and 2) since the best sample in the buffer may be the best sample overall
  // and could be considered as a repositioning target.
  MaintainSampleBuffers(x_lcs_curr);

  // Augment the considered samples with the best from the buffer, if eligible.
  AugmentSamplesWithBuffer(c3_objects);

  // Set up hysteresis values.
  double hyst_c3_to_repos = progress_params_.hyst_c3_to_repos;
  double hyst_repos_to_c3 = progress_params_.hyst_repos_to_c3;
  double hyst_repos_to_repos = progress_params_.hyst_repos_to_repos;
  double hyst_c3_to_repos_frac = progress_params_.hyst_c3_to_repos_frac;
  double hyst_repos_to_c3_frac = progress_params_.hyst_repos_to_c3_frac;
  double hyst_repos_to_repos_frac = progress_params_.hyst_repos_to_repos_frac;

  // Review the cost results to determine the best sample.
  bool force_c3_mode = radio_out->channel[12];
  double best_other_cost;
  if (num_total_samples > 1) {
    vector<double> additional_sample_cost_vector =
        vector<double>(all_sample_costs_.begin() + 1, all_sample_costs_.end());
    best_other_cost = *std::min_element(additional_sample_cost_vector.begin(),
                                        additional_sample_cost_vector.end());
    vector<double>::iterator it =
        std::min_element(std::begin(additional_sample_cost_vector),
                         std::end(additional_sample_cost_vector));
    best_sample_index_ =
        (SampleIndex)(std::distance(std::begin(additional_sample_cost_vector),
                                    it) +
                      1);
  } else {
    force_c3_mode = true;
  }

  // Determine whether to do C3 or reposition.
  mode_switch_reason_ = ModeSwitchReason::kNoSwitch;
  double curr_cost = all_sample_costs_[SampleIndex::kCurrentLocation];
  double repos_target_cost =
      all_sample_costs_[SampleIndex::kCurrentReposTarget];
  if (is_doing_c3_ == true) {  // Currently doing C3.
    pursued_target_source_ = PursuedTargetSource::kNoTarget;

    // Keep track of progress while in C3 mode.
    bool met_minimum_progress = true;  // Reset by below function.
    bool print_current_node_cost = radio_out->channel[6];
    KeepTrackOfC3ModeProgress(x_lcs_curr, x_lcs_final_des, met_minimum_progress,
                              print_current_node_cost);

    // Switch to repositioning if fixed goals have all been met.
    if (achieved_fixed_goal_) {
      is_doing_c3_ = false;
      std::cout << "All objects on target, switching to repositioning mode"
                << std::endl;
    }
    // Switch to repositioning if progress was insufficient.
    else if (!met_minimum_progress && !force_c3_mode &&
             (sampling_params_.num_additional_samples_c3 > 0)) {
      is_doing_c3_ = false;
      mode_switch_reason_ = ModeSwitchReason::kToReposUnproductive;
      std::cout << "Repositioning after not making progress in C3" << std::endl;
    }

    // Switch to repositioning if one of the other samples is better, with
    // hysteresis.
    else if (((!progress_params_.use_relative_hysteresis &&
               curr_cost > best_other_cost + hyst_c3_to_repos) ||
              (progress_params_.use_relative_hysteresis &&
               curr_cost >
                   best_other_cost + hyst_c3_to_repos_frac * curr_cost)) &&
             !force_c3_mode) {
      is_doing_c3_ = false;
      mode_switch_reason_ = ModeSwitchReason::kToReposCost;
      std::cout << "Repositioning because found good sample" << std::endl;
    }

    // Reset progress metrics if switching to repositioning.
    if (!is_doing_c3_) {
      finished_reposition_flag_ = false;
      ResetProgressMetrics();

      // Determine the source of the repositioning target.
      if (best_sample_index_ > sampling_params_.num_additional_samples_c3) {
        pursued_target_source_ = PursuedTargetSource::kFromBuffer;
        // Remove the sample from the buffer.
        sample_buffer_.row(num_in_buffer_ - 1) = VectorXd::Zero(n_q_);
        sample_costs_buffer_[num_in_buffer_ - 1] = -1;
        num_in_buffer_--;
      } else {
        pursued_target_source_ = PursuedTargetSource::kNewSample;
      }
    }
  } else {  // Currently repositioning.
    // First, apply hysteresis between repositioning targets.
    if (best_sample_index_ == SampleIndex::kCurrentReposTarget) {
      pursued_target_source_ = PursuedTargetSource::kPrevious;
    } else {
      // This means there is a lower cost sample other than the current
      // repositioning target. If the lowest cost sample is not at least the
      // hysteresis amount better than the current repositioning target, then
      // continue pursuing the previous repositioning target.
      if ((repos_target_cost < best_other_cost + hyst_repos_to_repos &&
           !progress_params_.use_relative_hysteresis) ||
          (repos_target_cost <
               best_other_cost + hyst_repos_to_repos_frac * repos_target_cost &&
           progress_params_.use_relative_hysteresis)) {
        best_sample_index_ = SampleIndex::kCurrentReposTarget;
        best_other_cost = repos_target_cost;
        finished_reposition_flag_ = false;
        pursued_target_source_ = PursuedTargetSource::kPrevious;
      }
      // Controller will switch to pursuing a new sample from its previous
      // repositioning target only if the cost of switching to that new sample
      // (with repos_to_repos hysteresis) is less than switching to C3 from
      // current location (with repos_to_c3 hysteresis), so add the
      // repos_to_repos hysteresis value here before the comparison to the
      // current location C3 cost with repos_to_c3 hysteresis afterwards.
      else {
        pursued_target_source_ = PursuedTargetSource::kNewSample;
        if (!progress_params_.use_relative_hysteresis) {
          best_other_cost += hyst_repos_to_repos;
        } else {
          best_other_cost += hyst_repos_to_repos_frac * repos_target_cost;
        }
      }
    }

    double wall_offset = 0;

    if (elastoplastic_sc3_options_.include_walls &&
        sampling_params_.sample_on_wall) {
      double x_min = elastoplastic_sc3_options_.workspace_limits[0][3];
      double x_max = elastoplastic_sc3_options_.workspace_limits[0][4];
      double y_min = elastoplastic_sc3_options_.workspace_limits[1][3];
      double y_max = elastoplastic_sc3_options_.workspace_limits[1][4];

      // if ee is close to wall, raise z_height to avoid hitting
      if ((x_lcs_curr[0] <= x_min + 0.05 &&
           x_lcs_curr[0] >=
               x_min - elastoplastic_sc3_options_.workspace_margins) ||
          (x_lcs_curr[0] >= x_max - 0.05 &&
           x_lcs_curr[0] <=
               x_max + elastoplastic_sc3_options_.workspace_margins) ||
          (x_lcs_curr[1] <= y_min + 0.05 &&
           x_lcs_curr[1] >=
               y_min - elastoplastic_sc3_options_.workspace_margins) ||
          (x_lcs_curr[1] >= y_max - 0.05 &&
           x_lcs_curr[1] <=
               y_max + elastoplastic_sc3_options_.workspace_margins)) {
        wall_offset = 0.01;
      }
    }

    // Switch to C3 if forced by xbox controller.
    if (force_c3_mode) {
      std::cout << "Forcing into C3 mode" << std::endl;
      is_doing_c3_ = true;
      mode_switch_reason_ = ModeSwitchReason::kToC3Xbox;
      pursued_target_source_ = PursuedTargetSource::kNoTarget;
      // Add the current state to the unsuccessful sample buffer.  It gets
      // automatically removed if the object moves beyond the buffer movement
      // thresholds.
      AddToUnsuccessfulBuffer(candidate_states[0]);
    }
    // Stay in repositioning if fixed goal is met.
    else if (achieved_fixed_goal_) {
      finished_reposition_flag_ = false;
      std::cout << "All objects at fixed goals; stay out of the way."
                << std::endl;
    }
    // Switch to C3 if the current sample is better, with hysteresis.
    else if (((!progress_params_.use_relative_hysteresis &&
               best_other_cost > curr_cost + hyst_repos_to_c3) ||
              (progress_params_.use_relative_hysteresis &&
               best_other_cost >
                   curr_cost + hyst_repos_to_c3_frac * best_other_cost)) &&
             (x_lcs_curr[2] < sampling_params_.z_height +
                                  sampling_params_.c3_min_clearance +
                                  wall_offset ||
              !sampling_params_.ee_z_close)) {
      is_doing_c3_ = true;
      finished_reposition_flag_ = false;
      if (repos_target_cost > progress_params_.finished_reposition_cost) {
        mode_switch_reason_ = ModeSwitchReason::kToC3ReachedReposTarget;
        std::cout << "Switching to C3 because reached repositioning target"
                  << std::endl;
        // Add the repositioning target to the unsuccessful sample buffer.  It
        // gets automatically removed if the object moves beyond the buffer
        // movement thresholds.
        AddToUnsuccessfulBuffer(
            candidate_states[SampleIndex::kCurrentReposTarget]);
      } else {
        mode_switch_reason_ = ModeSwitchReason::kToC3Cost;
        std::cout << "Switching to C3 because lower in cost" << std::endl;
        // Add the current state to the unsuccessful sample buffer.  It gets
        // automatically removed if the object moves beyond the buffer movement
        // thresholds.
        AddToUnsuccessfulBuffer(candidate_states[0]);
      }
      pursued_target_source_ = PursuedTargetSource::kNoTarget;
    }
  }

  // Update C3 objects and intermediates for current and best samples.
  c3_curr_plan_ = c3_objects.at(SampleIndex::kCurrentLocation);
  c3_best_plan_ = c3_objects.at(best_sample_index_);

  // TODO If doing warmstarting, will need to save z, delta, and w vectors.

  // Update the execution trajectories.
  double t = context.get_discrete_state(plan_start_time_index_)[0];
  UpdateC3ExecutionTrajectory(x_lcs_curr, t);
  UpdateRepositioningExecutionTrajectory(x_lcs_curr, t);

  // Add delay.
  std::this_thread::sleep_for(
      std::chrono::milliseconds(controller_params_.control_loop_delay_ms));

  // End of control loop cleanup.
  auto finish = std::chrono::high_resolution_clock::now();
  auto elapsed = finish - start;
  double solve_time =
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() /
      1e6;
  filtered_solve_time_ = (1 - solve_time_filter_constant_) * solve_time +
                         (solve_time_filter_constant_)*filtered_solve_time_;

  return drake::systems::EventStatus::Succeeded();
}

// TODO @bibit:  This implementation barely differs from that of
// SamplingC3Controller; consider refactoring to avoid code duplication.
std::pair<double, vector<VectorXd>> ElastoPlasticSC3Controller::CalcCost(
    C3CostComputationType cost_type, const c3::LCS& lcs_for_cost,
    const c3::C3::CostMatrices& cost_mats,
    const std::shared_ptr<c3::C3>& c3_object,
    const bool& force_tracking_disabled,
    const bool& print_cost_breakdown) const {
  // Extract needed information from the C3 object.
  const LCS lcs_for_plan = c3_object->GetLCS();
  vector<VectorXd> x_desired = c3_object->GetDesiredState();
  vector<VectorXd> x_plan = c3_object->GetStateSolution();
  vector<VectorXd> u_plan = c3_object->GetInputSolution();
  vector<VectorXd> lambda_plan = c3_object->GetForceSolution();

  // TODO @bibit: The original CalcCost extracts the x and u trajectories from
  // the C3 object's z_sol_ vector, which can differ from the C3 object's
  // x_sol_ and u_sol_ vectors if end_on_qp_step is false.  This may be
  // considered a bug in C3, but for now, extract the same trajectories to
  // maintain functionality.  If C3 is fixed so that x_sol_ and u_sol_ (and
  // lambda_sol_) always reflect the trajectories corresponding to z_sol_, then
  // the following 5 lines can be removed since x_plan and u_plan will already
  // be correct from the above getters.
  vector<VectorXd> z_plan = c3_object->GetFullSolution();
  for (int i = 0; i < N_; i++) {
    x_plan[i] = z_plan[i].segment(0, n_x_);
    lambda_plan[i] = z_plan[i].segment(n_x_, n_lambda_ + n_lambda_internal_);
    u_plan[i] = z_plan[i].segment(n_x_ + n_lambda_ + n_lambda_internal_, n_u_);
  }
  DRAKE_THROW_UNLESS(z_plan.size() == N_);
  DRAKE_THROW_UNLESS(x_plan.size() == N_);
  DRAKE_THROW_UNLESS(u_plan.size() == N_);

  // The x_plan from the C3 object does not include the x_N state, so add it in
  // using the LCS rollout from the last x, u, and lambda.
  MatrixXd A_N = lcs_for_plan.A().back();
  MatrixXd B_N = lcs_for_plan.B().back();
  MatrixXd D_N = lcs_for_plan.D().back();
  VectorXd d_N = lcs_for_plan.d().back();
  x_plan.push_back(A_N * x_plan.back() + B_N * u_plan.back() +
                   D_N * lambda_plan.back() + d_N);

  // Initialize the cost-driving trajectories to match the C3 plan.
  vector<VectorXd> XX = x_plan;
  vector<VectorXd> UU = u_plan;

  // Declare the matrices to use for cost computation.
  vector<MatrixXd> Q_cost = cost_mats.Q;
  vector<MatrixXd> R_cost = cost_mats.R;

  // Set a few more variables necessary for some of the cost types.
  auto simulate_config = c3::LCSSimulateConfig();
  simulate_config.regularized = true;
  simulate_config.min_exp = -8;

  // Compute the states and controls to use for cost computation, and change the
  // cost matrices if necessary.
  if (cost_type == C3CostComputationType::kSimLCS) {
    // Simulate the LCS from initial condition using the C3 plan's controls.
    XX = TrajectoryEvaluator::SimulateLCSOverTrajectory(
        x_plan[0], u_plan, lcs_for_plan, lcs_for_cost, simulate_config);

  } else if (cost_type == C3CostComputationType::kUseC3Plan) {
    // No need to do anything here.

  } else if (cost_type == C3CostComputationType::kSimLCSReplaceC3EEPlan) {
    // Simulate the LCS from initial condition using the C3 plan's controls.
    vector<VectorXd> XX_sim = TrajectoryEvaluator::SimulateLCSOverTrajectory(
        x_plan[0], u_plan, lcs_for_plan, lcs_for_cost, simulate_config);

    // Use the simulated object trajectories but the planned robot trajectory.
    for (int i = 0; i < N_ + 1; i++) {
      XX[i].segment(3, n_q_ - 3) = XX_sim[i].segment(3, n_q_ - 3);
      XX[i].segment(n_q_ + 3, n_v_ - 3) = XX_sim[i].segment(n_q_ + 3, n_v_ - 3);
    }

  } else if (cost_type == C3CostComputationType::kSimImpedance) {
    // Simulate PD with feedforward control using the C3 plan's states and
    // controls from the initial condition.
    auto [XX_sim, UU_sim] = TrajectoryEvaluator::SimulatePDControlWithLCS(
        x_plan, UU, Kp_for_cost_, Kd_for_cost_, lcs_for_plan, lcs_for_cost,
        !force_tracking_disabled, simulate_config);
    XX = XX_sim;
    UU = UU_sim;

  } else if (cost_type == C3CostComputationType::kSimImpedanceReplaceC3EEPlan) {
    // Simulate PD with feedforward control using the C3 plan's states and
    // controls from the initial condition.
    auto [XX_sim, UU_sim] = TrajectoryEvaluator::SimulatePDControlWithLCS(
        x_plan, UU, Kp_for_cost_, Kd_for_cost_, lcs_for_plan, lcs_for_cost,
        !force_tracking_disabled, simulate_config);
    UU = UU_sim;

    // Use the simulated object trajectories but the planned robot trajectory.
    for (int i = 0; i < N_ + 1; i++) {
      XX[i].segment(3, n_q_ - 3) = XX_sim[i].segment(3, n_q_ - 3);
      XX[i].segment(n_q_ + 3, n_v_ - 3) = XX_sim[i].segment(n_q_ + 3, n_v_ - 3);
    }

  } else if (cost_type == C3CostComputationType::kSimImpedanceObjectCostOnly) {
    // Simulate PD with feedforward control using the C3 plan's states and
    // controls from the initial condition.
    auto [XX_sim, UU_sim] = TrajectoryEvaluator::SimulatePDControlWithLCS(
        x_plan, UU, Kp_for_cost_, Kd_for_cost_, lcs_for_plan, lcs_for_cost,
        !force_tracking_disabled, simulate_config);
    XX = XX_sim;
    UU = UU_sim;

    // Set R and the robot portion of the Q matrix to zero so that only the
    // object state errors contribute to cost.
    for (int i = 0; i < N_ + 1; i++) {
      Q_cost[i].block(0, 0, 3, 3) *= 0.0;
      Q_cost[i].block(n_q_, n_q_, 3, 3) *= 0.0;
      if (i < N_) {
        R_cost[i] *= 0.0;
      }
    }
  }

  // Compute the cost.
  double cost = TrajectoryEvaluator::ComputeQuadraticTrajectoryCost(
      XX, x_desired, Q_cost, UU, R_cost);

  if (print_cost_breakdown) {
    std::cout << "===== NEW COST BREAKDOWN =====" << std::endl;
    // Errors
    MatrixXd Q_identity = MatrixXd::Identity(n_x_, n_x_);
    double error_contrib_ee_pos =
        TrajectoryEvaluator::ComputeQuadraticTrajectoryCost(0, 3, XX, x_desired,
                                                            Q_identity);
    double error_contrib_ee_vel =
        TrajectoryEvaluator::ComputeQuadraticTrajectoryCost(
            n_q_, n_q_ + 3, XX, x_desired, Q_identity);

    double error_contrib_obj_pos =
        TrajectoryEvaluator::ComputeQuadraticTrajectoryCost(
            3, n_q_ - 3, XX, x_desired, Q_identity);
    double error_contrib_obj_vel =
        TrajectoryEvaluator::ComputeQuadraticTrajectoryCost(
            n_v_ + 3, n_v_ - 3, XX, x_desired, Q_identity);

    // Costs
    double cost_contrib_ee_pos =
        TrajectoryEvaluator::ComputeQuadraticTrajectoryCost(0, 3, XX, x_desired,
                                                            Q_cost);
    double cost_contrib_ee_vel =
        TrajectoryEvaluator::ComputeQuadraticTrajectoryCost(n_q_, n_q_ + 3, XX,
                                                            x_desired, Q_cost);
    double cost_contrib_u =
        TrajectoryEvaluator::ComputeQuadraticTrajectoryCost(UU, R_cost);

    double cost_contrib_obj_pos =
        TrajectoryEvaluator::ComputeQuadraticTrajectoryCost(3, n_q_ - 3, XX,
                                                            x_desired, Q_cost);
    double cost_contrib_obj_vel =
        TrajectoryEvaluator::ComputeQuadraticTrajectoryCost(
            n_v_ + 3, n_v_ - 3, XX, x_desired, Q_cost);

    std::cout << "Error breakdown" << std::endl;
    std::cout << "\t total error contribution from x_ee: "
              << error_contrib_ee_pos << std::endl;
    std::cout << "\t total error contribution from x_obj: "
              << error_contrib_obj_pos << std::endl;
    std::cout << "\t total error contribution from v_ee: "
              << error_contrib_ee_vel << std::endl;
    std::cout << "\t total error contribution from v_obj: "
              << error_contrib_obj_vel << std::endl;

    std::cout << "\nCOST BREAKDOWN" << std::endl;
    std::cout << "\t total cost contribution from x_ee: " << cost_contrib_ee_pos
              << std::endl;
    std::cout << "\t total cost contribution from x_obj: "
              << cost_contrib_obj_pos << std::endl;
    std::cout << "\t total cost contribution from v_ee: " << cost_contrib_ee_vel
              << std::endl;
    std::cout << "\t total cost contribution from v_obj: "
              << cost_contrib_obj_vel << std::endl;
    std::cout << "\t total cost contribution from u: " << cost_contrib_u
              << std::endl;

    std::cout << "\t total cost is: " << cost << std::endl;
    std::cout << "\t total cost object terms only is : "
              << cost_contrib_obj_pos + cost_contrib_obj_vel << std::endl;
    std::cout << "\n\n";
  }

  std::pair<double, vector<VectorXd>> ret(cost, XX);
  return ret;
}

std::pair<vector<LCS>, vector<LCS>>
ElastoPlasticSC3Controller::CreateLCSObjectsForSamples(
    const vector<VectorXd>& candidate_states,
    const drake::VectorX<double>& x_lcs_curr,
    const vector<SortedPair<GeometryId>>& internal_contact_pairs,
    const vector<double>& yield_forces,
    const ElastoPlasticLCSFactoryOptions& eplcs_factory_options) const {
  // Build the LCS candidates.
  vector<LCS> lcs_candidates;
  vector<LCS> lcs_candidates_for_cost;

  int num_total_samples = candidate_states.size();
  for (int i = 0; i < num_total_samples; i++) {
    // Context needs to be updated to create the LCS objects.
    UpdateContext(n_q_, n_v_, n_u_, plant_, context_, plant_ad_, context_ad_,
                  candidate_states[i]);

    // Resolve external contact pairs and create the LCS.
    vector<SortedPair<GeometryId>> resolved_external_contact_pairs =
        GetResolvedContactPairs(
            plant_, *context_, contact_pairs_,
            elastoplastic_sc3_options_.resolve_contacts_to,
            elastoplastic_sc3_options_.num_friction_directions_per_contact
                .value(),
            verbose_);
    LCS lcs_object_sample =
        ElastoPlasticLCSFactory(plant_, *context_, plant_ad_, *context_ad_,
                                resolved_external_contact_pairs,
                                internal_contact_pairs, yield_forces,
                                eplcs_factory_options)
            .GenerateLCS();
    lcs_candidates.push_back(lcs_object_sample);

    // Create different LCS objects for cost calculation.
    vector<SortedPair<GeometryId>> resolved_external_contact_pairs_for_cost =
        GetResolvedContactPairs(
            plant_, *context_, contact_pairs_,
            elastoplastic_sc3_options_.resolve_contacts_to_for_cost,
            elastoplastic_sc3_options_.num_friction_directions_per_contact
                .value(),
            verbose_);
    ElastoPlasticLCSFactoryOptions eplcs_factory_options_for_cost =
        eplcs_factory_options;
    eplcs_factory_options_for_cost.SetLCSFactoryOptionsFromBase(
        eplcs_factory_options);
    eplcs_factory_options_for_cost.N =
        N_ * elastoplastic_sc3_options_.lcs_dt_resolution;
    eplcs_factory_options_for_cost.dt =
        dt_ / elastoplastic_sc3_options_.lcs_dt_resolution;
    eplcs_factory_options_for_cost.num_contacts =
        resolved_external_contact_pairs_for_cost.size();
    eplcs_factory_options_for_cost.spring_stiffness = 0.0;
    eplcs_factory_options_for_cost.num_friction_directions_per_contact =
        elastoplastic_sc3_options_.num_friction_directions_per_contact_for_cost;
    eplcs_factory_options_for_cost.mu_per_contact =
        elastoplastic_sc3_options_.mu_for_cost;
    eplcs_factory_options_for_cost.planar_normal_direction =
        elastoplastic_sc3_options_.planar_normal_direction;
    LCS lcs_object_sample_for_cost =
        ElastoPlasticLCSFactory(plant_, *context_, plant_ad_, *context_ad_,
                                resolved_external_contact_pairs_for_cost,
                                internal_contact_pairs, yield_forces,
                                eplcs_factory_options_for_cost)
            .GenerateLCS();
    lcs_candidates_for_cost.push_back(lcs_object_sample_for_cost);
  }

  // Reset the context to the current lcs state.
  UpdateContext(n_q_, n_v_, n_u_, plant_, context_, plant_ad_, context_ad_,
                x_lcs_curr);

  if (verbose_) {
    // Print the LCS matrices for verbose inspection.
    LCS verbose_lcs = lcs_candidates.at(SampleIndex::kCurrentLocation);
    std::cout << "A: " << std::endl;
    std::cout << verbose_lcs.A()[0] << std::endl;
    std::cout << "B: " << std::endl;
    std::cout << verbose_lcs.B()[0] << std::endl;
    std::cout << "D: " << std::endl;
    std::cout << verbose_lcs.D()[0] << std::endl;
    std::cout << "d: " << std::endl;
    std::cout << verbose_lcs.d()[0] << std::endl;
    std::cout << "E: " << std::endl;
    std::cout << verbose_lcs.E()[0] << std::endl;
    std::cout << "F: " << std::endl;
    std::cout << verbose_lcs.F()[0] << std::endl;
    std::cout << "H: " << std::endl;
    std::cout << verbose_lcs.H()[0] << std::endl;
    std::cout << "c: " << std::endl;
    std::cout << verbose_lcs.c()[0] << std::endl;
  }

  return std::make_pair(lcs_candidates, lcs_candidates_for_cost);
}

void ElastoPlasticSC3Controller::PruneOutdatedSamplesFromBuffer(
    const VectorXd& x_lcs, int* num_in_buffer, MatrixXd* sample_buffer,
    VectorXd* sample_costs_buffer, const double& pos_error_sample_retention,
    [[maybe_unused]] const double& ang_error_sample_retention) const {
  (void)ang_error_sample_retention;  // Unused; network nodes are 3D only.

  int n_buffer_length = sample_costs_buffer->size();

  // Get elastoplastic network node locations, both current and from the
  // buffer.
  vector<Eigen::Array<bool, Eigen::Dynamic, 1>> mask_satisfies_pos;
  for (int i = 0; i < n_nodes_; i++) {
    Vector3d node_curr = x_lcs.segment(3 + 3 * i, 3);
    MatrixXd node_buffer =
        sample_buffer->block(0, 3 + 3 * i, n_buffer_length, 3);

    // Compute the linear difference.
    MatrixXd pos_deltas = node_buffer.rowwise() - node_curr.transpose();
    VectorXd distances = pos_deltas.rowwise().norm();
    mask_satisfies_pos.push_back(distances.array() <
                                 pos_error_sample_retention);
  }

  // Keep buffer if none of nodes moved.
  int retained_count = 0;
  MatrixXd retained_samples = MatrixXd::Zero(n_buffer_length, n_q_);
  VectorXd retained_costs = -1 * VectorXd::Ones(n_buffer_length);
  for (int i = 0; i < *num_in_buffer; i++) {
    bool keep = true;
    for (int j = 0; j < n_nodes_; j++) {
      if (!mask_satisfies_pos.at(j)[i]) {
        keep = false;
        break;
      }
    }
    if (keep) {
      retained_samples.row(retained_count) = sample_buffer->row(i);
      retained_costs[retained_count] = (*sample_costs_buffer)[i];
      retained_count++;
    }
    if ((*sample_costs_buffer)[i] < 0) {
      break;
    }
  }
  *num_in_buffer = retained_count;
  *sample_buffer = retained_samples;
  *sample_costs_buffer = retained_costs;
}

void ElastoPlasticSC3Controller::KeepTrackOfC3ModeProgress(
    const drake::VectorX<double>& x_lcs_curr,
    const BasicVector<double>& x_lcs_final_des, bool& met_minimum_progress,
    const bool& print_current_node_cost) const {
  bool updated_cost = false;
  bool updated_config_cost = false;
  bool updated_error = false;
  double cost_progress_fraction = -INFINITY;  // Negative means progress.

  // Accumulate configuration costs across all deformable nodes.
  VectorXd node_error_vec = x_lcs_curr.segment(3, n_q_ - 3) -
                            x_lcs_final_des.get_value().segment(3, n_q_ - 3);
  double curr_node_cost = node_error_vec.transpose() *
                          Q_[0].block(3, 3, n_q_ - 3, n_q_ - 3) *
                          node_error_vec;

  // Check for progress along different metrics.
  if ((all_sample_costs_[SampleIndex::kCurrentLocation] < lowest_cost_) ||
      (lowest_cost_ == -1.0)) {
    lowest_cost_ = all_sample_costs_[SampleIndex::kCurrentLocation];
    updated_cost = true;
  }
  if ((curr_node_cost < lowest_node_current_cost_) ||
      (lowest_node_current_cost_ == -1.0)) {
    lowest_node_current_cost_ = curr_node_cost;
    updated_config_cost = true;
  }
  if ((current_node_error_ < lowest_node_error_) ||
      (lowest_node_error_ == -1.0)) {
    lowest_node_error_ = current_node_error_;
    updated_error = true;
  }

  // So that the SamplingC3Controller::OutputDebug can report the node costs,
  // set these variables too.
  current_position_error_ = current_node_error_;
  lowest_pos_and_rot_current_cost_ = lowest_node_current_cost_;
  lowest_position_error_ = lowest_node_error_;

  // One of the progress metrics requires a history of object
  // configuration costs.  Maintain this history and check for progress.
  node_cost_history_.push(curr_node_cost);
  int max_history_length = progress_params_.progress_enforced_over_n_loops;
  if (node_cost_history_.size() > max_history_length) {
    node_cost_history_.pop();
  }
  // Check for progress if the history is full.
  if (node_cost_history_.size() == max_history_length) {
    // Note:  front() is the oldest cost, back() is the most recent.
    cost_progress_fraction =  // Negative means progress.
        ((node_cost_history_.back() - node_cost_history_.front()) /
         node_cost_history_.front());
  }

  if (print_current_node_cost) {
    std::cout << "Current node cost: " << curr_node_cost << std::endl;
  }

  // Keep track of how many control loops have passed since the best seen
  // progress metric in this mode.
  ProgressMetric progress_metric = progress_params_.track_c3_progress_via;
  if (((progress_metric == ProgressMetric::kC3Cost) && updated_cost) ||
      ((progress_metric == ProgressMetric::kConfigCost) &&
       updated_config_cost) ||
      ((progress_metric == ProgressMetric::kPosOrRotCost) && updated_error)) {
    best_progress_steps_ago_ = 0;
  } else {
    best_progress_steps_ago_++;
  }

  // Detect if progress was sufficient according to progress metric.
  int num_control_loops_to_wait = progress_params_.num_control_loops_to_wait;
  if (progress_metric == ProgressMetric::kConfigCostDrop) {
    if (cost_progress_fraction >
        -progress_params_.progress_enforced_cost_drop) {
      met_minimum_progress = false;
    }
  } else if (best_progress_steps_ago_ > num_control_loops_to_wait) {
    met_minimum_progress = false;
  }
}

void ElastoPlasticSC3Controller::ResetProgressMetrics() const {
  lowest_cost_ = -1.0;
  lowest_node_current_cost_ = -1.0;
  lowest_node_error_ = -1.0;
  best_progress_steps_ago_ = 0;
  // Clear the stored history of object configuration costs.
  while (!node_cost_history_.empty()) {
    node_cost_history_.pop();
  }
}

std::pair<vector<SortedPair<GeometryId>>, vector<double>>
ElastoPlasticSC3Controller::GetCurrentElastoPlasticProperties(
    const dairlib::lcmt_elastoplastic_network& elastoplastic_network_lcmt)
    const {
  DRAKE_DEMAND(elastoplastic_network_lcmt.num_points == n_nodes_);

  // Extract the appropriate internal contact pairs and yield forces.
  int n_connections = elastoplastic_network_lcmt.num_connections;
  DRAKE_DEMAND(3 * n_connections == n_lambda_internal_);
  vector<SortedPair<GeometryId>> internal_contact_pairs;
  vector<double> yield_forces(n_connections);
  vector<double> spring_constants(
      n_connections);  // TODO @bibit:  unused until more
                       // deformation models are implemented
  for (int i = 0; i < n_connections; i++) {
    GeometryId geom_1 = internal_contact_geometries_.at(
        elastoplastic_network_lcmt.connections.at(i)[0]);
    GeometryId geom_2 = internal_contact_geometries_.at(
        elastoplastic_network_lcmt.connections.at(i)[1]);
    internal_contact_pairs.push_back(SortedPair<GeometryId>(geom_1, geom_2));
    yield_forces[i] = elastoplastic_network_lcmt.yield_forces.at(i);
    spring_constants[i] = elastoplastic_network_lcmt.spring_constants.at(i);
  }

  return std::make_pair(internal_contact_pairs, yield_forces);
}

void ElastoPlasticSC3Controller::SetNodeMasses(
    const dairlib::lcmt_elastoplastic_network& elastoplastic_network_lcmt)
    const {
  for (int i = 0; i < n_nodes_; i++) {
    double node_mass = elastoplastic_network_lcmt.node_masses.at(i);
    plant_.get_body(node_body_indices_[i]).SetMass(context_, node_mass);
    plant_ad_.get_body(node_body_indices_[i])
        .SetMass(context_ad_, drake::AutoDiffXd(node_mass));
  }
}

}  // namespace systems
}  // namespace dairlib
