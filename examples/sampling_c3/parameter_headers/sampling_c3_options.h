#pragma once
#include <iostream>
#include <numeric>
#include <stdexcept>

#include "c3/core/c3_options.h"
#include "c3/multibody/lcs_factory_options.h"

#include "drake/common/yaml/yaml_read_archive.h"

using c3::C3Options;
using c3::LCSFactoryOptions;

struct SamplingC3Options : C3Options, LCSFactoryOptions {
  std::string projection_type;  // "QP" or "MIQP" or "C3+"
  double solve_time_filter_alpha = 0.0;
  double publish_frequency = 100.0;  // Hz

  int num_outer_threads;  // for outer sampling loop.

  /// Additional radius-based workspace limit.
  std::vector<double> robot_radius_limits;

  /// Whether to use a predicted EE location for each mode.
  bool use_predicted_x0_c3;
  bool use_predicted_x0_repos;
  bool use_predicted_x0_reset_mechanism;  // Resets if prediction is too far.

  /// Contact pair parameters.
  std::vector<double> mu_per_pair_type;
  std::vector<std::vector<int>> resolve_contacts_to_lists;
  std::vector<int> resolve_as_planar_contacts_list;
  std::vector<int> resolve_contacts_to;
  std::vector<int> resolve_contacts_to_for_cost;
  int num_contacts_index;
  int num_contacts_index_for_cost;
  std::vector<double> mu_for_cost;
  int num_contacts_for_cost;

  int num_planar_contacts;
  int n_lambda_with_tangential;
  std::vector<int> starting_index_per_contact_in_lambda_t_vector;

  int num_planar_contacts_cost;
  int n_lambda_with_tangential_cost;
  std::vector<int> num_friction_directions_per_contact_for_cost;
  std::vector<int> starting_index_per_contact_in_lambda_t_vector_cost;

  double planning_dt_pose;
  double planning_dt_position;
  int lcs_dt_resolution;
  double dt_cost;
  double nominal_ee_accel;

  /// Cost parameters.
  bool use_quaternion_dependent_cost;
  double q_quaternion_dependent_weight;
  double q_quaternion_dependent_regularizer_fraction;

  std::vector<double> Kp_for_ee_pd_rollout;
  std::vector<double> Kd_for_ee_pd_rollout;

  bool include_walls;
  bool include_back_wall;

  /// Cost parameters for pose tracking.
  std::vector<std::vector<double>> g_gamma_list;
  std::vector<std::vector<double>> g_lambda_n_list;
  std::vector<std::vector<double>> g_lambda_t_list;
  std::vector<std::vector<double>> g_lambda_list;

  std::vector<std::vector<double>> g_eta_slack_list;
  std::vector<std::vector<double>> g_eta_n_list;
  std::vector<std::vector<double>> g_eta_t_list;
  std::vector<std::vector<double>> g_eta_list;

  std::vector<std::vector<double>> u_gamma_list;
  std::vector<std::vector<double>> u_lambda_n_list;
  std::vector<std::vector<double>> u_lambda_t_list;
  std::vector<std::vector<double>> u_lambda_list;

  std::vector<std::vector<double>> u_eta_slack_list;
  std::vector<std::vector<double>> u_eta_n_list;
  std::vector<std::vector<double>> u_eta_t_list;
  std::vector<std::vector<double>> u_eta_list;

  /// Cost parameters for position tracking.
  double w_Q_position;
  double w_R_position;
  double w_G_position;
  double w_U_position;
  std::vector<double> q_vector_position;
  std::vector<double> r_vector_position;

  std::vector<double> g_x_position;
  std::vector<std::vector<double>> g_gamma_position_list;
  std::vector<std::vector<double>> g_lambda_n_position_list;
  std::vector<std::vector<double>> g_lambda_t_position_list;
  std::vector<std::vector<double>> g_lambda_position_list;
  std::vector<double> g_u_position;

  std::vector<double> u_x_position;
  std::vector<std::vector<double>> u_gamma_position_list;
  std::vector<std::vector<double>> u_lambda_n_position_list;
  std::vector<std::vector<double>> u_lambda_t_position_list;
  std::vector<std::vector<double>> u_lambda_position_list;
  std::vector<double> u_u_position;

  // Only applicable for C3+
  std::vector<std::vector<double>> g_eta_slack_position_list;
  std::vector<std::vector<double>> g_eta_n_position_list;
  std::vector<std::vector<double>> g_eta_t_position_list;
  std::vector<std::vector<double>> g_eta_position_list;
  std::vector<std::vector<double>> u_eta_slack_position_list;
  std::vector<std::vector<double>> u_eta_n_position_list;
  std::vector<std::vector<double>> u_eta_t_position_list;
  std::vector<std::vector<double>> u_eta_position_list;

  std::vector<double>
      u_horizontal_limits;  ///< Limits for horizontal actuator inputs.
  std::vector<double>
      u_vertical_limits;  ///< Limits for vertical actuator inputs.
  std::vector<Eigen::VectorXd>
      workspace_limits;      ///< Workspace boundaries as vectors.
  double workspace_margins;  ///< Margins to be maintained within the workspace.
  std::vector<Eigen::VectorXd>
      head_workspace_limits;  ///< Workspace boundaries for the head as vectors.
  std::vector<double>
      ee_velocity_limits;  ///< Limits for end-effector velocities.

  C3Options c3_options_pose;
  LCSFactoryOptions lcs_factory_options_pose;
  C3Options c3_options_position;
  LCSFactoryOptions lcs_factory_options_position;

  template <typename Archive>
  void Serialize(Archive* a) {
    C3Options::Serialize(a);
    LCSFactoryOptions::Serialize(a);
    a->Visit(DRAKE_NVP(projection_type));
    a->Visit(DRAKE_NVP(solve_time_filter_alpha));
    a->Visit(DRAKE_NVP(publish_frequency));
    a->Visit(DRAKE_NVP(num_outer_threads));
    a->Visit(DRAKE_NVP(robot_radius_limits));
    a->Visit(DRAKE_NVP(use_predicted_x0_c3));
    a->Visit(DRAKE_NVP(use_predicted_x0_repos));
    a->Visit(DRAKE_NVP(use_predicted_x0_reset_mechanism));

    a->Visit(DRAKE_NVP(mu_per_pair_type));
    a->Visit(DRAKE_NVP(resolve_contacts_to_lists));
    a->Visit(DRAKE_NVP(resolve_as_planar_contacts_list));
    a->Visit(DRAKE_NVP(num_contacts_index));
    a->Visit(DRAKE_NVP(num_contacts_index_for_cost));

    a->Visit(DRAKE_NVP(planning_dt_pose));
    a->Visit(DRAKE_NVP(planning_dt_position));
    a->Visit(DRAKE_NVP(lcs_dt_resolution));
    a->Visit(DRAKE_NVP(dt_cost));
    a->Visit(DRAKE_NVP(nominal_ee_accel));

    a->Visit(DRAKE_NVP(use_quaternion_dependent_cost));
    a->Visit(DRAKE_NVP(q_quaternion_dependent_weight));
    a->Visit(DRAKE_NVP(q_quaternion_dependent_regularizer_fraction));

    a->Visit(DRAKE_NVP(Kp_for_ee_pd_rollout));
    a->Visit(DRAKE_NVP(Kd_for_ee_pd_rollout));

    a->Visit(DRAKE_NVP(include_walls));
    a->Visit(DRAKE_NVP(include_back_wall));

    a->Visit(DRAKE_NVP(g_gamma_list));
    a->Visit(DRAKE_NVP(g_lambda_n_list));
    a->Visit(DRAKE_NVP(g_lambda_t_list));
    a->Visit(DRAKE_NVP(g_lambda_list));

    a->Visit(DRAKE_NVP(u_gamma_list));
    a->Visit(DRAKE_NVP(u_lambda_n_list));
    a->Visit(DRAKE_NVP(u_lambda_t_list));
    a->Visit(DRAKE_NVP(u_lambda_list));

    a->Visit(DRAKE_NVP(w_Q_position));
    a->Visit(DRAKE_NVP(w_R_position));
    a->Visit(DRAKE_NVP(w_G_position));
    a->Visit(DRAKE_NVP(w_U_position));
    a->Visit(DRAKE_NVP(q_vector_position));
    a->Visit(DRAKE_NVP(r_vector_position));

    a->Visit(DRAKE_NVP(g_x_position));
    a->Visit(DRAKE_NVP(g_gamma_position_list));
    a->Visit(DRAKE_NVP(g_lambda_n_position_list));
    a->Visit(DRAKE_NVP(g_lambda_t_position_list));
    a->Visit(DRAKE_NVP(g_lambda_position_list));
    a->Visit(DRAKE_NVP(g_u_position));

    a->Visit(DRAKE_NVP(u_x_position));
    a->Visit(DRAKE_NVP(u_gamma_position_list));
    a->Visit(DRAKE_NVP(u_lambda_n_position_list));
    a->Visit(DRAKE_NVP(u_lambda_t_position_list));
    a->Visit(DRAKE_NVP(u_lambda_position_list));
    a->Visit(DRAKE_NVP(u_u_position));

    // Only applicable for C3+
    a->Visit(DRAKE_NVP(g_eta_slack_list));
    a->Visit(DRAKE_NVP(g_eta_n_list));
    a->Visit(DRAKE_NVP(g_eta_t_list));
    a->Visit(DRAKE_NVP(g_eta_list));
    a->Visit(DRAKE_NVP(u_eta_slack_list));
    a->Visit(DRAKE_NVP(u_eta_n_list));
    a->Visit(DRAKE_NVP(u_eta_t_list));
    a->Visit(DRAKE_NVP(u_eta_list));
    a->Visit(DRAKE_NVP(g_eta_slack_position_list));
    a->Visit(DRAKE_NVP(g_eta_n_position_list));
    a->Visit(DRAKE_NVP(g_eta_t_position_list));
    a->Visit(DRAKE_NVP(g_eta_position_list));
    a->Visit(DRAKE_NVP(u_eta_slack_position_list));
    a->Visit(DRAKE_NVP(u_eta_n_position_list));
    a->Visit(DRAKE_NVP(u_eta_t_position_list));
    a->Visit(DRAKE_NVP(u_eta_position_list));

    a->Visit(DRAKE_NVP(u_horizontal_limits));
    a->Visit(DRAKE_NVP(u_vertical_limits));
    a->Visit(DRAKE_NVP(workspace_limits));
    a->Visit(DRAKE_NVP(workspace_margins));
    a->Visit(DRAKE_NVP(head_workspace_limits));
    a->Visit(DRAKE_NVP(ee_velocity_limits));

    DRAKE_ASSERT(num_friction_directions.has_value());

    // Set a few parameters based on num_contacts_index, differentiating between
    // for C3 solve and for C3 cost computation.
    resolve_contacts_to = resolve_contacts_to_lists[num_contacts_index];
    resolve_contacts_to_for_cost =
        resolve_contacts_to_lists[num_contacts_index_for_cost];

    if (!include_walls) {
      if (resolve_contacts_to.back() != 0 ||
          resolve_contacts_to_for_cost.back() != 0) {
        throw std::runtime_error(
            "Walls are not included, the number of contacts between object and "
            "wall must be 0.");
      }
    }

    num_contacts = std::accumulate(resolve_contacts_to.begin(),
                                   resolve_contacts_to.end(), 0);
    num_contacts_for_cost =
        std::accumulate(resolve_contacts_to_for_cost.begin(),
                        resolve_contacts_to_for_cost.end(), 0);
    mu_per_contact = std::vector<double>();
    for (size_t i = 0; i < mu_per_pair_type.size(); ++i) {
      int repeat = resolve_contacts_to_lists[num_contacts_index][i];
      mu_per_contact.value().insert(mu_per_contact.value().end(), repeat,
                                    mu_per_pair_type[i]);
    }
    mu_for_cost = std::vector<double>();
    for (size_t i = 0; i < mu_per_pair_type.size(); ++i) {
      int repeat = resolve_contacts_to_lists[num_contacts_index_for_cost][i];
      mu_for_cost.insert(mu_for_cost.end(), repeat, mu_per_pair_type[i]);
    }

    // Process planar contact info
    std::tie(num_planar_contacts, num_friction_directions_per_contact) =
        ProcessPlanarContactInformation(resolve_as_planar_contacts_list,
                                        resolve_contacts_to,
                                        num_friction_directions.value());
    std::tie(num_planar_contacts_cost,
             num_friction_directions_per_contact_for_cost) =
        ProcessPlanarContactInformation(resolve_as_planar_contacts_list,
                                        resolve_contacts_to_for_cost,
                                        num_friction_directions.value());

    for (size_t i = 0; i < static_cast<size_t>(num_contacts.value()); ++i) {
      starting_index_per_contact_in_lambda_t_vector.push_back(
          2 * std::accumulate(
                  num_friction_directions_per_contact.value().begin(),
                  num_friction_directions_per_contact.value().begin() + i, 0));
    }
    for (size_t i = 0; i < static_cast<size_t>(num_contacts_for_cost); ++i) {
      starting_index_per_contact_in_lambda_t_vector_cost.push_back(
          2 * std::accumulate(
                  num_friction_directions_per_contact_for_cost.begin(),
                  num_friction_directions_per_contact_for_cost.begin() + i, 0));
    }

    n_lambda_with_tangential =
        2 * num_friction_directions.value() *
            (num_contacts.value() - num_planar_contacts) +
        2 * num_planar_contacts;
    n_lambda_with_tangential_cost =
        2 * num_friction_directions.value() *
            (num_contacts_for_cost - num_planar_contacts_cost) +
        2 * num_planar_contacts_cost;

    // Create C3 options for both pose and position tracking.
    std::cout
        << "Setting C3 and LCS factory options for pose and position tracking."
        << std::endl;
    SetCommonOptions(&c3_options_pose, &lcs_factory_options_pose);
    SetPoseTrackingOptions(&c3_options_pose, &lcs_factory_options_pose);
    SetCommonOptions(&c3_options_position, &lcs_factory_options_position);
    SetPositionTrackingOptions(&c3_options_position,
                               &lcs_factory_options_position);
  }

  C3Options GetC3Options(const bool& is_pose_tracking) const {
    if (is_pose_tracking) {
      return c3_options_pose;
    }
    return c3_options_position;
  }

  LCSFactoryOptions GetLCSFactoryOptions(const bool& is_pose_tracking) const {
    if (is_pose_tracking) {
      return lcs_factory_options_pose;
    }
    return lcs_factory_options_position;
  }

 private:
  void PopulateCostMatricesFromVectors(C3Options* options) const {
    std::vector<double> g_vector = std::vector<double>();
    g_vector.insert(g_vector.end(), options->g_x.begin(), options->g_x.end());
    if (contact_model == "stewart_and_trinkle") {
      g_vector.insert(g_vector.end(), options->g_gamma.begin(),
                      options->g_gamma.end());
      g_vector.insert(g_vector.end(), options->g_lambda_n.begin(),
                      options->g_lambda_n.end());
      g_vector.insert(g_vector.end(), options->g_lambda_t.begin(),
                      options->g_lambda_t.end());
    } else {
      g_vector.insert(g_vector.end(), options->g_lambda.begin(),
                      options->g_lambda.end());
    }
    g_vector.insert(g_vector.end(), options->g_u.begin(), options->g_u.end());

    if (projection_type == "C3+") {
      if (contact_model == "stewart_and_trinkle") {
        g_vector.insert(g_vector.end(), options->g_eta_slack->begin(),
                        options->g_eta_slack->end());
        g_vector.insert(g_vector.end(), options->g_eta_n->begin(),
                        options->g_eta_n->end());
        g_vector.insert(g_vector.end(), options->g_eta_t->begin(),
                        options->g_eta_t->end());
      } else {
        g_vector.insert(g_vector.end(), options->g_eta->begin(),
                        options->g_eta->end());
      }
    }

    std::vector<double> u_vector = std::vector<double>();
    u_vector.insert(u_vector.end(), options->u_x.begin(), options->u_x.end());
    if (contact_model == "stewart_and_trinkle") {
      u_vector.insert(u_vector.end(), options->u_gamma.begin(),
                      options->u_gamma.end());
      u_vector.insert(u_vector.end(), options->u_lambda_n.begin(),
                      options->u_lambda_n.end());
      u_vector.insert(u_vector.end(), options->u_lambda_t.begin(),
                      options->u_lambda_t.end());
    } else {
      u_vector.insert(u_vector.end(), options->u_lambda.begin(),
                      options->u_lambda.end());
    }
    u_vector.insert(u_vector.end(), options->u_u.begin(), options->u_u.end());

    if (projection_type == "C3+") {
      if (contact_model == "stewart_and_trinkle") {
        u_vector.insert(u_vector.end(), options->u_eta_slack->begin(),
                        options->u_eta_slack->end());
        u_vector.insert(u_vector.end(), options->u_eta_n->begin(),
                        options->u_eta_n->end());
        u_vector.insert(u_vector.end(), options->u_eta_t->begin(),
                        options->u_eta_t->end());
      } else {
        u_vector.insert(u_vector.end(), options->u_eta->begin(),
                        options->u_eta->end());
      }
    }

    Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(
        options->q_vector.data(), options->q_vector.size());
    Eigen::VectorXd r = Eigen::Map<const Eigen::VectorXd>(
        options->r_vector.data(), options->r_vector.size());
    Eigen::VectorXd g =
        Eigen::Map<const Eigen::VectorXd>(g_vector.data(), g_vector.size());
    Eigen::VectorXd u =
        Eigen::Map<const Eigen::VectorXd>(u_vector.data(), u_vector.size());

    options->Q = options->w_Q * q.asDiagonal();
    options->R = options->w_R * r.asDiagonal();
    options->G = options->w_G * g.asDiagonal();
    options->U = options->w_U * u.asDiagonal();
  }

  void SetCommonOptions(C3Options* c3_options,
                        LCSFactoryOptions* lcs_factory_options) const {
    c3_options->warm_start = warm_start;
    c3_options->penalize_input_change = penalize_input_change;
    c3_options->end_on_qp_step = end_on_qp_step;
    c3_options->scale_lcs = scale_lcs;

    c3_options->num_threads = num_threads;
    c3_options->delta_option = delta_option;

    c3_options->admm_iter = admm_iter;

    c3_options->gamma = gamma;
    c3_options->rho_scale = rho_scale;

    c3_options->M = M;
    c3_options->qp_projection_alpha = qp_projection_alpha;
    c3_options->qp_projection_scaling = qp_projection_scaling;
    c3_options->final_augmented_cost_contact_scaling =
        final_augmented_cost_contact_scaling;
    c3_options->final_augmented_cost_contact_indices =
        final_augmented_cost_contact_indices;

    lcs_factory_options->contact_model = contact_model;
    lcs_factory_options->num_contacts = num_contacts;
    lcs_factory_options->num_friction_directions = num_friction_directions;
    lcs_factory_options->spring_stiffness = 0;  // not used in sampling C3
    lcs_factory_options->mu_per_contact = mu_per_contact;
    lcs_factory_options->N = N;
    lcs_factory_options->num_friction_directions_per_contact =
        num_friction_directions_per_contact;
    lcs_factory_options->planar_normal_direction = planar_normal_direction;
    lcs_factory_options->planar_normal_direction_per_contact =
        planar_normal_direction_per_contact;
  }

  void SetPositionTrackingOptions(
      C3Options* c3_options, LCSFactoryOptions* lcs_factory_options) const {
    lcs_factory_options->dt = planning_dt_position;
    c3_options->w_Q = w_Q_position;
    c3_options->w_R = w_R_position;
    c3_options->w_G = w_G_position;
    c3_options->w_U = w_U_position;
    c3_options->q_vector = q_vector_position;
    c3_options->r_vector = r_vector_position;

    c3_options->g_x = g_x_position;
    c3_options->g_gamma = g_gamma_position_list[num_contacts_index];
    c3_options->g_lambda_n = g_lambda_n_position_list[num_contacts_index];
    c3_options->g_lambda_t = g_lambda_t_position_list[num_contacts_index];
    c3_options->g_lambda = g_lambda_position_list[num_contacts_index];
    c3_options->g_u = g_u_position;

    c3_options->u_x = u_x_position;
    c3_options->u_gamma = u_gamma_position_list[num_contacts_index];
    c3_options->u_lambda_n = u_lambda_n_position_list[num_contacts_index];
    c3_options->u_lambda_t = u_lambda_t_position_list[num_contacts_index];
    c3_options->u_lambda = u_lambda_position_list[num_contacts_index];
    c3_options->u_u = u_u_position;

    // Only applicable for C3+
    if (projection_type == "C3+") {
      c3_options->g_eta_slack = g_eta_slack_position_list[num_contacts_index];
      c3_options->g_eta_n = g_eta_n_position_list[num_contacts_index];
      c3_options->g_eta_t = g_eta_t_position_list[num_contacts_index];
      c3_options->g_eta = g_eta_position_list[num_contacts_index];
      c3_options->u_eta_slack = u_eta_slack_position_list[num_contacts_index];
      c3_options->u_eta_n = u_eta_n_position_list[num_contacts_index];
      c3_options->u_eta_t = u_eta_t_position_list[num_contacts_index];
      c3_options->u_eta = u_eta_position_list[num_contacts_index];
    }

    MakePlanarLambdaCost(c3_options, lcs_factory_options);
    PopulateCostMatricesFromVectors(c3_options);
  }

  void SetPoseTrackingOptions(C3Options* c3_options,
                              LCSFactoryOptions* lcs_factory_options) const {
    lcs_factory_options->dt = planning_dt_pose;
    c3_options->w_Q = w_Q;
    c3_options->w_R = w_R;
    c3_options->w_G = w_G;
    c3_options->w_U = w_U;
    c3_options->q_vector = q_vector;
    c3_options->r_vector = r_vector;

    c3_options->g_x = g_x;
    c3_options->g_gamma = g_gamma_list[num_contacts_index];
    c3_options->g_lambda_n = g_lambda_n_list[num_contacts_index];
    c3_options->g_lambda_t = g_lambda_t_list[num_contacts_index];
    c3_options->g_lambda = g_lambda_list[num_contacts_index];
    c3_options->g_u = g_u;

    c3_options->u_x = u_x;
    c3_options->u_gamma = u_gamma_list[num_contacts_index];
    c3_options->u_lambda_n = u_lambda_n_list[num_contacts_index];
    c3_options->u_lambda_t = u_lambda_t_list[num_contacts_index];
    c3_options->u_lambda = u_lambda_list[num_contacts_index];
    c3_options->u_u = u_u;

    // Only applicable for C3+
    if (projection_type == "C3+") {
      c3_options->g_eta_slack = g_eta_slack_list[num_contacts_index];
      c3_options->g_eta_n = g_eta_n_list[num_contacts_index];
      c3_options->g_eta_t = g_eta_t_list[num_contacts_index];
      c3_options->g_eta = g_eta_list[num_contacts_index];
      c3_options->u_eta_slack = u_eta_slack_list[num_contacts_index];
      c3_options->u_eta_n = u_eta_n_list[num_contacts_index];
      c3_options->u_eta_t = u_eta_t_list[num_contacts_index];
      c3_options->u_eta = u_eta_list[num_contacts_index];
    }

    MakePlanarLambdaCost(c3_options, lcs_factory_options);
    PopulateCostMatricesFromVectors(c3_options);
  }

  // Convert lambda weights to the planar form:
  void MakePlanarLambdaCost(C3Options* c3_options,
                            LCSFactoryOptions* lcs_factory_options) const {
    int offset = 0;
    for (size_t i = 0; i < resolve_contacts_to_lists[num_contacts_index].size();
         ++i) {
      if (resolve_as_planar_contacts_list[i]) {
        int erase_start_index_for_lambda =
            2 * num_friction_directions.value() *
                std::accumulate(
                    resolve_contacts_to_lists[num_contacts_index].begin(),
                    resolve_contacts_to_lists[num_contacts_index].begin() + i,
                    0) -
            offset;
        int erase_end_index_for_lambda =
            erase_start_index_for_lambda +
            2 * (num_friction_directions.value() - 1) *
                resolve_contacts_to_lists[num_contacts_index][i];

        c3_options->g_lambda.erase(
            c3_options->g_lambda.begin() + erase_start_index_for_lambda,
            c3_options->g_lambda.begin() + erase_end_index_for_lambda);
        c3_options->u_lambda.erase(
            c3_options->u_lambda.begin() + erase_start_index_for_lambda,
            c3_options->u_lambda.begin() + erase_end_index_for_lambda);
        c3_options->g_lambda_t.erase(
            c3_options->g_lambda_t.begin() + erase_start_index_for_lambda,
            c3_options->g_lambda_t.begin() + erase_end_index_for_lambda);
        c3_options->u_lambda_t.erase(
            c3_options->u_lambda_t.begin() + erase_start_index_for_lambda,
            c3_options->u_lambda_t.begin() + erase_end_index_for_lambda);

        if (projection_type == "C3+") {
          DRAKE_ASSERT(c3_options->g_eta.has_value());
          auto& g_eta = c3_options->g_eta.value();
          DRAKE_ASSERT(c3_options->u_eta.has_value());
          auto& u_eta = c3_options->u_eta.value();
          DRAKE_ASSERT(c3_options->g_eta_t.has_value());
          auto& g_eta_t = c3_options->g_eta_t.value();
          DRAKE_ASSERT(c3_options->u_eta_t.has_value());
          auto& u_eta_t = c3_options->u_eta_t.value();
          g_eta.erase(g_eta.begin() + erase_start_index_for_lambda,
                      g_eta.begin() + erase_end_index_for_lambda);
          u_eta.erase(u_eta.begin() + erase_start_index_for_lambda,
                      u_eta.begin() + erase_end_index_for_lambda);
          g_eta_t.erase(g_eta_t.begin() + erase_start_index_for_lambda,
                        g_eta_t.begin() + erase_end_index_for_lambda);
          u_eta_t.erase(u_eta_t.begin() + erase_start_index_for_lambda,
                        u_eta_t.begin() + erase_end_index_for_lambda);
        }
        offset += 2 * (num_friction_directions.value() - 1) *
                  resolve_contacts_to_lists[num_contacts_index][i];
      }
    }
  }

  // Compute total number of planar friction directions
  // and create a vector that contains the number of friction directions for
  // each contact point
  std::pair<int, std::vector<int>> ProcessPlanarContactInformation(
      const std::vector<int>& resolve_as_planar_contacts_list,
      const std::vector<int>& resolve_contacts_to_list,
      int num_friction_directions) {
    int num_planar_contacts = 0;
    int planar_contact = 1;
    std::vector<int> num_friction_directions_per_contact;
    for (size_t i = 0; i < resolve_contacts_to_list.size(); ++i) {
      for (int j = 0; j < resolve_contacts_to_list[i]; ++j) {
        num_planar_contacts += (resolve_as_planar_contacts_list[i] ? 1 : 0);
        num_friction_directions_per_contact.push_back(
            resolve_as_planar_contacts_list[i] ? planar_contact
                                               : num_friction_directions);
      }
    }
    return std::pair<int, std::vector<int>>(
        num_planar_contacts, num_friction_directions_per_contact);
  }
};
