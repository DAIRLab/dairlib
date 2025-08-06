#pragma once
#include <numeric>
#include <iostream>

#include "solvers/c3_options.h"

#include "drake/common/yaml/yaml_read_archive.h"

struct SamplingC3Options : C3Options {
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
  std::vector<int> resolve_contacts_to;
  std::vector<int> resolve_contacts_to_for_cost;
  int num_contacts_index;
  int num_contacts_index_for_cost;
  std::vector<double> mu_for_cost;
  int num_contacts_for_cost;

  double planning_dt_pose;
  double planning_dt_position;
  int lcs_dt_resolution;
  double dt_cost;
  

  /// Cost parameters.
  bool use_quaternion_dependent_cost;
  double q_quaternion_dependent_weight;
  double q_quaternion_dependent_regularizer_fraction;

  std::vector<double> Kp_for_ee_pd_rollout;
  std::vector<double> Kd_for_ee_pd_rollout;

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

  C3Options c3_options_pose;
  C3Options c3_options_position;

  template <typename Archive>
  void Serialize(Archive* a) {
    C3Options::Serialize(a);
    a->Visit(DRAKE_NVP(num_outer_threads));
    a->Visit(DRAKE_NVP(robot_radius_limits));
    a->Visit(DRAKE_NVP(use_predicted_x0_c3));
    a->Visit(DRAKE_NVP(use_predicted_x0_repos));
    a->Visit(DRAKE_NVP(use_predicted_x0_reset_mechanism));

    a->Visit(DRAKE_NVP(mu_per_pair_type));
    a->Visit(DRAKE_NVP(resolve_contacts_to_lists));
    a->Visit(DRAKE_NVP(num_contacts_index));
    a->Visit(DRAKE_NVP(num_contacts_index_for_cost));

    a->Visit(DRAKE_NVP(planning_dt_pose));
    a->Visit(DRAKE_NVP(planning_dt_position));
    a->Visit(DRAKE_NVP(lcs_dt_resolution));
    a->Visit(DRAKE_NVP(dt_cost));

    a->Visit(DRAKE_NVP(use_quaternion_dependent_cost));
    a->Visit(DRAKE_NVP(q_quaternion_dependent_weight));
    a->Visit(DRAKE_NVP(q_quaternion_dependent_regularizer_fraction));

    a->Visit(DRAKE_NVP(Kp_for_ee_pd_rollout));
    a->Visit(DRAKE_NVP(Kd_for_ee_pd_rollout));

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

    // Set a few parameters based on num_contacts_index, differentiating between
    // for C3 solve and for C3 cost computation.
    resolve_contacts_to = resolve_contacts_to_lists[num_contacts_index];
    resolve_contacts_to_for_cost =
      resolve_contacts_to_lists[num_contacts_index_for_cost];
    num_contacts = std::accumulate(
      resolve_contacts_to.begin(), resolve_contacts_to.end(), 0);
    num_contacts_for_cost = std::accumulate(
      resolve_contacts_to_for_cost.begin(), resolve_contacts_to_for_cost.end(),
      0);
    mu.clear();
    for (size_t i = 0; i < mu_per_pair_type.size(); ++i) {
      int repeat = resolve_contacts_to_lists[num_contacts_index][i];
      mu.insert(mu.end(), repeat, mu_per_pair_type[i]);
    }
    mu_for_cost.clear();
    for (size_t i = 0; i < mu_per_pair_type.size(); ++i) {
      int repeat = resolve_contacts_to_lists[num_contacts_index_for_cost][i];
      mu_for_cost.insert(mu_for_cost.end(), repeat, mu_per_pair_type[i]);
    }

    // Create C3 options for both pose and position tracking.
    SetCommonC3Options(&c3_options_pose);
    SetPoseTrackingOptions(&c3_options_pose);
    SetCommonC3Options(&c3_options_position);
    SetPositionTrackingOptions(&c3_options_position);
  }

  C3Options GetC3Options(const bool& is_pose_tracking) const {
    if (is_pose_tracking) { return c3_options_pose; }
    return c3_options_position;
  }

  private:
    void PopulateCostMatricesFromVectors(C3Options* options) const {
      std::vector<double> g_vector = std::vector<double>();
      g_vector.insert(g_vector.end(), options->g_x.begin(), options->g_x.end());
      if (options->contact_model == "stewart_and_trinkle") {
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

      if (options->projection_type == "C3+") {
          if (options->contact_model == "stewart_and_trinkle") {
          g_vector.insert(g_vector.end(), options->g_eta_slack.begin(),
                          options->g_eta_slack.end());
          g_vector.insert(g_vector.end(), options->g_eta_n.begin(),
                          options->g_eta_n.end());
          g_vector.insert(g_vector.end(), options->g_eta_t.begin(),
                          options->g_eta_t.end());
        } else {
          g_vector.insert(g_vector.end(), options->g_eta.begin(),
                          options->g_eta.end());
        }
      }

      std::vector<double> u_vector = std::vector<double>();
      u_vector.insert(u_vector.end(), options->u_x.begin(), options->u_x.end());
      if (options->contact_model == "stewart_and_trinkle") {
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

      if (options->projection_type == "C3+") {
        if (options->contact_model == "stewart_and_trinkle") {
          u_vector.insert(u_vector.end(), options->u_eta_slack.begin(),
                          options->u_eta_slack.end());
          u_vector.insert(u_vector.end(), options->u_eta_n.begin(),
                          options->u_eta_n.end());
          u_vector.insert(u_vector.end(), options->u_eta_t.begin(),
                          options->u_eta_t.end());
        } else {
          u_vector.insert(u_vector.end(), options->u_eta.begin(),
                          options->u_eta.end());
        }
      }

      options->g_vector = g_vector;
      options->u_vector = u_vector;

      Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(
          options->q_vector.data(), options->q_vector.size());
      Eigen::VectorXd r = Eigen::Map<const Eigen::VectorXd>(
          options->r_vector.data(), options->r_vector.size());
      Eigen::VectorXd g = Eigen::Map<const Eigen::VectorXd>(
          options->g_vector.data(), options->g_vector.size());
      Eigen::VectorXd u = Eigen::Map<const Eigen::VectorXd>(
          options->u_vector.data(), options->u_vector.size());

      options->Q = options->w_Q * q.asDiagonal();
      options->R = options->w_R * r.asDiagonal();
      options->G = options->w_G * g.asDiagonal();
      options->U = options->w_U * u.asDiagonal();
    }

    void SetCommonC3Options(C3Options* options) const {
      options->admm_iter = admm_iter;
      options->rho = rho;
      options->rho_scale = rho_scale;
      options->num_threads = num_threads;
      options->delta_option = delta_option;
      options->contact_model = contact_model;
      options->projection_type = projection_type;
      options->warm_start = warm_start;
      options->use_predicted_x0 = false;  // unused by sampling C3
      options->end_on_qp_step = end_on_qp_step;
      options->solve_time_filter_alpha = solve_time_filter_alpha;
      options->publish_frequency = publish_frequency;

      options->workspace_limits = workspace_limits;
      options->workspace_margins = workspace_margins;
      options->u_horizontal_limits = u_horizontal_limits;
      options->u_vertical_limits = u_vertical_limits;

      options->N = N;
      options->gamma = gamma;

      options->solve_dt = 0;  // unused in all of C3
      options-> lcs_dt_resolution = lcs_dt_resolution;
      options->num_friction_directions = num_friction_directions;

      options->qp_projection_alpha = qp_projection_alpha;
      options->qp_projection_scaling = qp_projection_scaling;
      options->penalize_changes_in_u_across_solves =
          penalize_changes_in_u_across_solves;

      options->mu = mu;
      options->num_contacts = num_contacts;
    }

    void SetPositionTrackingOptions(C3Options* options) const {
      options->dt = planning_dt_position;
      options->dt_cost = planning_dt_position / lcs_dt_resolution;
      options->w_Q = w_Q_position;
      options->w_R = w_R_position;
      options->w_G = w_G_position;
      options->w_U = w_U_position;
      options->q_vector = q_vector_position;
      options->r_vector = r_vector_position;

      options->g_x = g_x_position;
      options->g_gamma = g_gamma_position_list[num_contacts_index];
      options->g_lambda_n = g_lambda_n_position_list[num_contacts_index];
      options->g_lambda_t = g_lambda_t_position_list[num_contacts_index];
      options->g_lambda = g_lambda_position_list[num_contacts_index];
      options->g_u = g_u_position;


      options->u_x = u_x_position;
      options->u_gamma = u_gamma_position_list[num_contacts_index];
      options->u_lambda_n = u_lambda_n_position_list[num_contacts_index];
      options->u_lambda_t = u_lambda_t_position_list[num_contacts_index];
      options->u_lambda = u_lambda_position_list[num_contacts_index];
      options->u_u = u_u_position;

      // Only applicable for C3+
      if (options->projection_type == "C3+") {
        options->g_eta_slack = g_eta_slack_position_list[num_contacts_index];
        options->g_eta_n = g_eta_n_position_list[num_contacts_index];
        options->g_eta_t = g_eta_t_position_list[num_contacts_index];
        options->g_eta = g_eta_position_list[num_contacts_index];
        options->u_eta_slack = u_eta_slack_position_list[num_contacts_index];
        options->u_eta_n = u_eta_n_position_list[num_contacts_index];
        options->u_eta_t = u_eta_t_position_list[num_contacts_index];
        options->u_eta = u_eta_position_list[num_contacts_index];
      }

      PopulateCostMatricesFromVectors(options);
    }

    void SetPoseTrackingOptions(C3Options* options) const {
      options->dt = planning_dt_pose;
      options->dt_cost = planning_dt_pose / lcs_dt_resolution;
      options->w_Q = w_Q;
      options->w_R = w_R;
      options->w_G = w_G;
      options->w_U = w_U;
      options->q_vector = q_vector;
      options->r_vector = r_vector;

      options->g_x = g_x;
      options->g_gamma = g_gamma_list[num_contacts_index];
      options->g_lambda_n = g_lambda_n_list[num_contacts_index];
      options->g_lambda_t = g_lambda_t_list[num_contacts_index];
      options->g_lambda = g_lambda_list[num_contacts_index];
      options->g_u = g_u;

      options->u_x = u_x;
      options->u_gamma = u_gamma_list[num_contacts_index];
      options->u_lambda_n = u_lambda_n_list[num_contacts_index];
      options->u_lambda_t = u_lambda_t_list[num_contacts_index];
      options->u_lambda = u_lambda_list[num_contacts_index];
      options->u_u = u_u;

      if (options->projection_type == "C3+") {
        options->g_eta_slack = g_eta_slack_list[num_contacts_index];
        options->g_eta_n = g_eta_n_list[num_contacts_index];
        options->g_eta_t = g_eta_t_list[num_contacts_index];
        options->g_eta = g_eta_list[num_contacts_index];
        options->u_eta_slack = u_eta_slack_list[num_contacts_index];
        options->u_eta_n = u_eta_n_list[num_contacts_index];
        options->u_eta_t = u_eta_t_list[num_contacts_index];
        options->u_eta = u_eta_list[num_contacts_index];
      }
      PopulateCostMatricesFromVectors(options);
    }
};
