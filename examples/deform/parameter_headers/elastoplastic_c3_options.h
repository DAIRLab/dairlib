#pragma once
#include <numeric>

#include "solvers/c3_options.h"

struct ElastoPlasticC3Options : C3Options {
  /// Contact pair parameters.
  std::vector<double> mu_per_pair_type;
  std::vector<std::vector<int>> resolve_contacts_to_lists;
  std::vector<int> resolve_as_planar_contacts_list;
  std::vector<int> resolve_contacts_to;
  int num_contacts_index;

  int num_planar_contacts;
  int n_lambda_with_tangential;
  std::vector<int> num_friction_directions_per_contact;
  std::vector<int> starting_index_per_contact_in_lambda_t_vector;

  Eigen::VectorXd q_target;
  std::vector<std::string> state_names;
  Eigen::VectorXd Kp;
  Eigen::VectorXd Kd;
  double w_Q_final;

  /// Cost parameters.
  bool use_quaternion_dependent_cost;
  double q_quaternion_dependent_weight;
  double q_quaternion_dependent_regularizer_fraction;

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

  C3Options c3_options;

  template <typename Archive>
  void Serialize(Archive* a) {
    C3Options::Serialize(a);
    a->Visit(DRAKE_NVP(mu_per_pair_type));
    a->Visit(DRAKE_NVP(resolve_contacts_to_lists));
    a->Visit(DRAKE_NVP(resolve_as_planar_contacts_list));
    a->Visit(DRAKE_NVP(num_contacts_index));

    a->Visit(DRAKE_NVP(q_target));
    a->Visit(DRAKE_NVP(state_names));
    a->Visit(DRAKE_NVP(Kp));
    a->Visit(DRAKE_NVP(Kd));
    a->Visit(DRAKE_NVP(w_Q_final));

    a->Visit(DRAKE_NVP(use_quaternion_dependent_cost));
    a->Visit(DRAKE_NVP(q_quaternion_dependent_weight));
    a->Visit(DRAKE_NVP(q_quaternion_dependent_regularizer_fraction));

    a->Visit(DRAKE_NVP(g_gamma_list));
    a->Visit(DRAKE_NVP(g_lambda_n_list));
    a->Visit(DRAKE_NVP(g_lambda_t_list));
    a->Visit(DRAKE_NVP(g_lambda_list));

    a->Visit(DRAKE_NVP(u_gamma_list));
    a->Visit(DRAKE_NVP(u_lambda_n_list));
    a->Visit(DRAKE_NVP(u_lambda_t_list));
    a->Visit(DRAKE_NVP(u_lambda_list));

    // Only applicable for C3+
    a->Visit(DRAKE_NVP(g_eta_slack_list));
    a->Visit(DRAKE_NVP(g_eta_n_list));
    a->Visit(DRAKE_NVP(g_eta_t_list));
    a->Visit(DRAKE_NVP(g_eta_list));
    a->Visit(DRAKE_NVP(u_eta_slack_list));
    a->Visit(DRAKE_NVP(u_eta_n_list));
    a->Visit(DRAKE_NVP(u_eta_t_list));
    a->Visit(DRAKE_NVP(u_eta_list));

    resolve_contacts_to = resolve_contacts_to_lists[num_contacts_index];

    num_contacts = std::accumulate(resolve_contacts_to.begin(),
                                   resolve_contacts_to.end(), 0);
    mu.clear();
    for (size_t i = 0; i < mu_per_pair_type.size(); ++i) {
      int repeat = resolve_contacts_to[i];
      mu.insert(mu.end(), repeat, mu_per_pair_type[i]);
    }

    // Process planar contact info
    std::tie(num_planar_contacts, num_friction_directions_per_contact) =
        ProcessPlanarContactInformation(resolve_as_planar_contacts_list,
                                        resolve_contacts_to,
                                        num_friction_directions);

    for (size_t i = 0; i < num_contacts; ++i) {
      starting_index_per_contact_in_lambda_t_vector.push_back(
          2 * std::accumulate(num_friction_directions_per_contact.begin(),
                              num_friction_directions_per_contact.begin() + i,
                              0));
    }

    n_lambda_with_tangential =
        2 * num_friction_directions * (num_contacts - num_planar_contacts) +
        2 * num_planar_contacts;

    SetC3Options(&c3_options);

    std::cout << "g_lambda size: " << g_lambda.size() << std::endl;
  }

  C3Options GetC3Options() const { return c3_options; }

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

  void SetC3Options(C3Options* options) const {
    options->admm_iter = admm_iter;
    options->rho = rho;
    options->rho_scale = rho_scale;
    options->num_threads = num_threads;
    options->delta_option = delta_option;
    options->contact_model = contact_model;
    options->projection_type = projection_type;
    options->warm_start = warm_start;
    options->use_predicted_x0 = use_predicted_x0;
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
    options->lcs_dt_resolution = 1;
    options->num_friction_directions = num_friction_directions;

    options->qp_projection_alpha = qp_projection_alpha;
    options->qp_projection_scaling = qp_projection_scaling;
    options->penalize_changes_in_u_across_solves =
        penalize_changes_in_u_across_solves;

    options->mu = mu;
    options->num_contacts = num_contacts;

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
    std::cout << "SetC3Options g_lambda size: " << options->g_lambda.size()
              << std::endl;
    MakePlanarLambdaCost(options);
    std::cout << "SetC3Options g_lambda size: " << options->g_lambda.size()
              << std::endl;
    PopulateCostMatricesFromVectors(options);
    std::cout << "SetC3Options g_lambda size: " << options->g_lambda.size()
              << std::endl;
  }

  // Convert lambda weights to the planar form:
  void MakePlanarLambdaCost(C3Options* options) const {
    int offset = 0;
    for (size_t i = 0; i < resolve_contacts_to_lists[num_contacts_index].size();
         ++i) {
      if (resolve_as_planar_contacts_list[i]) {
        int erase_start_index_for_lambda =
            2 * num_friction_directions *
                std::accumulate(
                    resolve_contacts_to_lists[num_contacts_index].begin(),
                    resolve_contacts_to_lists[num_contacts_index].begin() + i,
                    0) -
            offset;
        int erase_end_index_for_lambda =
            erase_start_index_for_lambda +
            2 * (num_friction_directions - 1) *
                resolve_contacts_to_lists[num_contacts_index][i];

        options->g_lambda.erase(
            options->g_lambda.begin() + erase_start_index_for_lambda,
            options->g_lambda.begin() + erase_end_index_for_lambda);
        options->u_lambda.erase(
            options->u_lambda.begin() + erase_start_index_for_lambda,
            options->u_lambda.begin() + erase_end_index_for_lambda);
        options->g_lambda_t.erase(
            options->g_lambda_t.begin() + erase_start_index_for_lambda,
            options->g_lambda_t.begin() + erase_end_index_for_lambda);
        options->u_lambda_t.erase(
            options->u_lambda_t.begin() + erase_start_index_for_lambda,
            options->u_lambda_t.begin() + erase_end_index_for_lambda);

        if (options->projection_type == "C3+") {
          options->g_eta.erase(
              options->g_eta.begin() + erase_start_index_for_lambda,
              options->g_eta.begin() + erase_end_index_for_lambda);
          options->u_eta.erase(
              options->u_eta.begin() + erase_start_index_for_lambda,
              options->u_eta.begin() + erase_end_index_for_lambda);
          options->g_eta_t.erase(
              options->g_eta_t.begin() + erase_start_index_for_lambda,
              options->g_eta_t.begin() + erase_end_index_for_lambda);
          options->u_eta_t.erase(
              options->u_eta_t.begin() + erase_start_index_for_lambda,
              options->u_eta_t.begin() + erase_end_index_for_lambda);
        }
        offset += 2 * (num_friction_directions - 1) *
                  resolve_contacts_to_lists[num_contacts_index][i];
      }
    }
  }

  // Compute total number of planar friction directions and create a vector that
  // contains the number of friction directions for each contact point.
  std::pair<int, std::vector<int>> ProcessPlanarContactInformation(
      const std::vector<int>& resolve_as_planar_contacts_list,
      const std::vector<int>& resolve_contacts_to_list,
      int num_friction_directions) {
    int num_planar_contacts = 0;
    int planar_contact = 1;
    std::vector<int> num_friction_directions_per_contact;
    for (int i = 0; i < resolve_contacts_to_list.size(); ++i) {
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
