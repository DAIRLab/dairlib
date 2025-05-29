#pragma once
#include <iostream>

#include "solvers/c3_options.h"

#include "drake/common/yaml/yaml_read_archive.h"

struct SamplingC3Options {
  // Hyperparameters
  int admm_iter;          // total number of ADMM iterations
  float rho;              // initial value of the rho parameter
  float rho_scale;        // scaling of rho parameter (/rho = rho_scale * /rho)
  int num_threads;        // for inner C3 loop.
                          // 0 is dynamic, greater than 0 for a fixed count
  int num_outer_threads;  // for outer sampling loop.
  int delta_option;       // different options for delta update
  std::string projection_type;
  std::string contact_model;
  bool warm_start;
  bool use_predicted_x0;
  bool end_on_qp_step;
  bool use_predicted_x0_c3;
  bool use_predicted_x0_repos;
  bool at_least_predict_first_planned_trajectory_knot;
  bool use_predicted_x0_reset_mechanism;
  double solve_time_filter_alpha;
  double publish_frequency;

  std::vector<Eigen::VectorXd> workspace_limits;
  std::vector<double> robot_radius_limits;
  double workspace_margins;
  std::vector<double> u_horizontal_limits;
  std::vector<double> u_vertical_limits;

  int N;
  double gamma;

  double w_Q_pose_tracking;
  double w_R_pose_tracking;
  double w_G_pose_tracking;
  double w_U_pose_tracking;

  // Parameters used for cost computation function
  double Kp_for_cost_type_3;
  double Kd_for_cost_type_3;

  bool use_quaternion_dependent_cost;
  double q_quaternion_dependent_weight;
  double q_quaternion_dependent_regularizer_fraction;

  std::vector<double> q_vector_pose_tracking;
  std::vector<double>
      r_vector;  // This is common for both position and pose tracking

  std::vector<double> g_vector_pose_tracking;
  std::vector<double> g_vector_pose_tracking_for_curr_location;
  std::vector<double> g_x_pose_tracking;
  std::vector<std::vector<double>> g_gamma_pose_tracking;
  std::vector<std::vector<double>> g_lambda_n_pose_tracking;
  std::vector<std::vector<double>> g_lambda_t_pose_tracking;
  std::vector<std::vector<double>> g_lambda_pose_tracking;
  std::vector<double> g_u_pose_tracking;

  std::vector<double> u_vector_pose_tracking;
  std::vector<double> u_vector_pose_tracking_for_curr_location;
  std::vector<double> u_x_pose_tracking;
  std::vector<std::vector<double>> u_gamma_pose_tracking;
  std::vector<std::vector<double>> u_lambda_n_pose_tracking;
  std::vector<std::vector<double>> u_lambda_t_pose_tracking;
  std::vector<std::vector<double>> u_lambda_pose_tracking;
  std::vector<double> u_u_pose_tracking;

  std::vector<double> g_vector_position_tracking;
  std::vector<double> g_vector_position_tracking_for_curr_location;
  std::vector<double> q_vector_position_tracking;
  double w_Q_position_tracking;
  double w_R_position_tracking;
  double w_G_position_tracking;
  double w_U_position_tracking;

  std::vector<double> g_x_position_tracking;
  std::vector<std::vector<double>> g_gamma_position_tracking;
  std::vector<std::vector<double>> g_lambda_n_position_tracking;
  std::vector<std::vector<double>> g_lambda_t_position_tracking;
  std::vector<std::vector<double>> g_lambda_position_tracking;
  std::vector<double> g_u_position_tracking;

  std::vector<double> u_vector_position_tracking;
  std::vector<double> u_vector_position_tracking_for_curr_location;
  std::vector<double> u_x_position_tracking;
  std::vector<std::vector<double>> u_gamma_position_tracking;
  std::vector<std::vector<double>> u_lambda_n_position_tracking;
  std::vector<std::vector<double>> u_lambda_t_position_tracking;
  std::vector<std::vector<double>> u_lambda_position_tracking;
  std::vector<double> u_u_position_tracking;

  double qp_projection_alpha;
  double qp_projection_scaling;
  bool penalize_deviation_from_previous_input_solution;

  std::vector<std::vector<double>> mu_list;
  double planning_dt_pose_tracking;
  double planning_dt_position_tracking;
  int num_friction_directions;
  int num_contacts_index;
  int num_contacts_index_for_cost;
  int num_contacts_index_for_curr_location;
  std::vector<int> num_contacts_list;
  std::vector<std::vector<int>> resolve_contacts_to_list;
  Eigen::MatrixXd Q_pose_tracking;
  Eigen::MatrixXd Q_position_tracking;
  Eigen::MatrixXd R;
  Eigen::MatrixXd G_pose_tracking;
  Eigen::MatrixXd G_pose_tracking_for_curr_location;
  Eigen::MatrixXd U_pose_tracking;
  Eigen::MatrixXd U_pose_tracking_for_curr_location;
  Eigen::MatrixXd G_position_tracking;
  Eigen::MatrixXd G_position_tracking_for_curr_location;
  Eigen::MatrixXd U_position_tracking;
  Eigen::MatrixXd U_position_tracking_for_curr_location;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(admm_iter));
    a->Visit(DRAKE_NVP(rho));
    a->Visit(DRAKE_NVP(rho_scale));
    a->Visit(DRAKE_NVP(num_threads));
    a->Visit(DRAKE_NVP(num_outer_threads));
    a->Visit(DRAKE_NVP(delta_option));
    a->Visit(DRAKE_NVP(projection_type));
    a->Visit(DRAKE_NVP(contact_model));
    a->Visit(DRAKE_NVP(warm_start));
    a->Visit(DRAKE_NVP(end_on_qp_step));
    a->Visit(DRAKE_NVP(use_predicted_x0_c3));
    a->Visit(DRAKE_NVP(use_predicted_x0_repos));
    a->Visit(DRAKE_NVP(at_least_predict_first_planned_trajectory_knot));
    a->Visit(DRAKE_NVP(use_predicted_x0_reset_mechanism));
    a->Visit(DRAKE_NVP(solve_time_filter_alpha));
    a->Visit(DRAKE_NVP(publish_frequency));
    a->Visit(DRAKE_NVP(workspace_limits));
    a->Visit(DRAKE_NVP(workspace_margins));
    a->Visit(DRAKE_NVP(robot_radius_limits));
    a->Visit(DRAKE_NVP(u_horizontal_limits));
    a->Visit(DRAKE_NVP(u_vertical_limits));
    a->Visit(DRAKE_NVP(N));
    a->Visit(DRAKE_NVP(gamma));

    a->Visit(DRAKE_NVP(mu_list));
    a->Visit(DRAKE_NVP(planning_dt_pose_tracking));
    a->Visit(DRAKE_NVP(planning_dt_position_tracking));
    a->Visit(DRAKE_NVP(num_friction_directions));
    a->Visit(DRAKE_NVP(num_contacts_index));
    a->Visit(DRAKE_NVP(num_contacts_index_for_cost));
    a->Visit(DRAKE_NVP(num_contacts_index_for_curr_location));
    a->Visit(DRAKE_NVP(num_contacts_list));
    a->Visit(DRAKE_NVP(resolve_contacts_to_list));

    a->Visit(DRAKE_NVP(w_Q_pose_tracking));
    a->Visit(DRAKE_NVP(w_R_pose_tracking));
    a->Visit(DRAKE_NVP(w_G_pose_tracking));
    a->Visit(DRAKE_NVP(w_U_pose_tracking));
    a->Visit(DRAKE_NVP(w_Q_position_tracking));
    a->Visit(DRAKE_NVP(w_R_position_tracking));
    a->Visit(DRAKE_NVP(w_G_position_tracking));
    a->Visit(DRAKE_NVP(w_U_position_tracking));

    a->Visit(DRAKE_NVP(Kp_for_cost_type_3));
    a->Visit(DRAKE_NVP(Kd_for_cost_type_3));

    a->Visit(DRAKE_NVP(use_quaternion_dependent_cost));
    a->Visit(DRAKE_NVP(q_quaternion_dependent_weight));
    a->Visit(DRAKE_NVP(q_quaternion_dependent_regularizer_fraction));
    a->Visit(DRAKE_NVP(q_vector_pose_tracking));
    a->Visit(DRAKE_NVP(q_vector_position_tracking));
    a->Visit(DRAKE_NVP(r_vector));

    a->Visit(DRAKE_NVP(g_x_pose_tracking));
    a->Visit(DRAKE_NVP(g_gamma_pose_tracking));
    a->Visit(DRAKE_NVP(g_lambda_n_pose_tracking));
    a->Visit(DRAKE_NVP(g_lambda_t_pose_tracking));
    a->Visit(DRAKE_NVP(g_lambda_pose_tracking));
    a->Visit(DRAKE_NVP(g_u_pose_tracking));
    a->Visit(DRAKE_NVP(g_x_position_tracking));
    a->Visit(DRAKE_NVP(g_gamma_position_tracking));
    a->Visit(DRAKE_NVP(g_lambda_n_position_tracking));
    a->Visit(DRAKE_NVP(g_lambda_t_position_tracking));
    a->Visit(DRAKE_NVP(g_lambda_position_tracking));
    a->Visit(DRAKE_NVP(g_u_position_tracking));

    a->Visit(DRAKE_NVP(u_x_pose_tracking));
    a->Visit(DRAKE_NVP(u_gamma_pose_tracking));
    a->Visit(DRAKE_NVP(u_lambda_n_pose_tracking));
    a->Visit(DRAKE_NVP(u_lambda_t_pose_tracking));
    a->Visit(DRAKE_NVP(u_lambda_pose_tracking));
    a->Visit(DRAKE_NVP(u_u_pose_tracking));
    a->Visit(DRAKE_NVP(u_x_position_tracking));
    a->Visit(DRAKE_NVP(u_gamma_position_tracking));
    a->Visit(DRAKE_NVP(u_lambda_n_position_tracking));
    a->Visit(DRAKE_NVP(u_lambda_t_position_tracking));
    a->Visit(DRAKE_NVP(u_lambda_position_tracking));
    a->Visit(DRAKE_NVP(u_u_position_tracking));

    a->Visit(DRAKE_NVP(qp_projection_alpha));
    a->Visit(DRAKE_NVP(qp_projection_scaling));
    a->Visit(DRAKE_NVP(penalize_deviation_from_previous_input_solution));
  }

  C3Options GetC3Options(bool is_pose_tracking, int num_contacts_index) const {
    C3Options c3_options;
    SetCommonC3Options(&c3_options, num_contacts_index);
    if (is_pose_tracking) {
      SetPoseTrackingOptions(c3_options, num_contacts_index);
    } else {
      SetPositionTrackingOptions(c3_options, num_contacts_index);
    }
    return c3_options;
  }
  // Function to populate the cost matrices from the vectors in C3Options.
  // This mimics the serialization done in the C3Options struct when reading
  // from the YAML file.
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

  // Function to set the common parameters for the C3Options object
  void SetCommonC3Options(C3Options* options, int num_contacts_index) const {
    options->admm_iter = admm_iter;
    options->rho = rho;
    options->rho_scale = rho_scale;
    options->num_threads = num_threads;
    options->delta_option = delta_option;
    options->contact_model = contact_model;
    options->projection_type = projection_type;
    options->warm_start = warm_start;
    options->use_predicted_x0 = false;  // Not used by sampling C3
    options->end_on_qp_step = end_on_qp_step;
    options->use_robust_formulation = false;
    options->solve_time_filter_alpha = solve_time_filter_alpha;
    options->publish_frequency = publish_frequency;

    options->workspace_limits = workspace_limits;
    options->workspace_margins = workspace_margins;
    options->u_horizontal_limits = u_horizontal_limits;
    options->u_vertical_limits = u_vertical_limits;

    options->N = N;
    options->gamma = gamma;

    options->solve_dt = 0;  // Not used by sampling C3
    options->num_friction_directions = num_friction_directions;

    options->qp_projection_alpha = qp_projection_alpha;
    options->qp_projection_scaling = qp_projection_scaling;
    options->penalize_deviation_from_previous_input_solution =
        penalize_deviation_from_previous_input_solution;

    options->mu = mu_list[num_contacts_index];
    options->num_contacts = num_contacts_list[num_contacts_index];
  }

  // Function to set position tracking parameters
  void SetPositionTrackingOptions(C3Options& options,
                                  int num_contacts_index) const {
    options.dt = planning_dt_position_tracking;
    options.w_Q = w_Q_position_tracking;
    options.w_R = w_R_position_tracking;
    options.w_G = w_G_position_tracking;
    options.w_U = w_U_position_tracking;
    options.q_vector = q_vector_position_tracking;
    options.r_vector = r_vector;

    options.g_x = g_x_position_tracking;
    options.g_gamma = g_gamma_position_tracking[num_contacts_index];
    options.g_lambda_n = g_lambda_n_position_tracking[num_contacts_index];
    options.g_lambda_t = g_lambda_t_position_tracking[num_contacts_index];
    options.g_lambda = g_lambda_position_tracking[num_contacts_index];
    options.g_u = g_u_position_tracking;

    options.u_x = u_x_position_tracking;
    options.u_gamma = u_gamma_position_tracking[num_contacts_index];
    options.u_lambda_n = u_lambda_n_position_tracking[num_contacts_index];
    options.u_lambda_t = u_lambda_t_position_tracking[num_contacts_index];
    options.u_lambda = u_lambda_position_tracking[num_contacts_index];
    options.u_u = u_u_position_tracking;

    PopulateCostMatricesFromVectors(&options);
  }

  // Function to set pose tracking parameters
  void SetPoseTrackingOptions(C3Options& options,
                              int num_contacts_index) const {
    options.dt = planning_dt_pose_tracking;
    options.w_Q = w_Q_pose_tracking;
    options.w_R = w_R_pose_tracking;
    options.w_G = w_G_pose_tracking;
    options.w_U = w_U_pose_tracking;
    options.q_vector = q_vector_pose_tracking;
    options.r_vector = r_vector;

    options.g_x = g_x_pose_tracking;
    options.g_gamma = g_gamma_pose_tracking[num_contacts_index];
    options.g_lambda_n = g_lambda_n_pose_tracking[num_contacts_index];
    options.g_lambda_t = g_lambda_t_pose_tracking[num_contacts_index];
    options.g_lambda = g_lambda_pose_tracking[num_contacts_index];
    options.g_u = g_u_pose_tracking;

    options.u_x = u_x_pose_tracking;
    options.u_gamma = u_gamma_pose_tracking[num_contacts_index];
    options.u_lambda_n = u_lambda_n_pose_tracking[num_contacts_index];
    options.u_lambda_t = u_lambda_t_pose_tracking[num_contacts_index];
    options.u_lambda = u_lambda_pose_tracking[num_contacts_index];
    options.u_u = u_u_pose_tracking;

    PopulateCostMatricesFromVectors(&options);
  }
};
