#pragma once
#include <iostream>

#include "drake/common/yaml/yaml_read_archive.h"

struct C3Options {
  // Hyperparameters
  int admm_iter;     // total number of ADMM iterations
  float rho;         // initial value of the rho parameter
  float rho_scale;   // scaling of rho parameter (/rho = rho_scale * /rho)
  int num_threads;   // for inner C3 loop.
                     // 0 is dynamic, greater than 0 for a fixed count
  int num_outer_threads;  // for outer sampling loop.
  // int delta_option;  // different options for delta update
  std::string projection_type;
  std::string contact_model;
  bool warm_start;
  bool use_predicted_x0;
  bool use_predicted_x0_c3;
  bool use_predicted_x0_repos;
  bool at_least_predict_first_planned_trajectory_knot;
  double solve_time_filter_alpha;

  std::vector<double> world_x_limits;
  std::vector<double> world_y_limits;
  std::vector<double> world_z_limits;
  std::vector<double> u_horizontal_limits;
  std::vector<double> u_vertical_limits;

  double ee_z_state_min;
  double workspace_margins;

  int N;
  double gamma;
  double w_Q;
  double w_R;
  double w_G;
  double w_U;

  std::vector<double> q_vector;
  std::vector<double> q_vector_position_and_orientation;
  std::vector<double> r_vector;
  std::vector<double> g_vector;
  std::vector<double> g_x;
  std::vector<std::vector<double>> g_gamma;
  std::vector<std::vector<double>> g_lambda_n;
  std::vector<std::vector<double>> g_lambda_t;
  std::vector<std::vector<double>> g_lambda;

  std::vector<double> g_u;
  std::vector<double> u_vector;
  std::vector<double> u_x;
  std::vector<std::vector<double>> u_gamma;
  std::vector<std::vector<double>> u_lambda_n;
  std::vector<std::vector<double>> u_lambda_t;
  std::vector<std::vector<double>> u_lambda;
  std::vector<double> u_u;

  double qp_projection_alpha;
  double qp_projection_scaling;

  std::vector<std::vector<double>> mu;
  double dt;                    // dt for everything if not using sampling-based
                                // C3 controller.
  double planning_dt;           // dt for planning when comparing samples.
  double execution_dt;          // dt for execution after comparing samples.
  int num_friction_directions;
  int num_contacts_index;
  std::vector<int> num_contacts;
  std::vector<std::vector<int>> resolve_contacts_to_list;
  Eigen::MatrixXd Q_position;
  Eigen::MatrixXd Q_position_and_orientation;
  Eigen::MatrixXd R;
  Eigen::MatrixXd G;
  Eigen::MatrixXd U;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(admm_iter));
    a->Visit(DRAKE_NVP(rho));
    a->Visit(DRAKE_NVP(rho_scale));
    a->Visit(DRAKE_NVP(num_threads));
    a->Visit(DRAKE_NVP(num_outer_threads));
    // a->Visit(DRAKE_NVP(delta_option));
    a->Visit(DRAKE_NVP(contact_model));
    a->Visit(DRAKE_NVP(projection_type));
    if (projection_type == "QP") {
      DRAKE_DEMAND(contact_model == "anitescu");
    }
    a->Visit(DRAKE_NVP(warm_start));
    a->Visit(DRAKE_NVP(use_predicted_x0));
    a->Visit(DRAKE_NVP(use_predicted_x0_c3));
    a->Visit(DRAKE_NVP(use_predicted_x0_repos));
    a->Visit(DRAKE_NVP(at_least_predict_first_planned_trajectory_knot));
    a->Visit(DRAKE_NVP(solve_time_filter_alpha));

    a->Visit(DRAKE_NVP(world_x_limits));
    a->Visit(DRAKE_NVP(world_y_limits));
    a->Visit(DRAKE_NVP(world_z_limits));
    a->Visit(DRAKE_NVP(u_horizontal_limits));
    a->Visit(DRAKE_NVP(u_vertical_limits));
    a->Visit(DRAKE_NVP(ee_z_state_min));
    a->Visit(DRAKE_NVP(workspace_margins));

    a->Visit(DRAKE_NVP(mu));
    a->Visit(DRAKE_NVP(dt));
    a->Visit(DRAKE_NVP(planning_dt));
    a->Visit(DRAKE_NVP(execution_dt));
    a->Visit(DRAKE_NVP(num_friction_directions));
    a->Visit(DRAKE_NVP(num_contacts_index));
    a->Visit(DRAKE_NVP(num_contacts));
    a->Visit(DRAKE_NVP(resolve_contacts_to_list));

    a->Visit(DRAKE_NVP(N));
    a->Visit(DRAKE_NVP(gamma));
    a->Visit(DRAKE_NVP(w_Q));
    a->Visit(DRAKE_NVP(w_R));
    a->Visit(DRAKE_NVP(w_G));
    a->Visit(DRAKE_NVP(w_U));
    a->Visit(DRAKE_NVP(q_vector));
    a->Visit(DRAKE_NVP(q_vector_position_and_orientation));
    a->Visit(DRAKE_NVP(r_vector));
    a->Visit(DRAKE_NVP(g_x));
    a->Visit(DRAKE_NVP(g_gamma));
    a->Visit(DRAKE_NVP(g_lambda_n));
    a->Visit(DRAKE_NVP(g_lambda_t));
    a->Visit(DRAKE_NVP(g_lambda));
    a->Visit(DRAKE_NVP(g_u));
    a->Visit(DRAKE_NVP(u_x));
    a->Visit(DRAKE_NVP(u_gamma));
    a->Visit(DRAKE_NVP(u_lambda_n));
    a->Visit(DRAKE_NVP(u_lambda_t));
    a->Visit(DRAKE_NVP(u_lambda));
    a->Visit(DRAKE_NVP(u_u));
    a->Visit(DRAKE_NVP(qp_projection_alpha));
    a->Visit(DRAKE_NVP(qp_projection_scaling));

    g_vector = std::vector<double>();
    g_vector.insert(g_vector.end(), g_x.begin(), g_x.end());
    if (contact_model == "stewart_and_trinkle") {
      g_vector.insert(g_vector.end(), g_gamma[num_contacts_index].begin(), g_gamma[num_contacts_index].end());
      g_vector.insert(g_vector.end(), g_lambda_n[num_contacts_index].begin(), g_lambda_n[num_contacts_index].end());
      g_vector.insert(g_vector.end(), g_lambda_t[num_contacts_index].begin(), g_lambda_t[num_contacts_index].end());
    } else {
      g_vector.insert(g_vector.end(), g_lambda[num_contacts_index].begin(), g_lambda[num_contacts_index].end());
    }

    g_vector.insert(g_vector.end(), g_u.begin(), g_u.end());
    u_vector = std::vector<double>();
    u_vector.insert(u_vector.end(), u_x.begin(), u_x.end());
    if (contact_model == "stewart_and_trinkle") {
      u_vector.insert(u_vector.end(), u_gamma[num_contacts_index].begin(), u_gamma[num_contacts_index].end());
      u_vector.insert(u_vector.end(), u_lambda_n[num_contacts_index].begin(), u_lambda_n[num_contacts_index].end());
      u_vector.insert(u_vector.end(), u_lambda_t[num_contacts_index].begin(), u_lambda_t[num_contacts_index].end());
    } else {
      u_vector.insert(u_vector.end(), u_lambda[num_contacts_index].begin(), u_lambda[num_contacts_index].end());
    }
    u_vector.insert(u_vector.end(), u_u.begin(), u_u.end());

    Eigen::VectorXd q = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        this->q_vector.data(), this->q_vector.size());
    Eigen::VectorXd q_position_and_orientation = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        this->q_vector_position_and_orientation.data(), this->q_vector_position_and_orientation.size());
    Eigen::VectorXd r = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        this->r_vector.data(), this->r_vector.size());
    Eigen::VectorXd g = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        this->g_vector.data(), this->g_vector.size());
    Eigen::VectorXd u = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        this->u_vector.data(), this->u_vector.size());

    Q_position = w_Q * q.asDiagonal();
    Q_position_and_orientation = w_Q * q_position_and_orientation.asDiagonal();
    R = w_R * r.asDiagonal();
    G = w_G * g.asDiagonal();
    U = w_U * u.asDiagonal();
  }
};