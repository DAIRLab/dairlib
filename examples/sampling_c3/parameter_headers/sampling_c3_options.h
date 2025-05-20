#pragma once
#include <iostream>

#include "drake/common/yaml/yaml_read_archive.h"

struct SamplingC3Options {
  // Hyperparameters
  int num_outer_threads;  // for outer sampling loop.
  std::string contact_model;
  bool use_predicted_x0_c3;
  bool use_predicted_x0_repos;
  bool at_least_predict_first_planned_trajectory_knot;
  bool use_predicted_x0_reset_mechanism;

  double w_Q;
  double w_R;
  double w_G;
  double w_U;


  bool use_quaternion_dependent_cost;
  double q_quaternion_dependent_weight;
  double q_quaternion_dependent_regularizer_fraction;

  std::vector<double> q_vector_position_and_orientation;
  std::vector<double> r_vector;           // This is common for both position and pose tracking


  std::vector<double> g_vector;
  std::vector<double> g_vector_for_curr_location;
  std::vector<double> g_x;
  std::vector<std::vector<double>> g_gamma;
  std::vector<std::vector<double>> g_lambda_n;
  std::vector<std::vector<double>> g_lambda_t;
  std::vector<std::vector<double>> g_lambda;
  std::vector<double> g_u;


  std::vector<double> u_vector;
  std::vector<double> u_vector_for_curr_location;
  std::vector<double> u_x;
  std::vector<std::vector<double>> u_gamma;
  std::vector<std::vector<double>> u_lambda_n;
  std::vector<std::vector<double>> u_lambda_t;
  std::vector<std::vector<double>> u_lambda;
  std::vector<double> u_u;


  std::vector<double> g_vector_position_tracking;
  std::vector<double> g_vector_position_tracking_for_curr_location;
  std::vector<double> q_vector;
  double gamma_position_tracking;
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

  std::vector<std::vector<double>> mu;
  double planning_dt_position_tracking;           // dt for planning when comparing samples. Used for position tracking.
  double planning_dt;           // dt for planning when comparing samples.
  double execution_dt;          // dt for execution after comparing samples.
  int num_contacts_index;
  int num_contacts_index_for_cost;
  int num_contacts_index_for_curr_location;
  std::vector<int> num_contacts;
  std::vector<std::vector<int>> resolve_contacts_to_list;
  Eigen::MatrixXd Q_position;
  Eigen::MatrixXd Q_position_and_orientation;
  Eigen::MatrixXd R;
  Eigen::MatrixXd G;
  Eigen::MatrixXd G_for_curr_location;
  Eigen::MatrixXd U;
  Eigen::MatrixXd U_for_curr_location;
  Eigen::MatrixXd G_position_tracking;
  Eigen::MatrixXd G_position_tracking_for_curr_location;
  Eigen::MatrixXd U_position_tracking;
  Eigen::MatrixXd U_position_tracking_for_curr_location;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(num_outer_threads));
    a->Visit(DRAKE_NVP(contact_model));
    a->Visit(DRAKE_NVP(use_predicted_x0_c3));
    a->Visit(DRAKE_NVP(use_predicted_x0_repos));
    a->Visit(DRAKE_NVP(at_least_predict_first_planned_trajectory_knot));
    a->Visit(DRAKE_NVP(use_predicted_x0_reset_mechanism));

    a->Visit(DRAKE_NVP(mu));
    a->Visit(DRAKE_NVP(planning_dt_position_tracking));
    a->Visit(DRAKE_NVP(planning_dt));
    a->Visit(DRAKE_NVP(execution_dt));
    a->Visit(DRAKE_NVP(num_contacts_index));
    a->Visit(DRAKE_NVP(num_contacts_index_for_cost));
    a->Visit(DRAKE_NVP(num_contacts_index_for_curr_location));
    a->Visit(DRAKE_NVP(num_contacts));
    a->Visit(DRAKE_NVP(resolve_contacts_to_list));

    a->Visit(DRAKE_NVP(w_Q));
    a->Visit(DRAKE_NVP(w_R));
    a->Visit(DRAKE_NVP(w_G));
    a->Visit(DRAKE_NVP(w_U));
    a->Visit(DRAKE_NVP(gamma_position_tracking));
    a->Visit(DRAKE_NVP(w_Q_position_tracking));
    a->Visit(DRAKE_NVP(w_R_position_tracking));
    a->Visit(DRAKE_NVP(w_G_position_tracking));
    a->Visit(DRAKE_NVP(w_U_position_tracking));

    a->Visit(DRAKE_NVP(use_quaternion_dependent_cost));
    a->Visit(DRAKE_NVP(q_quaternion_dependent_weight));
    a->Visit(DRAKE_NVP(q_quaternion_dependent_regularizer_fraction));
    a->Visit(DRAKE_NVP(q_vector));
    a->Visit(DRAKE_NVP(q_vector_position_and_orientation));
    a->Visit(DRAKE_NVP(r_vector));

    a->Visit(DRAKE_NVP(g_x));
    a->Visit(DRAKE_NVP(g_gamma));
    a->Visit(DRAKE_NVP(g_lambda_n));
    a->Visit(DRAKE_NVP(g_lambda_t));
    a->Visit(DRAKE_NVP(g_lambda));
    a->Visit(DRAKE_NVP(g_u));
    a->Visit(DRAKE_NVP(g_x_position_tracking));
    a->Visit(DRAKE_NVP(g_gamma_position_tracking));
    a->Visit(DRAKE_NVP(g_lambda_n_position_tracking));
    a->Visit(DRAKE_NVP(g_lambda_t_position_tracking));
    a->Visit(DRAKE_NVP(g_lambda_position_tracking));
    a->Visit(DRAKE_NVP(g_u_position_tracking));

    a->Visit(DRAKE_NVP(u_x));
    a->Visit(DRAKE_NVP(u_gamma));
    a->Visit(DRAKE_NVP(u_lambda_n));
    a->Visit(DRAKE_NVP(u_lambda_t));
    a->Visit(DRAKE_NVP(u_lambda));
    a->Visit(DRAKE_NVP(u_u));
    a->Visit(DRAKE_NVP(u_x_position_tracking));
    a->Visit(DRAKE_NVP(u_gamma_position_tracking));
    a->Visit(DRAKE_NVP(u_lambda_n_position_tracking));
    a->Visit(DRAKE_NVP(u_lambda_t_position_tracking));
    a->Visit(DRAKE_NVP(u_lambda_position_tracking));
    a->Visit(DRAKE_NVP(u_u_position_tracking));

    g_vector = std::vector<double>();
    g_vector.insert(g_vector.end(), g_x.begin(), g_x.end());

    g_vector_for_curr_location = std::vector<double>();
    g_vector_for_curr_location.insert(g_vector_for_curr_location.end(), g_x.begin(), g_x.end());

    g_vector_position_tracking = std::vector<double>();
    g_vector_position_tracking.insert(g_vector_position_tracking.end(),
                                      g_x_position_tracking.begin(), g_x_position_tracking.end());

    g_vector_position_tracking_for_curr_location = std::vector<double>();
    g_vector_position_tracking_for_curr_location.insert(g_vector_position_tracking_for_curr_location.end(),
                                      g_x_position_tracking.begin(), g_x_position_tracking.end());

    if (contact_model == "stewart_and_trinkle") {
      g_vector.insert(g_vector.end(), g_gamma[num_contacts_index].begin(), g_gamma[num_contacts_index].end());
      g_vector.insert(g_vector.end(), g_lambda_n[num_contacts_index].begin(), g_lambda_n[num_contacts_index].end());
      g_vector.insert(g_vector.end(), g_lambda_t[num_contacts_index].begin(), g_lambda_t[num_contacts_index].end());

      g_vector_for_curr_location.insert(g_vector_for_curr_location.end(), g_gamma[num_contacts_index_for_curr_location].begin(), g_gamma[num_contacts_index_for_curr_location].end());
      g_vector_for_curr_location.insert(g_vector_for_curr_location.end(), g_lambda_n[num_contacts_index_for_curr_location].begin(), g_lambda_n[num_contacts_index_for_curr_location].end());
      g_vector_for_curr_location.insert(g_vector_for_curr_location.end(), g_lambda_t[num_contacts_index_for_curr_location].begin(), g_lambda_t[num_contacts_index_for_curr_location].end());

      g_vector_position_tracking.insert(g_vector_position_tracking.end(), g_gamma_position_tracking[num_contacts_index].begin(), g_gamma_position_tracking[num_contacts_index].end());
      g_vector_position_tracking.insert(g_vector_position_tracking.end(), g_lambda_n_position_tracking[num_contacts_index].begin(), g_lambda_n_position_tracking[num_contacts_index].end());
      g_vector_position_tracking.insert(g_vector_position_tracking.end(), g_lambda_t_position_tracking[num_contacts_index].begin(), g_lambda_t_position_tracking[num_contacts_index].end());

      g_vector_position_tracking_for_curr_location.insert(g_vector_position_tracking_for_curr_location.end(), g_gamma_position_tracking[num_contacts_index_for_curr_location].begin(), g_gamma_position_tracking[num_contacts_index_for_curr_location].end());
      g_vector_position_tracking_for_curr_location.insert(g_vector_position_tracking_for_curr_location.end(), g_lambda_n_position_tracking[num_contacts_index_for_curr_location].begin(), g_lambda_n_position_tracking[num_contacts_index_for_curr_location].end());
      g_vector_position_tracking_for_curr_location.insert(g_vector_position_tracking_for_curr_location.end(), g_lambda_t_position_tracking[num_contacts_index_for_curr_location].begin(), g_lambda_t_position_tracking[num_contacts_index_for_curr_location].end());

    } else {
      g_vector.insert(g_vector.end(), g_lambda[num_contacts_index].begin(), g_lambda[num_contacts_index].end());
      g_vector_for_curr_location.insert(g_vector_for_curr_location.end(), g_lambda[num_contacts_index_for_curr_location].begin(), g_lambda[num_contacts_index_for_curr_location].end());
      g_vector_position_tracking.insert(g_vector_position_tracking.end(), g_lambda_position_tracking[num_contacts_index].begin(), g_lambda_position_tracking[num_contacts_index].end());
      g_vector_position_tracking_for_curr_location.insert(g_vector_position_tracking_for_curr_location.end(), g_lambda_position_tracking[num_contacts_index_for_curr_location].begin(), g_lambda_position_tracking[num_contacts_index_for_curr_location].end());
    }

    g_vector.insert(g_vector.end(), g_u.begin(), g_u.end());
    g_vector_for_curr_location.insert(g_vector_for_curr_location.end(), g_u.begin(), g_u.end());
    g_vector_position_tracking.insert(g_vector_position_tracking.end(), g_u_position_tracking.begin(), g_u_position_tracking.end());
    g_vector_position_tracking_for_curr_location.insert(g_vector_position_tracking_for_curr_location.end(), g_u_position_tracking.begin(), g_u_position_tracking.end());

    u_vector = std::vector<double>();
    u_vector.insert(u_vector.end(), u_x.begin(), u_x.end());

    u_vector_for_curr_location = std::vector<double>();
    u_vector_for_curr_location.insert(u_vector_for_curr_location.end(), u_x.begin(), u_x.end());

    u_vector_position_tracking = std::vector<double>();
    u_vector_position_tracking.insert(u_vector_position_tracking.end(), u_x_position_tracking.begin(), u_x_position_tracking.end());

    u_vector_position_tracking_for_curr_location = std::vector<double>();
    u_vector_position_tracking_for_curr_location.insert(u_vector_position_tracking_for_curr_location.end(), u_x_position_tracking.begin(), u_x_position_tracking.end());

    if (contact_model == "stewart_and_trinkle") {
      u_vector.insert(u_vector.end(), u_gamma[num_contacts_index].begin(), u_gamma[num_contacts_index].end());
      u_vector.insert(u_vector.end(), u_lambda_n[num_contacts_index].begin(), u_lambda_n[num_contacts_index].end());
      u_vector.insert(u_vector.end(), u_lambda_t[num_contacts_index].begin(), u_lambda_t[num_contacts_index].end());

      u_vector_for_curr_location.insert(u_vector_for_curr_location.end(), u_gamma[num_contacts_index_for_curr_location].begin(), u_gamma[num_contacts_index_for_curr_location].end());
      u_vector_for_curr_location.insert(u_vector_for_curr_location.end(), u_lambda_n[num_contacts_index_for_curr_location].begin(), u_lambda_n[num_contacts_index_for_curr_location].end());
      u_vector_for_curr_location.insert(u_vector_for_curr_location.end(), u_lambda_t[num_contacts_index_for_curr_location].begin(), u_lambda_t[num_contacts_index_for_curr_location].end());

      u_vector_position_tracking.insert(u_vector_position_tracking.end(), u_gamma_position_tracking[num_contacts_index].begin(), u_gamma_position_tracking[num_contacts_index].end());
      u_vector_position_tracking.insert(u_vector_position_tracking.end(), u_lambda_n_position_tracking[num_contacts_index].begin(), u_lambda_n_position_tracking[num_contacts_index].end());
      u_vector_position_tracking.insert(u_vector_position_tracking.end(), u_lambda_t_position_tracking[num_contacts_index].begin(), u_lambda_t_position_tracking[num_contacts_index].end());

      u_vector_position_tracking_for_curr_location.insert(u_vector_position_tracking_for_curr_location.end(), u_gamma_position_tracking[num_contacts_index_for_curr_location].begin(), u_gamma_position_tracking[num_contacts_index_for_curr_location].end());
      u_vector_position_tracking_for_curr_location.insert(u_vector_position_tracking_for_curr_location.end(), u_lambda_n_position_tracking[num_contacts_index_for_curr_location].begin(), u_lambda_n_position_tracking[num_contacts_index_for_curr_location].end());
      u_vector_position_tracking_for_curr_location.insert(u_vector_position_tracking_for_curr_location.end(), u_lambda_t_position_tracking[num_contacts_index_for_curr_location].begin(), u_lambda_t_position_tracking[num_contacts_index_for_curr_location].end());

    } else {
      u_vector.insert(u_vector.end(), u_lambda[num_contacts_index].begin(), u_lambda[num_contacts_index].end());
      u_vector_for_curr_location.insert(u_vector_for_curr_location.end(), u_lambda[num_contacts_index_for_curr_location].begin(), u_lambda[num_contacts_index_for_curr_location].end());
      u_vector_position_tracking.insert(u_vector_position_tracking.end(), u_lambda_position_tracking[num_contacts_index].begin(), u_lambda_position_tracking[num_contacts_index].end());
      u_vector_position_tracking_for_curr_location.insert(u_vector_position_tracking_for_curr_location.end(), u_lambda_position_tracking[num_contacts_index_for_curr_location].begin(), u_lambda_position_tracking[num_contacts_index_for_curr_location].end());
    }
    u_vector.insert(u_vector.end(), u_u.begin(), u_u.end());
    u_vector_for_curr_location.insert(u_vector_for_curr_location.end(), u_u.begin(), u_u.end());
    u_vector_position_tracking.insert(u_vector_position_tracking.end(), u_u_position_tracking.begin(), u_u_position_tracking.end());
    u_vector_position_tracking_for_curr_location.insert(u_vector_position_tracking_for_curr_location.end(), u_u_position_tracking.begin(), u_u_position_tracking.end());

    Eigen::VectorXd q = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        this->q_vector.data(), this->q_vector.size());
    Eigen::VectorXd q_position_and_orientation = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        this->q_vector_position_and_orientation.data(), this->q_vector_position_and_orientation.size());

    Eigen::VectorXd r = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        this->r_vector.data(), this->r_vector.size());

    Eigen::VectorXd g = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        this->g_vector.data(), this->g_vector.size());
    Eigen::VectorXd g_for_curr_location = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        this->g_vector_for_curr_location.data(), this->g_vector_for_curr_location.size());
    Eigen::VectorXd g_position_tracking = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        this->g_vector_position_tracking.data(), this->g_vector_position_tracking.size());
    Eigen::VectorXd g_position_tracking_for_curr_location = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        this->g_vector_position_tracking_for_curr_location.data(), this->g_vector_position_tracking_for_curr_location.size());

    Eigen::VectorXd u = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        this->u_vector.data(), this->u_vector.size());
    Eigen::VectorXd u_for_curr_location = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        this->u_vector_for_curr_location.data(), this->u_vector_for_curr_location.size());
    Eigen::VectorXd u_position_tracking = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        this->u_vector_position_tracking.data(), this->u_vector_position_tracking.size());
    Eigen::VectorXd u_position_tracking_for_curr_location = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        this->u_vector_position_tracking_for_curr_location.data(), this->u_vector_position_tracking_for_curr_location.size());

    Q_position = w_Q * q.asDiagonal();
    Q_position_and_orientation = w_Q * q_position_and_orientation.asDiagonal();
    R = w_R * r.asDiagonal();
    G = w_G * g.asDiagonal();
    G_for_curr_location = w_G * g_for_curr_location.asDiagonal();
    G_position_tracking = w_G_position_tracking * g_position_tracking.asDiagonal();
    G_position_tracking_for_curr_location = w_G_position_tracking * g_position_tracking_for_curr_location.asDiagonal();
    U = w_U * u.asDiagonal();
    U_for_curr_location = w_U * u_for_curr_location.asDiagonal();
    U_position_tracking = w_U_position_tracking * u_position_tracking.asDiagonal();
    U_position_tracking_for_curr_location = w_U_position_tracking * u_position_tracking_for_curr_location.asDiagonal();
  }
};