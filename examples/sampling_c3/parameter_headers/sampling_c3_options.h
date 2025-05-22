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

  double w_Q_pose_tracking;
  double w_R_pose_tracking;
  double w_G_pose_tracking;
  double w_U_pose_tracking;


  bool use_quaternion_dependent_cost;
  double q_quaternion_dependent_weight;
  double q_quaternion_dependent_regularizer_fraction;

  std::vector<double> q_vector_pose_tracking;
  std::vector<double> r_vector_pose_tracking;
  std::vector<double> r_vector_position_tracking;


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

  std::vector<std::vector<double>> mu_list;
  double planning_dt_position_tracking;           // dt for planning when comparing samples. Used for position tracking.
  double planning_dt;           // dt for planning when comparing samples.
  std::vector<double> robot_radius_limits;
  int num_contacts_index;
  int num_contacts_index_for_cost;
  int num_contacts_index_for_curr_location;
  std::vector<int> num_contacts_list;
  std::vector<std::vector<int>> resolve_contacts_to_list;
  Eigen::MatrixXd Q_pose_tracking;
  Eigen::MatrixXd Q_position_tracking;
  Eigen::MatrixXd R_pose_tracking;
  Eigen::MatrixXd R_position_tracking;
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
    a->Visit(DRAKE_NVP(num_outer_threads));
    a->Visit(DRAKE_NVP(contact_model));
    a->Visit(DRAKE_NVP(use_predicted_x0_c3));
    a->Visit(DRAKE_NVP(use_predicted_x0_repos));
    a->Visit(DRAKE_NVP(at_least_predict_first_planned_trajectory_knot));
    a->Visit(DRAKE_NVP(use_predicted_x0_reset_mechanism));

    a->Visit(DRAKE_NVP(mu_list));
    a->Visit(DRAKE_NVP(planning_dt_position_tracking));
    a->Visit(DRAKE_NVP(planning_dt));
    a->Visit(DRAKE_NVP(robot_radius_limits));
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

    a->Visit(DRAKE_NVP(use_quaternion_dependent_cost));
    a->Visit(DRAKE_NVP(q_quaternion_dependent_weight));
    a->Visit(DRAKE_NVP(q_quaternion_dependent_regularizer_fraction));
    a->Visit(DRAKE_NVP(q_vector_pose_tracking));
    a->Visit(DRAKE_NVP(q_vector_position_tracking));
    a->Visit(DRAKE_NVP(r_vector_pose_tracking));
    a->Visit(DRAKE_NVP(r_vector_position_tracking));

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

    g_vector_pose_tracking = std::vector<double>();
    g_vector_pose_tracking.insert(g_vector_pose_tracking.end(), g_x_pose_tracking.begin(), g_x_pose_tracking.end());

    g_vector_pose_tracking_for_curr_location = std::vector<double>();
    g_vector_pose_tracking_for_curr_location.insert(g_vector_pose_tracking_for_curr_location.end(), g_x_pose_tracking.begin(), g_x_pose_tracking.end());

    g_vector_position_tracking = std::vector<double>();
    g_vector_position_tracking.insert(g_vector_position_tracking.end(),
                                      g_x_position_tracking.begin(), g_x_position_tracking.end());

    g_vector_position_tracking_for_curr_location = std::vector<double>();
    g_vector_position_tracking_for_curr_location.insert(g_vector_position_tracking_for_curr_location.end(),
                                      g_x_position_tracking.begin(), g_x_position_tracking.end());

    if (contact_model == "stewart_and_trinkle") {
      g_vector_pose_tracking.insert(g_vector_pose_tracking.end(), g_gamma_pose_tracking[num_contacts_index].begin(), g_gamma_pose_tracking[num_contacts_index].end());
      g_vector_pose_tracking.insert(g_vector_pose_tracking.end(), g_lambda_n_pose_tracking[num_contacts_index].begin(), g_lambda_n_pose_tracking[num_contacts_index].end());
      g_vector_pose_tracking.insert(g_vector_pose_tracking.end(), g_lambda_t_pose_tracking[num_contacts_index].begin(), g_lambda_t_pose_tracking[num_contacts_index].end());

      g_vector_pose_tracking_for_curr_location.insert(g_vector_pose_tracking_for_curr_location.end(), g_gamma_pose_tracking[num_contacts_index_for_curr_location].begin(), g_gamma_pose_tracking[num_contacts_index_for_curr_location].end());
      g_vector_pose_tracking_for_curr_location.insert(g_vector_pose_tracking_for_curr_location.end(), g_lambda_n_pose_tracking[num_contacts_index_for_curr_location].begin(), g_lambda_n_pose_tracking[num_contacts_index_for_curr_location].end());
      g_vector_pose_tracking_for_curr_location.insert(g_vector_pose_tracking_for_curr_location.end(), g_lambda_t_pose_tracking[num_contacts_index_for_curr_location].begin(), g_lambda_t_pose_tracking[num_contacts_index_for_curr_location].end());

      g_vector_position_tracking.insert(g_vector_position_tracking.end(), g_gamma_position_tracking[num_contacts_index].begin(), g_gamma_position_tracking[num_contacts_index].end());
      g_vector_position_tracking.insert(g_vector_position_tracking.end(), g_lambda_n_position_tracking[num_contacts_index].begin(), g_lambda_n_position_tracking[num_contacts_index].end());
      g_vector_position_tracking.insert(g_vector_position_tracking.end(), g_lambda_t_position_tracking[num_contacts_index].begin(), g_lambda_t_position_tracking[num_contacts_index].end());

      g_vector_position_tracking_for_curr_location.insert(g_vector_position_tracking_for_curr_location.end(), g_gamma_position_tracking[num_contacts_index_for_curr_location].begin(), g_gamma_position_tracking[num_contacts_index_for_curr_location].end());
      g_vector_position_tracking_for_curr_location.insert(g_vector_position_tracking_for_curr_location.end(), g_lambda_n_position_tracking[num_contacts_index_for_curr_location].begin(), g_lambda_n_position_tracking[num_contacts_index_for_curr_location].end());
      g_vector_position_tracking_for_curr_location.insert(g_vector_position_tracking_for_curr_location.end(), g_lambda_t_position_tracking[num_contacts_index_for_curr_location].begin(), g_lambda_t_position_tracking[num_contacts_index_for_curr_location].end());

    } else {
      g_vector_pose_tracking.insert(g_vector_pose_tracking.end(), g_lambda_pose_tracking[num_contacts_index].begin(), g_lambda_pose_tracking[num_contacts_index].end());
      g_vector_pose_tracking_for_curr_location.insert(g_vector_pose_tracking_for_curr_location.end(), g_lambda_pose_tracking[num_contacts_index_for_curr_location].begin(), g_lambda_pose_tracking[num_contacts_index_for_curr_location].end());
      g_vector_position_tracking.insert(g_vector_position_tracking.end(), g_lambda_position_tracking[num_contacts_index].begin(), g_lambda_position_tracking[num_contacts_index].end());
      g_vector_position_tracking_for_curr_location.insert(g_vector_position_tracking_for_curr_location.end(), g_lambda_position_tracking[num_contacts_index_for_curr_location].begin(), g_lambda_position_tracking[num_contacts_index_for_curr_location].end());
    }

    g_vector_pose_tracking.insert(g_vector_pose_tracking.end(), g_u_pose_tracking.begin(), g_u_pose_tracking.end());
    g_vector_pose_tracking_for_curr_location.insert(g_vector_pose_tracking_for_curr_location.end(), g_u_pose_tracking.begin(), g_u_pose_tracking.end());
    g_vector_position_tracking.insert(g_vector_position_tracking.end(), g_u_position_tracking.begin(), g_u_position_tracking.end());
    g_vector_position_tracking_for_curr_location.insert(g_vector_position_tracking_for_curr_location.end(), g_u_position_tracking.begin(), g_u_position_tracking.end());

    u_vector_pose_tracking = std::vector<double>();
    u_vector_pose_tracking.insert(u_vector_pose_tracking.end(), u_x_pose_tracking.begin(), u_x_pose_tracking.end());

    u_vector_pose_tracking_for_curr_location = std::vector<double>();
    u_vector_pose_tracking_for_curr_location.insert(u_vector_pose_tracking_for_curr_location.end(), u_x_pose_tracking.begin(), u_x_pose_tracking.end());

    u_vector_position_tracking = std::vector<double>();
    u_vector_position_tracking.insert(u_vector_position_tracking.end(), u_x_position_tracking.begin(), u_x_position_tracking.end());

    u_vector_position_tracking_for_curr_location = std::vector<double>();
    u_vector_position_tracking_for_curr_location.insert(u_vector_position_tracking_for_curr_location.end(), u_x_position_tracking.begin(), u_x_position_tracking.end());

    if (contact_model == "stewart_and_trinkle") {
      u_vector_pose_tracking.insert(u_vector_pose_tracking.end(), u_gamma_pose_tracking[num_contacts_index].begin(), u_gamma_pose_tracking[num_contacts_index].end());
      u_vector_pose_tracking.insert(u_vector_pose_tracking.end(), u_lambda_n_pose_tracking[num_contacts_index].begin(), u_lambda_n_pose_tracking[num_contacts_index].end());
      u_vector_pose_tracking.insert(u_vector_pose_tracking.end(), u_lambda_t_pose_tracking[num_contacts_index].begin(), u_lambda_t_pose_tracking[num_contacts_index].end());

      u_vector_pose_tracking_for_curr_location.insert(u_vector_pose_tracking_for_curr_location.end(), u_gamma_pose_tracking[num_contacts_index_for_curr_location].begin(), u_gamma_pose_tracking[num_contacts_index_for_curr_location].end());
      u_vector_pose_tracking_for_curr_location.insert(u_vector_pose_tracking_for_curr_location.end(), u_lambda_n_pose_tracking[num_contacts_index_for_curr_location].begin(), u_lambda_n_pose_tracking[num_contacts_index_for_curr_location].end());
      u_vector_pose_tracking_for_curr_location.insert(u_vector_pose_tracking_for_curr_location.end(), u_lambda_t_pose_tracking[num_contacts_index_for_curr_location].begin(), u_lambda_t_pose_tracking[num_contacts_index_for_curr_location].end());

      u_vector_position_tracking.insert(u_vector_position_tracking.end(), u_gamma_position_tracking[num_contacts_index].begin(), u_gamma_position_tracking[num_contacts_index].end());
      u_vector_position_tracking.insert(u_vector_position_tracking.end(), u_lambda_n_position_tracking[num_contacts_index].begin(), u_lambda_n_position_tracking[num_contacts_index].end());
      u_vector_position_tracking.insert(u_vector_position_tracking.end(), u_lambda_t_position_tracking[num_contacts_index].begin(), u_lambda_t_position_tracking[num_contacts_index].end());

      u_vector_position_tracking_for_curr_location.insert(u_vector_position_tracking_for_curr_location.end(), u_gamma_position_tracking[num_contacts_index_for_curr_location].begin(), u_gamma_position_tracking[num_contacts_index_for_curr_location].end());
      u_vector_position_tracking_for_curr_location.insert(u_vector_position_tracking_for_curr_location.end(), u_lambda_n_position_tracking[num_contacts_index_for_curr_location].begin(), u_lambda_n_position_tracking[num_contacts_index_for_curr_location].end());
      u_vector_position_tracking_for_curr_location.insert(u_vector_position_tracking_for_curr_location.end(), u_lambda_t_position_tracking[num_contacts_index_for_curr_location].begin(), u_lambda_t_position_tracking[num_contacts_index_for_curr_location].end());

    } else {
      u_vector_pose_tracking.insert(u_vector_pose_tracking.end(), u_lambda_pose_tracking[num_contacts_index].begin(), u_lambda_pose_tracking[num_contacts_index].end());
      u_vector_pose_tracking_for_curr_location.insert(u_vector_pose_tracking_for_curr_location.end(), u_lambda_pose_tracking[num_contacts_index_for_curr_location].begin(), u_lambda_pose_tracking[num_contacts_index_for_curr_location].end());
      u_vector_position_tracking.insert(u_vector_position_tracking.end(), u_lambda_position_tracking[num_contacts_index].begin(), u_lambda_position_tracking[num_contacts_index].end());
      u_vector_position_tracking_for_curr_location.insert(u_vector_position_tracking_for_curr_location.end(), u_lambda_position_tracking[num_contacts_index_for_curr_location].begin(), u_lambda_position_tracking[num_contacts_index_for_curr_location].end());
    }
    u_vector_pose_tracking.insert(u_vector_pose_tracking.end(), u_u_pose_tracking.begin(), u_u_pose_tracking.end());
    u_vector_pose_tracking_for_curr_location.insert(u_vector_pose_tracking_for_curr_location.end(), u_u_pose_tracking.begin(), u_u_pose_tracking.end());
    u_vector_position_tracking.insert(u_vector_position_tracking.end(), u_u_position_tracking.begin(), u_u_position_tracking.end());
    u_vector_position_tracking_for_curr_location.insert(u_vector_position_tracking_for_curr_location.end(), u_u_position_tracking.begin(), u_u_position_tracking.end());

    
    Eigen::VectorXd q_pose_tracking = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      this->q_vector_pose_tracking.data(), this->q_vector_pose_tracking.size());    
    Eigen::VectorXd q_position_tracking = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        this->q_vector_position_tracking.data(), this->q_vector_position_tracking.size());

    Eigen::VectorXd r_pose_tracking = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        this->r_vector_pose_tracking.data(), this->r_vector_pose_tracking.size());
    Eigen::VectorXd r_position_tracking = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        this->r_vector_position_tracking.data(), this->r_vector_position_tracking.size());

        Eigen::VectorXd g_pose_tracking = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      this->g_vector_pose_tracking.data(), this->g_vector_pose_tracking.size());
    Eigen::VectorXd g_pose_tracking_for_curr_location = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        this->g_vector_pose_tracking_for_curr_location.data(), this->g_vector_pose_tracking_for_curr_location.size());
    Eigen::VectorXd g_position_tracking = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        this->g_vector_position_tracking.data(), this->g_vector_position_tracking.size());
    Eigen::VectorXd g_position_tracking_for_curr_location = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        this->g_vector_position_tracking_for_curr_location.data(), this->g_vector_position_tracking_for_curr_location.size());

    Eigen::VectorXd u_pose_tracking = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        this->u_vector_pose_tracking.data(), this->u_vector_pose_tracking.size());
    Eigen::VectorXd u_pose_tracking_for_curr_location = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        this->u_vector_pose_tracking_for_curr_location.data(), this->u_vector_pose_tracking_for_curr_location.size());
    Eigen::VectorXd u_position_tracking = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        this->u_vector_position_tracking.data(), this->u_vector_position_tracking.size());
    Eigen::VectorXd u_position_tracking_for_curr_location = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        this->u_vector_position_tracking_for_curr_location.data(), this->u_vector_position_tracking_for_curr_location.size());

    Q_pose_tracking = w_Q_pose_tracking * q_pose_tracking.asDiagonal();
    // TODO: CHECK IF THE TUNING IS CORRECT. There used to be a bug here where Q_position used what was w_Q which was 
    // the pose tracking weight and not w_Q_position_tracking.
    Q_position_tracking = w_Q_position_tracking * q_position_tracking.asDiagonal();

    R_pose_tracking = w_R_pose_tracking * r_pose_tracking.asDiagonal();
    R_position_tracking = w_R_position_tracking * r_position_tracking.asDiagonal();

    G_pose_tracking = w_G_pose_tracking * g_pose_tracking.asDiagonal();
    G_pose_tracking_for_curr_location = w_G_pose_tracking * g_pose_tracking_for_curr_location.asDiagonal();
    G_position_tracking = w_G_position_tracking * g_position_tracking.asDiagonal();
    G_position_tracking_for_curr_location = w_G_position_tracking * g_position_tracking_for_curr_location.asDiagonal();
    
    U_pose_tracking = w_U_pose_tracking * u_pose_tracking.asDiagonal();
    U_pose_tracking_for_curr_location = w_U_pose_tracking * u_pose_tracking_for_curr_location.asDiagonal();
    U_position_tracking = w_U_position_tracking * u_position_tracking.asDiagonal();
    U_position_tracking_for_curr_location = w_U_position_tracking * u_position_tracking_for_curr_location.asDiagonal();
  }
};