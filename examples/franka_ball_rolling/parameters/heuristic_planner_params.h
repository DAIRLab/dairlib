#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

using Eigen::VectorXd;

struct HeuristicPlannerParams {

  std::string end_effector_simple_model;
  std::string ball_model;
  std::string ground_model;

  double roll_phase;
  double return_phase;
  VectorXd gait_parameters;
  int axis_option;
  double tilt_degrees;
  VectorXd q_new_vector;
  std::vector<double> g_new_list;
  std::vector<double> g_new_x;
  std::vector<double> g_new_gamma;
  std::vector<double> g_new_lambda_n;
  std::vector<double> g_new_lambda_t;
  std::vector<double> g_new_lambda;
  std::vector<double> g_new_u;
  VectorXd g_new_vector;

  VectorXd initial_start;
  VectorXd initial_finish;
  double stabilize_time;
  double move_time;

  template <typename Archive>
  void Serialize(Archive* a) {

    a->Visit(DRAKE_NVP(end_effector_simple_model));
    a->Visit(DRAKE_NVP(ball_model));
    a->Visit(DRAKE_NVP(ground_model));

    a->Visit(DRAKE_NVP(roll_phase));
    a->Visit(DRAKE_NVP(return_phase));
    a->Visit(DRAKE_NVP(gait_parameters));
    a->Visit(DRAKE_NVP(axis_option));
    a->Visit(DRAKE_NVP(tilt_degrees));

    a->Visit(DRAKE_NVP(q_new_vector));
    a->Visit(DRAKE_NVP(g_new_x));
    a->Visit(DRAKE_NVP(g_new_gamma));
    a->Visit(DRAKE_NVP(g_new_lambda_n));
    a->Visit(DRAKE_NVP(g_new_lambda_t));
    a->Visit(DRAKE_NVP(g_new_lambda));
    a->Visit(DRAKE_NVP(g_new_u));

    a->Visit(DRAKE_NVP(initial_start));
    a->Visit(DRAKE_NVP(initial_finish));
    a->Visit(DRAKE_NVP(stabilize_time));
    a->Visit(DRAKE_NVP(move_time));

    // TODO:: consider different contact model
    g_new_list = std::vector<double>();
    g_new_list.insert(g_new_list.end(), g_new_x.begin(), g_new_x.end());
    g_new_list.insert(g_new_list.end(), g_new_gamma.begin(), g_new_gamma.end());
    g_new_list.insert(g_new_list.end(), g_new_lambda_n.begin(), g_new_lambda_n.end());
    g_new_list.insert(g_new_list.end(), g_new_lambda_t.begin(), g_new_lambda_t.end());
    g_new_list.insert(g_new_list.end(), g_new_u.begin(), g_new_u.end());


    g_new_vector = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
              this->g_new_list.data(), this->g_new_list.size());
  }
};