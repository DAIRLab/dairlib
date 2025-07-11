#pragma once
#include <iostream>

#include "drake/common/yaml/yaml_read_archive.h"


/* Ways of computing C3 costs after solving the MPC problem:
  0. kSimLCS:                       Simulate the LCS dynamics from the planned
                                    inputs.
  1. kUseC3Plan:                    Use the C3 planned trajectory and inputs.
  2. kSimLCSReplaceC3EEPlan:        Simulate the LCS dynamics from the planned
                                    inputs only for the object; use the planned
                                    EE trajectory.
  3. kSimImpedance:                 Try to emulate the real cost of the system
                                    associated not only applying the planned
                                    inputs, but also tracking the planned EE
                                    trajectory with an impedance controller.
  4. kSimImpedanceReplaceC3EEPlan:  The same as kSimImpedance except the EE
                                    states are replaced with the plan from C3 at
                                    the end.
  5. kSimImpedanceObjectCostOnly:   The same as kSimImpedance except only the
                                    object terms contribute to the final cost.
*/
enum C3CostComputationType {
  kSimLCS,
  kUseC3Plan,
  kSimLCSReplaceC3EEPlan,
  kSimImpedance,
  kSimImpedanceReplaceC3EEPlan,
  kSimImpedanceObjectCostOnly,
};

struct C3Options {
  // Hyperparameters
  int admm_iter;     // total number of ADMM iterations
  float rho;         // initial value of the rho parameter
  float rho_scale;   // scaling of rho parameter (/rho = rho_scale * /rho)
  int num_threads;   // 0 is dynamic, greater than 0 for a fixed count
  int delta_option;  // different options for delta update
  std::string projection_type;
  std::string contact_model;
  bool warm_start;
  bool use_predicted_x0;
  bool end_on_qp_step;
  bool use_robust_formulation;
  double solve_time_filter_alpha;
  double publish_frequency;

  std::vector<double> u_horizontal_limits;
  std::vector<double> u_vertical_limits;
  std::vector<Eigen::VectorXd> workspace_limits;
  double workspace_margins;

  int N;
  double gamma;

  double w_Q;
  double w_R;
  double w_G;
  double w_U;

  std::vector<double> q_vector;
  std::vector<double> r_vector;

  std::vector<double> g_vector;
  std::vector<double> g_x;
  std::vector<double> g_gamma;
  std::vector<double> g_lambda_n;
  std::vector<double> g_lambda_t;
  std::vector<double> g_lambda;
  std::vector<double> g_eta_slack;
  std::vector<double> g_eta_n;
  std::vector<double> g_eta_t;
  std::vector<double> g_eta;
  std::vector<double> g_u;

  std::vector<double> u_vector;
  std::vector<double> u_x;
  std::vector<double> u_gamma;
  std::vector<double> u_lambda_n;
  std::vector<double> u_lambda_t;
  std::vector<double> u_lambda;
  std::vector<double> u_eta_slack;
  std::vector<double> u_eta_n;
  std::vector<double> u_eta_t;
  std::vector<double> u_eta;
  std::vector<double> u_u;

  double qp_projection_alpha;
  double qp_projection_scaling;
  bool penalize_changes_in_u_across_solves;
  std::vector<double> mu;
  double dt;
  double solve_dt;    // unused
  int num_friction_directions;
  int num_contacts;
  Eigen::MatrixXd Q;
  Eigen::MatrixXd R;
  Eigen::MatrixXd G;
  Eigen::MatrixXd U;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(admm_iter));
    a->Visit(DRAKE_NVP(rho));
    a->Visit(DRAKE_NVP(rho_scale));
    a->Visit(DRAKE_NVP(num_threads));
    a->Visit(DRAKE_NVP(delta_option));
    a->Visit(DRAKE_NVP(contact_model));
    a->Visit(DRAKE_NVP(projection_type));
    if (projection_type == "QP") {
      DRAKE_DEMAND(contact_model == "anitescu");
    }
    a->Visit(DRAKE_NVP(warm_start));
    a->Visit(DRAKE_NVP(use_predicted_x0));
    a->Visit(DRAKE_NVP(end_on_qp_step));
    a->Visit(DRAKE_NVP(use_robust_formulation));
    a->Visit(DRAKE_NVP(solve_time_filter_alpha));
    a->Visit(DRAKE_NVP(publish_frequency));

    a->Visit(DRAKE_NVP(workspace_limits));
    a->Visit(DRAKE_NVP(u_horizontal_limits));
    a->Visit(DRAKE_NVP(u_vertical_limits));
    a->Visit(DRAKE_NVP(workspace_margins));

    a->Visit(DRAKE_NVP(mu));
    a->Visit(DRAKE_NVP(dt));
    a->Visit(DRAKE_NVP(solve_dt));
    a->Visit(DRAKE_NVP(num_friction_directions));
    a->Visit(DRAKE_NVP(num_contacts));

    a->Visit(DRAKE_NVP(N));
    a->Visit(DRAKE_NVP(gamma));
    a->Visit(DRAKE_NVP(w_Q));
    a->Visit(DRAKE_NVP(w_R));
    a->Visit(DRAKE_NVP(w_G));
    a->Visit(DRAKE_NVP(w_U));
    a->Visit(DRAKE_NVP(q_vector));
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

    // Additional parameters for new C3 projection
    a->Visit(DRAKE_NVP(g_eta_slack));
    a->Visit(DRAKE_NVP(g_eta_n));
    a->Visit(DRAKE_NVP(g_eta_t));
    a->Visit(DRAKE_NVP(g_eta));
    a->Visit(DRAKE_NVP(u_eta_slack));
    a->Visit(DRAKE_NVP(u_eta_n));
    a->Visit(DRAKE_NVP(u_eta_t));
    a->Visit(DRAKE_NVP(u_eta));

    a->Visit(DRAKE_NVP(qp_projection_alpha));
    a->Visit(DRAKE_NVP(qp_projection_scaling));
    a->Visit(DRAKE_NVP(penalize_changes_in_u_across_solves));

    g_vector = std::vector<double>();
    g_vector.insert(g_vector.end(), g_x.begin(), g_x.end());
    if (contact_model == "stewart_and_trinkle") {
      g_vector.insert(g_vector.end(), g_gamma.begin(), g_gamma.end());
      g_vector.insert(g_vector.end(), g_lambda_n.begin(), g_lambda_n.end());
      g_vector.insert(g_vector.end(), g_lambda_t.begin(), g_lambda_t.end());
    } else {
      g_vector.insert(g_vector.end(), g_lambda.begin(), g_lambda.end());
    }
    g_vector.insert(g_vector.end(), g_u.begin(), g_u.end());
    if (projection_type == "NEXTGEN") {
      if (contact_model == "stewart_and_trinkle") {
        g_vector.insert(g_vector.end(), g_eta_slack.begin(), g_eta_slack.end());
        g_vector.insert(g_vector.end(), g_eta_n.begin(), g_eta_n.end());
        g_vector.insert(g_vector.end(), g_eta_t.begin(), g_eta_t.end());
      } else {
        g_vector.insert(g_vector.end(), g_eta.begin(), g_eta.end());
      }
    }

    u_vector = std::vector<double>();
    u_vector.insert(u_vector.end(), u_x.begin(), u_x.end());
    if (contact_model == "stewart_and_trinkle") {
      u_vector.insert(u_vector.end(), u_gamma.begin(), u_gamma.end());
      u_vector.insert(u_vector.end(), u_lambda_n.begin(), u_lambda_n.end());
      u_vector.insert(u_vector.end(), u_lambda_t.begin(), u_lambda_t.end());
    } else {
      u_vector.insert(u_vector.end(), u_lambda.begin(), u_lambda.end());
    }
    u_vector.insert(u_vector.end(), u_u.begin(), u_u.end());
    if (projection_type == "NEXTGEN") {
      if (contact_model == "stewart_and_trinkle") {
        u_vector.insert(u_vector.end(), u_eta_slack.begin(), u_eta_slack.end());
        u_vector.insert(u_vector.end(), u_eta_n.begin(), u_eta_n.end());
        u_vector.insert(u_vector.end(), u_eta_t.begin(), u_eta_t.end());
      } else {
        u_vector.insert(u_vector.end(), u_eta.begin(), u_eta.end());
      }
    }

    Eigen::VectorXd q = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        this->q_vector.data(), this->q_vector.size());
    Eigen::VectorXd r = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        this->r_vector.data(), this->r_vector.size());
    Eigen::VectorXd g = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        this->g_vector.data(), this->g_vector.size());
    Eigen::VectorXd u = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        this->u_vector.data(), this->u_vector.size());

    DRAKE_DEMAND(g_lambda.size() == num_contacts * num_friction_directions*2);
    DRAKE_DEMAND(u_lambda.size() == num_contacts * num_friction_directions*2);
    DRAKE_DEMAND(mu.size() == num_contacts);
    DRAKE_DEMAND(g.size() == u.size());

    Q = w_Q * q.asDiagonal();
    R = w_R * r.asDiagonal();
    G = w_G * g.asDiagonal();
    U = w_U * u.asDiagonal();
  }
};