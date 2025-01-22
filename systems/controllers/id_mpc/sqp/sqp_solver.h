#pragma once

#include "solvers/qpalm_solver.h"

namespace dairlib::systems::controllers::id_mpc {

struct SQPIterate {
  Eigen::VectorXd x_init;
  Eigen::VectorXd dx;
  Eigen::VectorXd x_sol;
  bool accepted;
  double constraint_viol;
  double cost;
};

struct LineSearchParams {
  double alpha_min = 1e-4;
  double theta_max = 1e-2;
  double theta_min = 1e-6;
  double eta = 1e-4;
  double gamma_phi = 1e-6;
  double gamma_theta = 1e-6;
  double gamma_alpha = 0.5;
};

class SQPSolver {
 public:
  SQPSolver(int n, int m,
            std::function<void (const Eigen::VectorXd&, solvers::QPData&)>
                make_qp,
            std::function<double (const Eigen::VectorXd&)> eval_constraint_viol,
            std::function<double (const Eigen::VectorXd&)> eval_cost,
            std::function<void (Eigen::VectorXd&)> proj_to_cspace);

  SQPIterate AllocateIterate() const;

  void DoSQPStep(const Eigen::VectorXd& x, SQPIterate& sol);

 private:

  /*!
   * line search assuming x_init and dx have already been properly set
   * @param sol SQPIterate to perform line search for
   */
  void LineSearch(SQPIterate& sol);

  std::function<void (const Eigen::VectorXd&, solvers::QPData&)> make_qp_;
  std::function<double (const Eigen::VectorXd&)> eval_constraint_viol_;
  std::function<double (const Eigen::VectorXd&)> eval_cost_;
  std::function<void (Eigen::VectorXd&)> proj_to_config_space_;

  solvers::QPALMSolver qp_solver_;
  solvers::QPData qp_;

  LineSearchParams lsparams_;
};

}