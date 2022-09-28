#pragma once
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/solver_options.h"

namespace dairlib::solvers{

/// Convenience struct for parsing solver options
struct DairOsqpSolverOptions {
  drake::solvers::SolverOptions osqp_options;
  int verbose;
  double rho;
  double sigma;
  int max_iter;
  double eps_abs;
  double eps_rel;
  double eps_prim_inf;
  double eps_dual_inf;
  double alpha;
  int linsys_solver;
  double delta;
  int polish;
  int polish_refine_iter;
  int scaled_termination;
  int check_termination;
  int warm_start;
  int scaling;
  int adaptive_rho;
  double adaptive_rho_interval;
  double adaptive_rho_tolerance;
  double adaptive_rho_fraction;
  double time_limit;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(verbose));
    a->Visit(DRAKE_NVP(rho));
    a->Visit(DRAKE_NVP(sigma));
    a->Visit(DRAKE_NVP(max_iter));
    a->Visit(DRAKE_NVP(eps_abs));
    a->Visit(DRAKE_NVP(eps_rel));
    a->Visit(DRAKE_NVP(eps_prim_inf));
    a->Visit(DRAKE_NVP(eps_dual_inf));
    a->Visit(DRAKE_NVP(alpha));
    a->Visit(DRAKE_NVP(linsys_solver));
    a->Visit(DRAKE_NVP(delta));
    a->Visit(DRAKE_NVP(polish));
    a->Visit(DRAKE_NVP(polish_refine_iter));
    a->Visit(DRAKE_NVP(scaled_termination));
    a->Visit(DRAKE_NVP(check_termination));
    a->Visit(DRAKE_NVP(warm_start));
    a->Visit(DRAKE_NVP(scaling));
    a->Visit(DRAKE_NVP(adaptive_rho));
    a->Visit(DRAKE_NVP(adaptive_rho_interval));
    a->Visit(DRAKE_NVP(adaptive_rho_tolerance));
    a->Visit(DRAKE_NVP(adaptive_rho_fraction));
    a->Visit(DRAKE_NVP(time_limit));

    osqp_options.SetOption(drake::solvers::OsqpSolver::id(), "verbose", verbose); //NOLINT
    osqp_options.SetOption(drake::solvers::OsqpSolver::id(), "rho",  rho); // NOLINT
    osqp_options.SetOption(drake::solvers::OsqpSolver::id(), "sigma",  sigma); // NOLINT
    osqp_options.SetOption(drake::solvers::OsqpSolver::id(), "max_iter",  max_iter); // NOLINT
    osqp_options.SetOption(drake::solvers::OsqpSolver::id(), "eps_abs",  eps_abs); // NOLINT
    osqp_options.SetOption(drake::solvers::OsqpSolver::id(), "eps_rel",  eps_rel); // NOLINT
    osqp_options.SetOption(drake::solvers::OsqpSolver::id(), "eps_prim_inf",  eps_prim_inf); // NOLINT
    osqp_options.SetOption(drake::solvers::OsqpSolver::id(), "eps_dual_inf",  eps_dual_inf); // NOLINT
    osqp_options.SetOption(drake::solvers::OsqpSolver::id(), "alpha",  alpha); // NOLINT
    osqp_options.SetOption(drake::solvers::OsqpSolver::id(), "linsys_solver", linsys_solver); //NOLINT
    osqp_options.SetOption(drake::solvers::OsqpSolver::id(), "delta",  delta); // NOLINT
    osqp_options.SetOption(drake::solvers::OsqpSolver::id(), "polish",  polish); // NOLINT
    osqp_options.SetOption(drake::solvers::OsqpSolver::id(), "polish_refine_iter",  polish_refine_iter); // NOLINT
    osqp_options.SetOption(drake::solvers::OsqpSolver::id(), "scaled_termination",  scaled_termination); // NOLINT
    osqp_options.SetOption(drake::solvers::OsqpSolver::id(), "check_termination",  check_termination); // NOLINT
    osqp_options.SetOption(drake::solvers::OsqpSolver::id(), "warm_start",  warm_start); // NOLINT
    osqp_options.SetOption(drake::solvers::OsqpSolver::id(), "scaling",  scaling); // NOLINT
    osqp_options.SetOption(drake::solvers::OsqpSolver::id(), "adaptive_rho",  adaptive_rho); // NOLINT
    osqp_options.SetOption(drake::solvers::OsqpSolver::id(), "adaptive_rho_interval",  adaptive_rho_interval); // NOLINT
    osqp_options.SetOption(drake::solvers::OsqpSolver::id(), "adaptive_rho_tolerance",  adaptive_rho_tolerance); // NOLINT
    osqp_options.SetOption(drake::solvers::OsqpSolver::id(), "adaptive_rho_fraction",  adaptive_rho_fraction); // NOLINT
    osqp_options.SetOption(drake::solvers::OsqpSolver::id(), "time_limit",  time_limit); // NOLINT
  }
};
}
