#pragma once

#include "yaml-cpp/yaml.h"

#include "drake/common/yaml/yaml_read_archive.h"

namespace dairlib {
namespace solvers {
struct OSQPSettingsYaml {
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
  int verbose;
  int scaled_termination;
  int check_termination;
  int warm_start;
  int scaling;
  int adaptive_rho;
  int adaptive_rho_interval;
  double adaptive_rho_tolerance;
  double adaptive_rho_fraction;

  template <typename Archive>
  void Serialize(Archive* a) {
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
    a->Visit(DRAKE_NVP(verbose));
    a->Visit(DRAKE_NVP(scaled_termination));
    a->Visit(DRAKE_NVP(check_termination));
    a->Visit(DRAKE_NVP(warm_start));
    a->Visit(DRAKE_NVP(scaling));
    a->Visit(DRAKE_NVP(adaptive_rho));
    a->Visit(DRAKE_NVP(adaptive_rho_interval));
    a->Visit(DRAKE_NVP(adaptive_rho_tolerance));
    a->Visit(DRAKE_NVP(adaptive_rho_fraction));
  }
};
}  // namespace solvers
}  // namespace dairlib