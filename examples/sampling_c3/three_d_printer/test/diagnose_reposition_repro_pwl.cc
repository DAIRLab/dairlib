// Standalone, deterministic repro of two Reposition() calls using the exact
// inputs captured from 8/5/2026 simlog-000005 -- recorded after the
// RepositionStraightLine fix (see diagnose_reposition_repro.cc / the plan doc)
// but still showing abrupt EE jumps. One captured loop (t=72.018s) is in the
// straight-line gating window; the other (t=72.108s) is in the
// RepositionPiecewiseLinear window, which was not touched by the earlier fix.
// Both showed a corrupted knot with x collapsed to 0.002 (the clamped lower
// workspace-x bound) in the real log. Build plain first (sanity check), then
// with -fsanitize=address,undefined (scoped via --per_file_copt to
// examples/sampling_c3/reposition.cc and this file, since a whole-tree
// sanitized build hits a pre-existing Drake/abseil incompatibility).

#include <iostream>

#include <Eigen/Dense>
#include <drake/common/yaml/yaml_io.h>

#include "examples/sampling_c3/parameter_headers/reposition_params.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_options.h"
#include "examples/sampling_c3/reposition.h"

using Eigen::Vector3d;
using Eigen::VectorXd;

namespace {

void RunScenario(const std::string& label, const VectorXd& x_lcs,
                 const Vector3d& repos_target,
                 const SamplingC3RepositionParams& reposition_params,
                 const SamplingC3Options& sampling_c3_options) {
  const int n_q = 10;
  const int n_x = 19;
  const int N = 10;
  const double dt = 0.075;
  const bool is_doing_c3 = false;
  bool finished_reposition_flag = false;

  std::cout << "=== " << label << " ===" << std::endl;
  Eigen::MatrixXd knots = dairlib::systems::Reposition(
      n_q, n_x, N, x_lcs, repos_target, dt, is_doing_c3,
      finished_reposition_flag, reposition_params, sampling_c3_options);
  for (int i = 0; i < knots.cols(); ++i) {
    std::cout << "  knot " << i << ": " << knots.col(i).head(3).transpose()
              << std::endl;
  }
  std::cout << std::endl;
}

}  // namespace

int main() {
  const std::string kLogDir =
      "/home/bibit/3d_printer/logs/2026/08_05_26/000005/";
  SamplingC3RepositionParams reposition_params =
      drake::yaml::LoadYamlFile<SamplingC3RepositionParams>(
          kLogDir + "repos_params_000005.yaml");
  SamplingC3Options sampling_c3_options =
      drake::yaml::LoadYamlFile<SamplingC3Options>(
          kLogDir + "sampling_c3_params_000005.yaml");

  // t=72.018s: xy_dist ~0.0125m -- straight-line gating window.
  VectorXd x_lcs_straight(19);
  x_lcs_straight << 0.16857494413852692, 0.10781942307949066,
      0.1191120445728302, -0.18182548880577087, -0.00947947520762682,
      -0.9911580681800842, -0.00019501829228829592, 0.16634902358055115,
      0.13004566729068756, 0.026536354795098305, 0.012634863145649433,
      -0.004866830073297024, -0.08729946613311768, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  Vector3d repos_target_straight(0.1802286992187482, 0.10333015934431672,
                                 0.04224230279172952);
  RunScenario("t=72.018s (straight-line branch, published knot 0 corrupted)",
              x_lcs_straight, repos_target_straight, reposition_params,
              sampling_c3_options);

  // t=72.108s: xy_dist ~0.037m -- RepositionPiecewiseLinear window.
  VectorXd x_lcs_pwl(19);
  x_lcs_pwl << 0.21756020188331604, 0.10374830663204193, 0.13307036459445953,
      -0.1818227618932724, -0.009464219212532043, -0.9911590218544006,
      -0.00019870222604367882, 0.16635248064994812, 0.13004589080810547,
      0.026536155492067337, 1.2996201515197754, -0.1028672307729721,
      0.4945715665817261, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  Vector3d repos_target_pwl(0.1802286992187482, 0.10333015934431672,
                            0.04224230279172952);
  RunScenario("t=72.108s (RepositionPiecewiseLinear branch, knot 1 corrupted)",
              x_lcs_pwl, repos_target_pwl, reposition_params,
              sampling_c3_options);

  return 0;
}
