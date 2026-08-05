// Standalone, deterministic repro of a single Reposition() call using the
// *exact* inputs captured from simlog-000004 at t=42.900s, the loop whose
// published knot 1 was corrupted (see
// /home/bibit/.claude/plans/i-am-seeing-some-precious-plum.md for the full
// evidence chain). Reposition() has no randomness and touches no shared
// controller state, so calling it directly here -- once built plain, once built
// with -fsanitize=address,undefined -- is a fast, deterministic way to try to
// catch a memory-safety/UB bug without needing to reproduce it inside a full,
// non-deterministic simulation run.

#include <iostream>

#include <Eigen/Dense>
#include <drake/common/yaml/yaml_io.h>

#include "examples/sampling_c3/parameter_headers/reposition_params.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_options.h"
#include "examples/sampling_c3/reposition.h"

using Eigen::Vector3d;
using Eigen::VectorXd;

int main() {
  const std::string kLogDir =
      "/home/bibit/3d_printer/logs/2026/08_05_26/000004/";
  SamplingC3RepositionParams reposition_params =
      drake::yaml::LoadYamlFile<SamplingC3RepositionParams>(
          kLogDir + "repos_params_000004.yaml");
  SamplingC3Options sampling_c3_options =
      drake::yaml::LoadYamlFile<SamplingC3Options>(
          kLogDir + "sampling_c3_params_000004.yaml");

  const int n_q = 10;
  const int n_x = 19;
  const int N = 10;
  const double dt = 0.075;
  const bool is_doing_c3 = false;
  bool finished_reposition_flag = false;

  // Exact x_lcs (full 19-dim state) from C3_ACTUAL at t=42.900s.
  VectorXd x_lcs(n_x);
  x_lcs << 0.16420584917068481, 0.11353716999292374, 0.12642791867256165,
      0.7033116221427917, -0.07237635552883148, -0.7033116221427917,
      -0.07237635552883148, 0.19793742895126343, 0.13655678927898407,
      9.993795538321137e-05, 0.0055417707189917564, 0.008091225288808346,
      -0.0875844731926918, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  // Exact repos_target (SAMPLE_LOCATIONS column 1 at the same loop).
  Vector3d repos_target(0.1707011793987627, 0.12302264233330215,
                        0.03365664289233885);

  Eigen::MatrixXd knots = dairlib::systems::Reposition(
      n_q, n_x, N, x_lcs, repos_target, dt, is_doing_c3,
      finished_reposition_flag, reposition_params, sampling_c3_options);

  std::cout << "finished_reposition_flag=" << finished_reposition_flag
            << "\n\n";
  std::cout << "knots (" << knots.rows() << " x " << knots.cols() << "):\n";
  for (int i = 0; i < knots.cols(); ++i) {
    std::cout << "  knot " << i << ": " << knots.col(i).transpose()
              << std::endl;
  }

  std::cout << "\nExpected (from manual re-derivation) knot 1 head(3): "
               "[0.1647, 0.1143, 0.1190]"
            << std::endl;
  std::cout << "Actually published (corrupted) knot 1: [0.002, 0.12, 0.0423]"
            << std::endl;

  return 0;
}
