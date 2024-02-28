#pragma once
#include <vector>
#include <cmath>
#include <assert.h>

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/path_parameterized_trajectory.h"

namespace dairlib::polynomials {

inline std::vector<double> get_chebyshev_points(int n) {
  assert(n >= 2);
  std::vector<double> cheb(n);
  for (int i = 0; i < n; i++) {
    double theta = M_PI *  (1.0 - static_cast<double>(i) / (n - 1.0));
    cheb[i] = 0.5 * (cos(theta) + 1);
  }
  // correct front and back in case of rounding error
  cheb.front() = 0.0;
  cheb.back() = 1.0;
  return cheb;
}

inline std::vector<double> get_even_points(int n) {
  assert(n >= 2);
  std::vector<double> even(n);
  for (int i = 0; i < n; i++) {
    even[i] = static_cast<double>(i) / static_cast<double>(n - 1);
  }
  // correct front and back in case of rounding error
  even.front() = 0.0;
  even.back() = 1.0;
  return even;
}


// Find a swing foot trajectory with continuous accelerations given by a 9th
// order polynomial, similarly to https://arxiv.org/pdf/1704.01271v1.pdf
// (equations 23-25), but with an additional cost on the mid-stride x/y
// position to accommodate stepping up/down
drake::trajectories::PathParameterizedTrajectory<double> AdaptSwingFootTraj(
    const drake::trajectories::PathParameterizedTrajectory<double>& prev_traj,
    double prev_time, double curr_time, double t_start, double t_end,
    double swing_foot_clearance, double z_vel_final, double z_pos_final_offset,
    const Eigen::Vector3d& footstep_target);

}