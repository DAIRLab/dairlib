#pragma once

#include "drake/common/trajectories/piecewise_polynomial.h"

class MPCReference {
 public:
  drake::trajectories::PiecewisePolynomial<double> q_traj_;
  drake::trajectories::PiecewisePolynomial<double> v_traj_;
  drake::trajectories::PiecewisePolynomial<double> lambda_traj_;
  drake::trajectories::PiecewisePolynomial<double> u_traj_;
  std::vector<double> knot_times_;
  std::vector<std::vector<std::string>> active_contacts_;
};
