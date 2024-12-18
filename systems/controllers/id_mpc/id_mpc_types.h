#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <Eigen/Dense>

#include "multibody/kinematic/world_point_evaluator.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

namespace dairlib::systems::controllers::id_mpc {

struct IDMPCSolutionTrajectories {
  drake::trajectories::PiecewisePolynomial<double> q_;
  drake::trajectories::PiecewisePolynomial<double> v_;
  drake::trajectories::PiecewisePolynomial<double> u_;
  drake::trajectories::PiecewisePolynomial<double> lambda_h_;
  drake::trajectories::PiecewisePolynomial<double> lambda_c_;
};



}