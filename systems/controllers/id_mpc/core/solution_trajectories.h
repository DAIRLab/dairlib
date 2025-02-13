#pragma once

#include "lcm/lcm_trajectory.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

namespace dairlib::systems::controllers::id_mpc {

struct SolutionTraj {
  drake::trajectories::PiecewisePolynomial<double> q;
  drake::trajectories::PiecewisePolynomial<double> v;
  drake::trajectories::PiecewisePolynomial<double> u;
  drake::trajectories::PiecewisePolynomial<double> lambda;

  static SolutionTraj FromLcmTrajectory(
      const LcmTrajectory& traj,
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context);
};

}

