#pragma once

#include "solvers/sqp/sqp_solver.h"
#include "lcm/lcm_trajectory.h"

namespace dairlib::systems::controllers::id_mpc {


/*!
 * Wrapper class for the MPC solution containing trajectories and debug info
 */
class MPCSolution {
 public:
  solvers::sqp::SQPIterate sqp_iterate;
  LcmTrajectory solution_trajectories;
  std::vector<std::vector<std::string>> contact_sequence;
  std::vector<double> breaks;
  bool is_initial_solve = true;
};

}