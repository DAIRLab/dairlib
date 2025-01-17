#pragma once

#include "lcm/lcm_trajectory.h"

namespace dairlib::systems::controllers::id_mpc {


/*!
 * Wrapper class for the MPC solution containing trajectories and debug info
 */
class MPCSolution {
 public:
  LcmTrajectory solution_trajectories;
  std::vector<std::vector<std::string>> contact_sequence;
  std::vector<double> breaks;
};

}