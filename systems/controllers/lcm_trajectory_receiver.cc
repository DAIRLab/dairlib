//
// Created by brian on 9/2/22.
//

#include "dairlib/lcmt_saved_traj.hpp"
#include "lcm_trajectory_receiver.h"

namespace dairlib::systems::controllers {

using std::string;

using dairlib::lcmt_saved_traj;

using drake::trajectories::PiecewisePolynomial;
using drake::systems::Context;
using drake::trajectories::Trajectory;


LcmTrajectoryReceiver::LcmTrajectoryReceiver(
    std::string traj_name, TrajectoryType traj_type)
    : traj_type_(traj_type), traj_name_(traj_name) {

  this->DeclareAbstractInputPort("lcmt_saved_traj", drake::Value<lcmt_saved_traj>{});

  // TODO: fix this
  Trajectory<double>* traj_inst = nullptr;
  this->DeclareAbstractOutputPort(
      traj_name, *traj_inst, &LcmTrajectoryReceiver::MakeTrajFromLcm);

}


}
}