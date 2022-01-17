#pragma once

#include "multibody/single_rigid_body_plant.h"

#include "drake/systems/framework/leaf_system.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/multibody/plant/multibody_plant.h"


namespace dairlib::mpc{
class AngularMomentumTrajGen :  public drake::systems::LeafSystem<double> {
 public:
  AngularMomentumTrajGen(const Eigen::Matrix3d& I);

 private:

  void CalcTraj(const drake::systems::Context<double> &context,
      drake::trajectories::Trajectory<double> *traj) const;

  const Eigen::Matrix3d& I_b_;
  int traj_out_port_;
  int mpc_traj_in_port_;

};
}
