#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

namespace dairlib::systems::controllers {

enum TrajectoryType{
  kZOH,
  kFOH,
  kCubicHermite,
  kCubicConstAccel,
  kCubicShapePreserving
};

class LcmTrajectoryReceiver : public drake::systems::LeafSystem<double> {
 public:
  LcmTrajectoryReceiver(std::string traj_name,
                        TrajectoryType traj_type);
 private:

  void MakeTrajFromLcm(const drake::systems::Context<double>& context,
                       drake::trajectories::Trajectory<double>* traj) const;

  const TrajectoryType traj_type_;
  const std::string traj_name_;
  std::map<std::string, drake::systems::OutputPortIndex> traj_output_ports_;
};

}
