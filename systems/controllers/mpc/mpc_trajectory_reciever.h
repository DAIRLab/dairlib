#pragma once

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

#include "lcm/lcm_trajectory.h"

namespace dairlib {

enum TrajectoryType {
  kZeroOrderHold=0,
  kFirstorderHold=1,
  kCubicHermite,
  kCubicConstAccel,
  kCubicShapePreserving
};

class MpcTrajectoryReceiver : public drake::systems::LeafSystem<double> {
 public:
  MpcTrajectoryReceiver(TrajectoryType com_type, TrajectoryType angular_type, bool planar);

  const drake::systems::OutputPort<double>& get_com_traj_output_port() {
    return this->get_output_port(com_traj_port_);
  }
  const drake::systems::OutputPort<double>& get_angular_traj_output_port() {
    return this->get_output_port(angular_traj_port_);
  }
  const drake::systems::OutputPort<double>& get_swing_ft_target_output_port() {
    return this->get_output_port(swing_ft_traj_port_);
  }

 private:

  Eigen::MatrixXd Make3dFromPlanar(Eigen::MatrixXd planar_knots) const ;

  void MakeComTrajFromLcm(const drake::systems::Context<double>& context,
                          drake::trajectories::Trajectory<double>* traj) const;

  void MakeAngularTrajFromLcm(const drake::systems::Context<double>& context,
                              drake::trajectories::Trajectory<double>* traj) const;

  void MakeSwingFtTargetFromLcm(const drake::systems::Context<double>& context,
                                drake::systems::BasicVector<double>* target) const;

  const TrajectoryType com_type_;
  const TrajectoryType angular_type_;
  const bool planar_;
  const int kAngularDim_;

  int com_traj_port_;
  int angular_traj_port_;
  int swing_ft_traj_port_;
};

}
