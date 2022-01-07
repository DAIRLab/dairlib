#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/math/autodiff.h"

namespace dairlib::mpc {
class CentroidalIKTrajGen : public drake::systems::LeafSystem<double> {
 public:
  CentroidalIKTrajGen(
      const drake::multibody::MultibodyPlant<drake::AutoDiffXd> plant);

  // Input port getters
  const drake::systems::InputPort<double> &get_input_port_fsm() const {
    return this->get_input_port(fsm_port_);
  }
  const drake::systems::InputPort<double> &get_input_port_touchdown_time() const{
    return this->get_input_port(touchdown_time_port_);
  }
  const drake::systems::InputPort<double> &get_input_port_mpc_traj() const {
    return this->get_input_port(mpc_traj_port_);
  }


 private:

  void CalcTraj(const drake::systems::Context<double> &context,
                drake::trajectories::Trajectory<double> *traj) const;

  const Eigen::Matrix3d& I_b_;


  int state_port_;
  int mpc_traj_port_;
  int fsm_port_;
  int touchdown_time_port_;
  double swing_foot_traj_port_;
  double pelvis_traj_port_;
  double dt_;

  const drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_;
  const drake::systems::Context<drake::AutoDiffXd>* context_;

};
}
