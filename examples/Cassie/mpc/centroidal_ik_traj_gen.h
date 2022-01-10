#pragma once

#include "multibody/single_rigid_body_plant.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/math/autodiff.h"

namespace dairlib::mpc {
class CentroidalIKTrajGen : public drake::systems::LeafSystem<double> {
 public:
  CentroidalIKTrajGen(
      const drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
      const drake::multibody::MultibodyPlant<double>& plant,
      const Eigen::Matrix3d& I, double mass, double dt, double stance_duration);

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
  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_pelvis_orientation_traj() const {
    return this->get_output_port(pelvis_traj_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_swing_foot_traj() const {
    return this->get_output_port(swing_foot_traj_port_);
  }
 private:

  void CalcTraj(const drake::systems::Context<double> &context,
                drake::systems::BasicVector<double> *solvec) const;

  void AssignPelvisTraj(const drake::systems::Context<double> & context,
                        drake::trajectories::Trajectory<double> *output_traj);

  void AssignSwingFootTraj(const drake::systems::Context<double> & context,
                        drake::trajectories::Trajectory<double> *output_traj);

  const Eigen::Matrix3d& I_b_;
  const double mass_;
  const std::vector<std::string> toe_frames_ = {"toe_left", "toe_right"};
  const Eigen::Vector3d toe_mid_ = Eigen::Vector3d(0.023715, 0.056, 0);
  const drake::Vector3<drake::AutoDiffXd> toe_mid_ad_ =
      drake::math::InitializeAutoDiff(toe_mid_);

  const drake::systems::CacheEntry* ik_sol_cache_entry_;

  int state_port_;
  drake::TypeSafeIndex<drake::systems::InputPortTag> mpc_traj_port_;
  int fsm_port_;
  int touchdown_time_port_;
  int swing_foot_traj_port_;
  int pelvis_traj_port_;

  mutable double mpc_timestamp_ = -1.0;
  mutable Eigen::VectorXd prev_ik_sol_;

  double dt_;
  double stance_duration_;
  const drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad_;
  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<drake::AutoDiffXd>* context_ad_;
  drake::systems::Context<double>* context_;

};
}
