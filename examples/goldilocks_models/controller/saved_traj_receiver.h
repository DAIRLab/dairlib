#pragma once

#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "dairlib/lcmt_trajectory_block.hpp"
#include "examples/goldilocks_models/reduced_order_models.h"
#include "examples/goldilocks_models/rom_walking_gains.h"
#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"
#include "multibody/view_frame.h"
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace goldilocks_models {

// One use case of ExponentialPlusPiecewisePolynomial: if we want to use a guard
// that merge optimal ROM traj with LIPM traj.

class SavedTrajReceiver : public drake::systems::LeafSystem<double> {
 public:
  SavedTrajReceiver(
      const ReducedOrderModel& rom,
      const drake::multibody::MultibodyPlant<double>& plant_feedback,
      const drake::multibody::MultibodyPlant<double>& plant_control,
      drake::systems::Context<double>* context_feedback,
      const std::vector<BodyPoint>& left_right_foot,
      std::vector<int> left_right_support_fsm_states,
      double single_support_duration, double double_support_duration,
      double desired_mid_foot_height, double desired_final_foot_height,
      const RomWalkingGains& gains,
      const StateMirror& state_mirror /*Only use for sim gap testing*/,
      const multibody::WorldYawViewFrame<double>& view_frame_feedback,
      const multibody::WorldYawViewFrame<double>& view_frame_control,
      bool wrt_com_in_local_frame, bool use_hybrid_rom_mpc);

  const drake::systems::InputPort<double>& get_input_port_lcm_traj() const {
    return this->get_input_port(saved_traj_lcm_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm() const {
    return this->get_input_port(fsm_port_);
  }

  // Desired trajs ports
  const drake::systems::OutputPort<double>& get_output_port_rom() const {
    return this->get_output_port(rom_traj_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_swing_foot() const {
    return this->get_output_port(swing_foot_traj_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_hip_rpy() const {
    return this->get_output_port(hip_rpy_traj_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_swing_hip() const {
    return this->get_output_port(swing_hip_yaw_traj_port_);
  }

  void AdjustPlannedTrajGivenGroundSlopeInput();
  const drake::systems::InputPort<double>& get_input_port_slope() const {
    return this->get_input_port(slope_port_);
  }

 private:
  void CalcRomTraj(const drake::systems::Context<double>& context,
                   drake::trajectories::Trajectory<double>* traj) const;
  void CalcSwingFootTraj(const drake::systems::Context<double>& context,
                         drake::trajectories::Trajectory<double>* traj) const;
  void CalcStanceHipTraj(const drake::systems::Context<double>& context,
                         drake::trajectories::Trajectory<double>* traj) const;
  void CalcSwingHipTraj(const drake::systems::Context<double>& context,
                        drake::trajectories::Trajectory<double>* traj) const;

  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  int saved_traj_lcm_port_;
  int state_port_;
  int fsm_port_;

  int rom_traj_port_;
  int swing_foot_traj_port_;
  int hip_rpy_traj_port_;
  int swing_hip_yaw_traj_port_;

  int liftoff_swing_foot_pos_idx_;
  int liftoff_stance_hip_pos_idx_;
  int liftoff_stance_hip_vel_idx_;
  int liftoff_swing_hip_pos_idx_;
  int liftoff_swing_hip_vel_idx_;

  mutable int prev_fsm_state_ = -1;
  mutable double lift_off_time_ = 0;

  int ny_;
  const drake::multibody::MultibodyPlant<double>& plant_feedback_;
  const drake::multibody::MultibodyPlant<double>& plant_control_;
  drake::systems::Context<double>* context_feedback_;
  std::unique_ptr<drake::systems::Context<double>> context_control_;
  const std::vector<BodyPoint>& left_right_foot_;
  std::vector<int> left_right_support_fsm_states_;

  int nq_;
  int nv_;
  int nx_;

  std::map<int, BodyPoint> swing_foot_map1_;
  std::map<int, int> stance_hip_roll_pos_map1_;
  std::map<int, int> stance_hip_pitch_pos_map1_;
  std::map<int, int> stance_hip_yaw_pos_map1_;
  std::map<int, int> swing_hip_yaw_pos_map1_;
  std::map<int, int> stance_hip_roll_vel_map1_;
  std::map<int, int> stance_hip_pitch_vel_map1_;
  std::map<int, int> stance_hip_yaw_vel_map1_;
  std::map<int, int> swing_hip_yaw_vel_map1_;
  std::map<bool, int> stance_hip_roll_pos_map2_;
  std::map<bool, int> stance_hip_pitch_pos_map2_;
  std::map<bool, int> stance_hip_yaw_pos_map2_;
  std::map<bool, int> swing_hip_yaw_pos_map2_;
  std::map<bool, int> stance_hip_roll_vel_map2_;
  std::map<bool, int> stance_hip_pitch_vel_map2_;
  std::map<bool, int> stance_hip_yaw_vel_map2_;
  std::map<bool, int> swing_hip_yaw_vel_map2_;

  // hacks
  double single_support_duration_;
  double double_support_duration_;

  // Not sure why sometimes in the beginning of single support phase, the 0
  // desired traj was read (it evaluates the traj in previous double support
  // phase). As a quick fix, I just shift the time a bit.
  // TODO: fix this
  double eps_hack_;

  // swing foot traj parameter
  double desired_mid_foot_height_;
  double desired_final_foot_height_;

  // [Test sim gap] -- use trajopt's traj directly in OSC
  drake::trajectories::PiecewisePolynomial<double> rom_pp_;
  int n_mode_;
  Eigen::MatrixXd x0_;
  Eigen::VectorXd x0_time_;
  Eigen::MatrixXd xf_;
  Eigen::VectorXd xf_time_;
  Eigen::VectorXd stance_foot_;

  //
  const multibody::WorldYawViewFrame<double>& view_frame_feedback_;
  const multibody::WorldYawViewFrame<double>& view_frame_control_;
  bool wrt_com_in_local_frame_;
  bool use_hybrid_rom_mpc_;

  // Heuristic
  double swing_foot_target_offset_x_;
  double final_foot_height_offset_for_right_leg_;

  // Testing slope
  bool use_slope_ = false;
  int slope_port_;
};

// We have IKTrajReceiver beside SavedTrajReceiver, because it also extracts the
// rows of the trajectory matrix that we want to track.
class IKTrajReceiver : public drake::systems::LeafSystem<double> {
 public:
  IKTrajReceiver(const drake::multibody::MultibodyPlant<double>& plant,
                 const std::vector<std::string>& ordered_pos_names);

 private:
  void CalcDesiredTraj(const drake::systems::Context<double>& context,
                       drake::trajectories::Trajectory<double>* traj) const;

  int saved_traj_lcm_port_;

  const drake::multibody::MultibodyPlant<double>& plant_;
  std::vector<int> ordered_indices_;

  int nq_;
};

}  // namespace goldilocks_models
}  // namespace dairlib
