#pragma once

#include "multibody/multibody_utils.h"
#include "systems/controllers/control_utils.h"
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

/// AlipSwingFootTrajGenerator generates a desired 3D trajectory of swing foot.
/// The trajectory is a cubic spline (two segments of cubic polynomials).
///
/// In the x-y plane, the start point of the traj is the swing foot position
/// at lift-off, and the end point is determined by solving for the appropriate
/// footstep location to reach the desired angular momentum about the contact
/// point at the end of the next stance period
/// (see https://doi.org/10.1109/ICRA48506.2021.9560821)
///
/// In the z direction, the start point is the
/// swing foot position it leaves the ground, and the mid point and end
/// point are both specified by the user.
///
/// Arguments of the constructor:
/// - MultibodyPlant of the robot
/// - left/right stance state of finite state machine
/// - duration of the left/right stance state of finite state machine
/// - left/right position (w.r.t. left/right foot body) and body frame of the
///     contact point
/// - desired height of the swing foot during mid swing phase
/// - desired height of the swing foot at the end of swing phase
/// - desired vertical velocity of the swing foot at the end of swing phase
/// - maximum distance between center of mass and x_fs
///     (used to restrict the footstep within an area)
/// - footstep offset (desired foot spread during steady state walking)
/// - center line offset
///      (minimum distance from the robot x-axis to each footstep xy location)
/// - wrt_com_in_local_frame
///      (whether to express the foot location relative to
///                                             the CoM in the pelvis yaw frame)

class AlipSwingFootTrajGenerator : public drake::systems::LeafSystem<double> {
 public:
  AlipSwingFootTrajGenerator(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context,
      std::vector<int> left_right_support_fsm_states,
      std::vector<double> left_right_support_durations,
      std::vector<std::pair<const Eigen::Vector3d,
                            const drake::multibody::Frame<double>&>>
      left_right_foot,
      std::string floating_base_body_name, double double_support_duration,
      double mid_foot_height, double desired_final_foot_height,
      double desired_final_vertical_foot_velocity,
      double max_com_to_x_footstep_dist, double footstep_offset,
      double center_line_offset);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm() const {
    return this->get_input_port(fsm_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm_switch_time()
  const {
    return this->get_input_port(liftoff_time_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_alip_state() const {
    return this->get_input_port(alip_state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_vdes() const {
    return this->get_input_port(vdes_port_);
  }

 private:
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void CalcFootStepAndStanceFootHeight(
      const drake::systems::Context<double>& context,
      const OutputVector<double>* robot_output,
      const double end_time_of_this_interval, Eigen::Vector2d* x_fs,
      double* stance_foot_height) const;

  drake::trajectories::PiecewisePolynomial<double> CreateSplineForSwingFoot(
      const double start_time_of_this_interval,
      const double end_time_of_this_interval, const double stance_duration,
      const Eigen::Vector3d& init_swing_foot_pos, const Eigen::Vector2d& x_fs,
      double stance_foot_height) const;

  void CalcTrajs(const drake::systems::Context<double>& context,
                 drake::trajectories::Trajectory<double>* traj) const;

  int state_port_;
  int fsm_port_;
  int liftoff_time_port_;
  int alip_state_port_;
  int vdes_port_;

  int liftoff_swing_foot_pos_idx_;
  int prev_fsm_state_idx_;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  const drake::multibody::BodyFrame<double>& world_;

  std::vector<int> left_right_support_fsm_states_;

  double double_support_duration_;

  // Parameters
  double m_;
  double mid_foot_height_;
  double desired_final_foot_height_;
  double desired_final_vertical_foot_velocity_;
  double max_com_to_footstep_dist_;
  const double footstep_offset_;     // in meters
  const double center_line_offset_;  // in meters

  // Maps
  std::map<int, std::pair<const Eigen::Vector3d,
                          const drake::multibody::Frame<double>&>>
      stance_foot_map_;
  std::map<int, std::pair<const Eigen::Vector3d,
                          const drake::multibody::Frame<double>&>>
      swing_foot_map_;
  std::map<int, double> duration_map_;
};

}  // namespace systems
}  // namespace dairlib
