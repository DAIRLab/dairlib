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

/// SwingFootTrajGenerator generates a desired 3D trajectory of swing foot.
/// The trajectory is a cubic spline (two segments of cubic polynomials).
/// In the x-y plane, the start point of the traj is the swing foot position
/// at lift-off, and the end point is either the capture point (CP) or the
/// neutral point derived from LIPM. In the z direction, the start point is the
/// swing foot position before it leaves the ground, and the mid point and end
/// point are both specified by the user. The footstep location, x_fs, can be
/// modified with two flags
///  - `add_speed_regularization`
///  - `is_feet_collision_avoid`
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
/// - footstep offset (to avoid foot collision)
/// - center line offset (used to restrict the footstep within an area)
/// - a flag enabling footstep modification (e.g. walking speed regularization)
/// - a flag enabling feet collision avoidance
/// - a flag enabling the usage of prediction of center of mass
///     (use predicted center of mass position at touchdown to calculate x_fs)
/// - an integer indicates which foot step algorithm to use:
///     0 is the capture point
///     1 is the neutral point derived from LIPM given the stance duration

class SwingFootTrajGenerator : public drake::systems::LeafSystem<double> {
 public:
  // TODO(yminchen): clean up the parameters. Maybe we should extract the
  //  collision avoidance into a new leafsystem?
  SwingFootTrajGenerator(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context,
      std::vector<int> left_right_support_fsm_states,
      std::vector<double> left_right_support_durations,
      std::vector<std::pair<const Eigen::Vector3d,
                            const drake::multibody::Frame<double>&>>
          left_right_foot,
      std::string floating_base_body_name, double mid_foot_height,
      double desired_final_foot_height,
      double desired_final_vertical_foot_velocity,
      double max_com_to_x_footstep_dist, double footstep_offset,
      double center_line_offset, bool add_speed_regularization,
      bool is_feet_collision_avoid, bool is_using_predicted_com,
      int footstep_option = 0);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm() const {
    return this->get_input_port(fsm_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm_switch_time()
      const {
    return this->get_input_port(fsm_switch_time_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_com() const {
    return this->get_input_port(com_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_sc() const {
    return this->get_input_port(speed_control_port_);
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
  int fsm_switch_time_port_;
  int com_port_;
  int speed_control_port_;

  int prev_liftoff_swing_foot_idx_;
  int prev_fsm_state_idx_;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  const drake::multibody::BodyFrame<double>& world_;
  const drake::multibody::Body<double>& pelvis_;

  std::vector<int> left_right_support_fsm_states_;

  // Parameters
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

  // options
  bool add_speed_regularization_;
  bool is_feet_collision_avoid_;
  bool is_using_predicted_com_;
  int footstep_option_;

  // COM vel filtering
  // TODO(yminchen): extract this filter out of WalkingSpeedControl and
  //  SwingFootTrajGen
  double cutoff_freq_ = 10; // in Hz.
  mutable Eigen::Vector3d filterred_com_vel_ = Eigen::Vector3d::Zero();
  mutable double last_timestamp_ = 0;
};

}  // namespace systems
}  // namespace dairlib
