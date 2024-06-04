#pragma once

#include <drake/common/yaml/yaml_io.h>
#include <drake/multibody/plant/multibody_plant.h>

#include "dairlib/lcmt_robot_output.hpp"
#include "examples/franka_ball_rolling/parameters/heuristic_planner_params.h"
#include "examples/franka_ball_rolling/parameters/simulate_franka_params.h"
#include "systems/framework/output_vector.h"
#include "systems/framework/state_vector.h"

#include "drake/systems/framework/leaf_system.h"
#define PI 3.14159265359

namespace dairlib {
namespace systems {

class MoveToInitial : public drake::systems::LeafSystem<double> {
  /// A class that use to drive franka to the initial ready pose for ball
  /// rolling kinda mimic ROS MoveIt functionalities
  /// @param sim_param Simulation parameters for the plant, containing basic set
  /// up parameters
  /// @param heuristic_param Heuristic parameters for high level planner,
  /// containing the manual determined end-effector tilt angle, and some
  /// initialization parameters
 public:
  MoveToInitial(const SimulateFrankaParams& sim_param,
                const HeuristicPlannerParams& heuristic_param);

  /// the first input port take in current lcs state (i.e. state for simplified
  /// model)
  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(franka_input_port_);
  }

  /// the output port send out the desired lcs state
  const drake::systems::OutputPort<double>& get_output_port_target() const {
    return this->get_output_port(target_port_);
  }

  /// Set all the parameters needed in this system
  /// @param sim_param Simulation parameters for the plant, containing basic set
  /// up parameters
  /// @param heuristic_param Heuristic parameters for high level planner,
  /// containing the manual determined end-effector tilt angle, and some
  /// initialization parameters
  void SetParameters(const SimulateFrankaParams& sim_param,
                     const HeuristicPlannerParams heuristic_param);

 private:
  /// Record the first timestamp that the system received message, other times
  /// just pass and do nothing
  drake::systems::EventStatus UpdateFirstMessageTime(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state) const;

  /// Calculate the desired lcs state. For this system, it is just assigning
  /// end-effector to the initial ready position
  void CalcTarget(const drake::systems::Context<double>& context,
                  TimestampedVector<double>* output) const;

  /// Calculate the desired contact feedforward torque and force. For this
  /// system, it is just setting everything to zero
  void CalcFeedForwardTorque(const drake::systems::Context<double>& context,
                             drake::systems::BasicVector<double>* torque) const;

  /// Used in CalcTarget, given start and finish position of the end-effector,
  /// do linear interpolation to move to that
  /// @param start The start point of the end-effector (should align with the
  /// whole plant setting)
  /// @param finish The finish point of the end-effector, it is also the
  /// initialization point of ball rolling task
  /// @param curr_time current time, used in calculating linear interpolation
  /// @param stabilize_time first stage time, just stay around start point
  /// @param move_time second stage time, do linear interpolated motion to
  /// finish point
  std::vector<Eigen::Vector3d> move_to_initial_position(
      const Eigen::Vector3d& start, const Eigen::Vector3d& finish,
      double curr_time, double stabilize_time, double move_time) const;

  /// Used in CalcTarget, given start and finish orientation of the
  /// end-effector, do linear interpolation to move to that
  /// @param start The start orientation of the end-effector (should align with
  /// the whole plant setting)
  /// @param finish The finish orientation of the end-effector, it is also the
  /// initialization orientation of ball rolling task
  /// @param curr_time current time, used in calculating linear interpolation
  /// @param stabilize_time first stage time, just stay around start point
  /// @param move_time second stage time, do linear interpolated motion to
  /// finish point
  Eigen::Quaterniond move_to_initial_orientation(
      const Eigen::Quaterniond& start_orientation, double tilt_degrees,
      double curr_time, double stabilize_time, double move_time) const;

  /// Input and output ports index
  drake::systems::InputPortIndex franka_input_port_;
  drake::systems::OutputPortIndex target_port_;
  drake::systems::OutputPortIndex contact_torque_port_;

  /// Flags and record for the first message time index
  drake::systems::AbstractStateIndex first_message_time_idx_;
  drake::systems::AbstractStateIndex received_first_message_idx_;

  /// Stabilize and move time to the initial position
  double stabilize_time1_;
  double move_time_;

  /// start and finish position move target
  Vector3d initial_start_;
  Vector3d initial_finish_;
  double x_c_;
  double y_c_;
  double traj_radius_;
  double initial_phase_;

  /// target (finish) tilted orientation for the end-effector
  double tilt_degrees_;
};

}  // namespace systems
}  // namespace dairlib