#pragma once

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/common/yaml/yaml_io.h>
#include "systems/framework/state_vector.h"
#include "systems/framework/output_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/franka_ball_rolling/parameters/simulate_franka_params.h"
#include "examples/franka_ball_rolling/parameters/heuristic_planner_params.h"
#define PI 3.14159265359

namespace dairlib {
namespace systems {

/// A class that use to drive franka to the initial ready pose for ball rolling
/// kinda mimic ROS MoveIt functionalities

class MoveToInitial
    : public drake::systems::LeafSystem<double> {
 public:
  MoveToInitial(
      const SimulateFrankaParams& sim_param,
      const HeuristicPlannerParams& heuristic_param);

  /// the first input port take in lcs state (i.e. state for simplified model)
  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(franka_input_port_);
  }

  /// the output port send out y_des (tracking target of simplified model) to the Heuristic planning block
  /// y_d contain the object position, first 4 quaternion and position xyz
  const drake::systems::OutputPort<double>& get_output_port_target() const {
    return this->get_output_port(target_port_);
  }

  void SetParameters(const SimulateFrankaParams& sim_param,
                               const HeuristicPlannerParams heuristic_param);

 private:

  drake::systems::EventStatus UpdateFirstMessageTime(
            const drake::systems::Context<double>& context,
            drake::systems::State<double>* state) const;

  void CalcTarget(const drake::systems::Context<double>& context,
                  TimestampedVector<double>* output) const;
  void CalcFeedForwardTorque(const drake::systems::Context<double>& context,
                               drake::systems::BasicVector<double>* torque) const;

  std::vector<Eigen::Vector3d> move_to_initial_position(
          const Eigen::Vector3d& start,
          const Eigen::Vector3d& finish,
          double curr_time, double stabilize_time,
          double move_time) const;

  Eigen::Quaterniond move_to_initial_orientation(
            const Eigen::Quaterniond& start_orientation,
            double tilt_degrees,
            double curr_time, double stabilize_time,
            double move_time) const;

  drake::systems::InputPortIndex franka_input_port_;
  drake::systems::OutputPortIndex target_port_;
  drake::systems::OutputPortIndex  contact_torque_port_;

  int first_message_time_idx_;
  int received_first_message_idx_;

  double stabilize_time1_;
  double move_time_;

  Vector3d initial_start_;
  Vector3d initial_finish_;
  double x_c_;
  double y_c_;
  double traj_radius_;
  double initial_phase_;

  // TODO:: add initial end-effector orientation parameter
  double tilt_degrees_;
};

}  // namespace systems
}  // namespace dairlib