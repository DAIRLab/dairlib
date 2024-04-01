#pragma once

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/common/yaml/yaml_io.h>
#include "systems/framework/state_vector.h"
#include "systems/framework/output_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/franka_ball_rolling/parameters/simulate_franka_params.h"
#include "examples/franka_ball_rolling/parameters/trajectory_params.h"
#define PI 3.14159265359

namespace dairlib {
namespace systems {

/// A class that generates tracking target for franka ball rolling example
/// Modified and simplified from plate balancing example for ball rolling

class TargetGenerator
    : public drake::systems::LeafSystem<double> {
 public:
  TargetGenerator(
      const drake::multibody::MultibodyPlant<double>& lcs_plant,
      const SimulateFrankaParams& sim_param,
      const BallRollingTrajectoryParams& traj_param);

  /// the first input port take in lcs state (i.e. state for simplified model)
  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(plant_state_port_);
  }

  /// the output port send out y_des (tracking target of simplified model) to the Heuristic planning block
  /// y_d contain the object position, first 4 quaternion and position xyz
  const drake::systems::OutputPort<double>&
  get_output_port_target() const {
    return this->get_output_port(target_port_);
  }

  void SetTrajectoryParameters(const SimulateFrankaParams& sim_param,
                               const BallRollingTrajectoryParams& traj_param);

 private:
  void CalcTrackTarget(const drake::systems::Context<double>& context,
                       drake::systems::BasicVector<double>* target) const;

  drake::systems::InputPortIndex plant_state_port_;
  drake::systems::OutputPortIndex target_port_;

  int trajectory_type_;

  /// circle trajectory parameters (general)
  double traj_radius_;
  double x_c_;
  double y_c_;
  double initial_phase_;

  // state based circular specific setting
  double lead_angle_;
  // time based circular specific setting
  double velocity_circle_;

  /// line trajectory parameters (general)
  double start_x_;
  double start_y_;
  double end_x_;
  double end_y_;
  // state based line specific setting
  double lead_step_;
  // time based line specific setting
  double velocity_line_;

  /// object on the table, height is fixed
  double object_height_;
};

}  // namespace systems
}  // namespace dairlib