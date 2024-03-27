#pragma once

#include <drake/multibody/plant/multibody_plant.h>
//#include "systems/framework/state_vector.h"
#include <drake/common/yaml/yaml_io.h>
#include "systems/framework/output_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/franka_ball_rolling/parameters/simulate_franka_params.h"
#define PI 3.14159265359

namespace dairlib {
namespace systems {

/// A class that generates tracking target for franka ball rolling example
/// Modified and simplified from plate balancing example for ball rolling

class TargetGenerator
    : public drake::systems::LeafSystem<double> {
 public:
  TargetGenerator(
      const drake::multibody::MultibodyPlant<double>& object_plant);

  /// the input port take lcmt_robot_output (x, u, timestamp) in from the state estimation block
  /// only need the state x to generate the target
  /// potential TODO:
  /// x contain the whole plants state, can seperate robot and object, position and velocity ports if needed
  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(plant_state_port_);
  }

  /// the output port send out y_des (tracking target of simplified model) to the Heuristic planning block
  /// potential TODO:
  /// y_d contain the whole desired state, can seperate robot and object, position and velocity ports if needed
  const drake::systems::OutputPort<double>&
  get_output_port_target() const {
    return this->get_output_port(target_port_);
  }

  void SetTrajectoryParameters(const int& trajectory_type, const double& traj_radius, const double& x_c, const double& y_c,
                               const double& lead_angle, const double& velocity_circle,
                               const double& start_x, const double& start_y, const double& end_x, const double& end_y,
                               const double& lead_step, const double& velocity_line);

 private:
  void CalcTrackTarget(const drake::systems::Context<double>& context,
                       drake::systems::BasicVector<double>* target) const;
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  drake::systems::InputPortIndex plant_state_port_;
  drake::systems::OutputPortIndex target_port_;

  int trajectory_type_;

  /// circle trajectory parameters (general)
  double traj_radius_;
  double x_c_;
  double y_c_;
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
};

}  // namespace systems
}  // namespace dairlib