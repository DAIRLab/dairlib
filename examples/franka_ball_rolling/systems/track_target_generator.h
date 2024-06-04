#pragma once

#include <drake/common/yaml/yaml_io.h>
#include <drake/multibody/plant/multibody_plant.h>

#include "dairlib/lcmt_robot_output.hpp"
#include "examples/franka_ball_rolling/parameters/simulate_franka_params.h"
#include "examples/franka_ball_rolling/parameters/trajectory_params.h"
#include "systems/framework/output_vector.h"
#include "systems/framework/state_vector.h"

#include "drake/systems/framework/leaf_system.h"
#define PI 3.14159265359

namespace dairlib {
namespace systems {

class TargetGenerator : public drake::systems::LeafSystem<double> {
 public:
  /// A class that generates tracking target (time dependent trajectory or
  /// state-dependent path) for franka ball rolling example
  /// @param lcs_plant The standard <double> MultibodyPlant which includes the
  /// end-effector and the object (i.e. the simplified model)
  /// @param sim_param Simulation parameters for the full model plant,
  /// containing basic set up parameters
  /// @param traj_param tracking trajectory parameter, containing trajectory
  /// types and corresponding target parameters
  TargetGenerator(const drake::multibody::MultibodyPlant<double>& lcs_plant,
                  const SimulateFrankaParams& sim_param,
                  const BallRollingTrajectoryParams& traj_param);

  /// the first input port take in lcs state (i.e. state for simplified model)
  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(plant_state_port_);
  }

  /// the output port send out y_des (tracking target of simplified model) to
  /// the Heuristic planning block, y_d contain the object state, first 4 are
  /// orientation (quaternion) and then position xyz
  const drake::systems::OutputPort<double>& get_output_port_target() const {
    return this->get_output_port(target_port_);
  }

  /// Set all the parameters needed in this system
  /// @param sim_param Simulation parameters for the full model plant,
  /// containing basic set up parameters
  /// @param traj_param tracking trajectory parameter, containing trajectory
  /// types and corresponding target parameters
  void SetTrajectoryParameters(const SimulateFrankaParams& sim_param,
                               const BallRollingTrajectoryParams& traj_param);

 private:
  /// Record the first timestamp that the system received message, other times
  /// just pass and do nothing
  drake::systems::EventStatus UpdateFirstMessageTime(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state) const;

  /// Calculate the desired trajectory. Specifically, only assigns the object's
  /// (ball's) target orientation and position
  void CalcTrackTarget(const drake::systems::Context<double>& context,
                       drake::systems::BasicVector<double>* target) const;

  /// Input and output ports index
  drake::systems::InputPortIndex plant_state_port_;
  drake::systems::OutputPortIndex target_port_;

  /// Flags and record for the first message time index
  drake::systems::AbstractStateIndex first_message_time_idx_;
  drake::systems::AbstractStateIndex received_first_message_idx_;

  /// useful variables, necessary dimensions derived from plant
  int n_q;
  int n_v;
  int n_x;

  /// trajectory type, set from parameter input
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