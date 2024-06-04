#pragma once

#include <drake/common/yaml/yaml_io.h>
#include <drake/multibody/plant/multibody_plant.h>

#include "dairlib/lcmt_robot_output.hpp"
#include "examples/franka_ball_rolling/parameters/heuristic_planner_params.h"
#include "examples/franka_ball_rolling/parameters/simulate_franka_params.h"
#include "examples/franka_ball_rolling/parameters/trajectory_params.h"
#include "solvers/c3.h"
#include "solvers/c3_options.h"
#include "systems/framework/output_vector.h"
#include "systems/framework/state_vector.h"

#include "drake/systems/framework/leaf_system.h"

#define PI 3.14159265359

namespace dairlib {
namespace systems {

class HeuristicGenerator : public drake::systems::LeafSystem<double> {
 public:
  /// A class that generates tracking target (time dependent trajectory or
  /// state-dependent path) for franka ball rolling example
  /// @param lcs_plant The standard <double> MultibodyPlant which includes the
  /// end-effector and the object (i.e. the simplified model)
  /// @param sim_param Simulation parameters for the full model plant,
  /// containing basic set up parameters
  /// @param traj_param tracking trajectory parameter, containing trajectory
  /// types and corresponding target parameters
  /// @param heuristic_param Heuristic parameters for high level planner,
  /// containing the manual determined end-effector tilt angle, and some
  /// initialization parameters
  /// @param c3_param C3 solving parameters, used to set different C3 gains in
  /// different gaiting phases
  HeuristicGenerator(const drake::multibody::MultibodyPlant<double>& lcs_plant,
                     const SimulateFrankaParams& sim_param,
                     const HeuristicPlannerParams& heuristic_param,
                     const BallRollingTrajectoryParams& trajectory_param,
                     const C3Options& c3_param);

  /// the first input port take in lcs state (i.e. state for simplified model)
  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(plant_state_port_);
  }

  /// the second input port take desired object trajectory from the trajectory
  /// generator
  const drake::systems::InputPort<double>& get_input_port_target() const {
    return this->get_input_port(input_target_port_);
  }

  /// the first output port send out time-based heuristic y_des (mainly for
  /// end-effector) to the C3 block
  const drake::systems::OutputPort<double>& get_output_port_target() const {
    return this->get_output_port(output_target_port_);
  }

  /// the second output port send out heuristically determined tilt orientation
  /// to impedance controller
  const drake::systems::OutputPort<double>& get_output_port_orientation()
      const {
    return this->get_output_port(orientation_port_);
  }

  /// the third output port send out time-based changing C3 gains
  const drake::systems::OutputPort<double>& get_output_port_cost_matrices()
      const {
    return this->get_output_port(cost_matrices_port_);
  }

  /// Set all the parameters needed in this system
  /// @param sim_param Simulation parameters for the full model plant,
  /// containing basic set up parameters
  /// @param heuristic_param Heuristic parameters for high level planner,
  /// containing the manual determined end-effector tilt angle, and some
  /// initialization parameters
  /// @param traj_param tracking trajectory parameter, containing trajectory
  /// types and corresponding target parameters
  /// @param c3_param C3 solving parameters, used to set different C3 gains in
  /// different gaiting phases
  void SetHeuristicParameters(
      const SimulateFrankaParams& sim_param,
      const HeuristicPlannerParams& heuristic_param,
      const BallRollingTrajectoryParams& trajectory_param,
      const C3Options& c3_param);

 private:
  /// Record the first timestamp that the system received message, other times
  /// just pass and do nothing
  drake::systems::EventStatus UpdateFirstMessageTime(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state) const;

  /// Calculate the time-based heuristically determined end-effector position
  /// and combine it with the target object trajectory to form target lcs state
  void CalcHeuristicTarget(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* target_state) const;

  /// Calculate the heuristically determined end-effector orientation
  void CalcHeuristicTilt(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* target_orientation) const;

  /// Calculate the time-based heuristically determined C3 gains
  void CalcHeuristicCostMatrices(
      const drake::systems::Context<double>& context,
      solvers::C3::CostMatrices* Cost_matrices) const;

  /// Input and output ports index
  drake::systems::InputPortIndex plant_state_port_;
  drake::systems::InputPortIndex input_target_port_;
  drake::systems::OutputPortIndex output_target_port_;
  drake::systems::OutputPortIndex orientation_port_;
  drake::systems::OutputPortIndex cost_matrices_port_;

  /// Flags and record for the first message time index
  drake::systems::AbstractStateIndex first_message_time_idx_;
  drake::systems::AbstractStateIndex received_first_message_idx_;

  /// useful variables, necessary dimensions derived from plant
  int n_q;
  int n_v;
  int n_x;

  /// time based phase and gait settings
  double roll_phase_;
  double return_phase_;
  double rolling_period_;
  VectorXd gait_parameters_;
  double table_offset_;
  double ee_default_height;
  int axis_option_;
  double tilt_degrees_;

  /// C3 gains (diagonal elements)
  C3Options c3_param_;
  VectorXd q_new_vector_;
  VectorXd g_new_vector_;

  /// set tilted end-effector towards circle center (tilt option 2) only used
  /// for circle trajectory
  double x_c_;
  double y_c_;
};

}  // namespace systems
}  // namespace dairlib