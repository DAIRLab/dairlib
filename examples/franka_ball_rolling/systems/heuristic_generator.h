#pragma once

#include <drake/multibody/plant/multibody_plant.h>
//#include "systems/framework/state_vector.h"
#include <drake/common/yaml/yaml_io.h>
#include "systems/framework/output_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/franka_ball_rolling/parameters/simulate_franka_params.h"
#include "examples/franka_ball_rolling/parameters/heuristic_gait_params.h"
#include "examples/franka_ball_rolling/parameters/trajectory_params.h"

#include "solvers/c3_options.h"
#include "solvers/c3.h"

#define PI 3.14159265359


namespace dairlib {
namespace systems {

/// A class that generates the time-based heuristic position
/// Should send out desired end-effector pose and C3 gains

class HeuristicGenerator
    : public drake::systems::LeafSystem<double> {
 public:
  HeuristicGenerator(
      const drake::multibody::MultibodyPlant<double>& lcs_plant,
      const SimulateFrankaParams& sim_param,
      const HeuristicGaitParams& heuristic_param,
      const BallRollingTrajectoryParams& trajectory_param,
      const C3Options& c3_param);

  /// the first input port take in lcs state (i.e. state for simplified model)
  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(plant_state_port_);
  }

  /// the second input port take desired object trajectory from the trajectory generator
  const drake::systems::InputPort<double>& get_input_port_target() const {
      return this->get_input_port(input_target_port_);
  }

  /// the first output port send out time-based heuristic y_des (mainly for end-effector) to the C3 block
  /// potential TODO:
  /// y_d contain the whole desired state, can seperate robot and object, position and velocity ports if needed
  const drake::systems::OutputPort<double>&
  get_output_port_target() const {
    return this->get_output_port(output_target_port_);
  }

  /// the second output port send out heuristicly determined tilt orientation to impedance controller
  const drake::systems::OutputPort<double>&
  get_output_port_orientation() const {
      return this->get_output_port(orientation_port_);
  }

  /// the third output port send out time-based C3 gains
  const drake::systems::OutputPort<double>&
  get_output_port_gain() const {
        return this->get_output_port(gain_port_);
  }

  void SetHeuristicParameters(const SimulateFrankaParams& sim_param,
                              const HeuristicGaitParams& heuristic_param,
                              const BallRollingTrajectoryParams& trajectory_param,
                              const C3Options& c3_param);

 private:
  void CalcHeuristicTarget(const drake::systems::Context<double>& context,
                           drake::systems::BasicVector<double>* target_state) const;
  void CalcHeuristicTilt(const drake::systems::Context<double>& context,
                             drake::systems::BasicVector<double>* target_orientation) const;
  void CalcHeuristicGain(const drake::systems::Context<double>& context,
                            solvers::C3::CostMatrices* Cost_matrices) const;

  drake::systems::InputPortIndex plant_state_port_;
  drake::systems::InputPortIndex input_target_port_;

  drake::systems::OutputPortIndex output_target_port_;
  drake::systems::OutputPortIndex orientation_port_;
  drake::systems::OutputPortIndex gain_port_;

  double roll_phase_;
  double return_phase_;
  double rolling_period_;
  VectorXd gait_parameters_;
  double table_offset_;
  int axis_option_;
  double tilt_degrees_;
  double x_c_;
  double y_c_;
  VectorXd q_new_vector_;

  double settling_time_;
  double ee_default_height;

  C3Options c3_param_;
};

}  // namespace systems
}  // namespace dairlib