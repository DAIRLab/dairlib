#pragma once

#include <drake/common/yaml/yaml_io.h>
#include <drake/multibody/plant/multibody_plant.h>

#include "dairlib/lcmt_robot_output.hpp"
#include "examples/franka_ball_rolling/parameters/simulate_franka_params.h"
#include "examples/franka_ball_rolling/parameters/trajectory_params.h"
#include "solvers/c3_options.h"
#include "solvers/c3_output.h"
#include "solvers/lcs.h"
#include "systems/framework/output_vector.h"
#include "systems/framework/timestamped_vector.h"

#include "drake/systems/framework/leaf_system.h"
#define PI 3.14159265359

namespace dairlib {
namespace systems {

/// A class that is used to relinearize the plant into a more refined LCS
/// according to the solve time and send the desired state/force pair (and
/// potential feedforward torque) to downstream impedance controller

/// since it requires relinearizing plant, code would largely overlap with
/// lcs_factory systems

class ControlRefineSender : public drake::systems::LeafSystem<double> {
 public:
  ControlRefineSender(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>& context,
      const drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
      drake::systems::Context<drake::AutoDiffXd>& context_ad,
      const std::vector<drake::SortedPair<drake::geometry::GeometryId>>
          contact_pairs,
      C3Options c3_options);

  /// the first input port take in c3 solution (only need to u part)
  const drake::systems::InputPort<double>& get_input_port_c3_solution() const {
    return this->get_input_port(c3_solution_port_);
  }
  /// the second input port take in lcs state (in order to linearize the plant)
  const drake::systems::InputPort<double>& get_input_port_lcs_state() const {
    return this->get_input_port(lcs_state_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_ee_orientation()
      const {
    return this->get_input_port(ee_orientation_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_contact_jacobian()
      const {
    return this->get_input_port(contact_jacobian_port_);
  }

  /// the first output port send out desired state of the impedance controller
  /// to track
  const drake::systems::OutputPort<double>& get_output_port_target() const {
    return this->get_output_port(target_port_);
  }

  /// the second output port send out potential feedforward torque
  const drake::systems::OutputPort<double>& get_output_port_contact_torque()
      const {
    return this->get_output_port(contact_torque_port_);
  }

 private:
  void CalcTrackTarget(const drake::systems::Context<double>& context,
                       TimestampedVector<double>* target) const;

  void CalcFeedForwardTorque(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* torque_force) const;

  // update discrete states to get the filtered approximate solve time
  drake::systems::EventStatus UpdateSolveTimeHistory(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state) const;

  drake::systems::InputPortIndex c3_solution_port_;
  drake::systems::InputPortIndex lcs_state_port_;
  drake::systems::InputPortIndex ee_orientation_port_;
  drake::systems::InputPortIndex contact_jacobian_port_;

  drake::systems::OutputPortIndex target_port_;
  drake::systems::OutputPortIndex contact_torque_port_;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>& context_;
  const drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad_;
  drake::systems::Context<drake::AutoDiffXd>& context_ad_;
  const std::vector<drake::SortedPair<drake::geometry::GeometryId>>
      contact_pairs_;

  C3Options c3_options_;

  // convenience for variable sizes
  int n_q_;
  int n_v_;
  int n_x_;
  int n_lambda_;
  int n_u_;
  int N_;

  // do all the calculation in event status to avoid code duplication
  int x_next_idx_;
  int force_idx_;

  // solve_time_filter
  int dt_history_idx_;
  const int dt_filter_length_{10};
  int prev_time_idx_;
  int dt_idx_;
};

}  // namespace systems
}  // namespace dairlib