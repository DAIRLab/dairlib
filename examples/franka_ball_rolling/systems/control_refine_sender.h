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

class ControlRefineSender : public drake::systems::LeafSystem<double> {
 public:
  /// A class that is used after the high level C3 plan is solved, relinearize
  /// the plant into a more refined LCS according to the solve time to calculate
  /// and send the desired state/force pair (and potential feedforward contact
  /// force and torque) to the downstream impedance controller, since it
  /// requires relinearizing plant, code would have a large portion overlapping
  /// with lcs_factory systems
  /// @param plant The standard <double> MultibodyPlant which includes the
  /// end-effector and the object (i.e. the simplified model)
  /// @param context the context (double) corresponds to the plant
  /// @param plant_ad An AutoDiffXd templated plant for gradient calculation
  /// @param context the context (AutoDiffXd) corresponds to the plant_ad
  /// @param contact_pairs A vector of SortedPairs of contact geometries
  /// @param c3_options C3 solving options (parameters), contain the contact
  /// model related information initialization parameters
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

  /// the second input port take in lcs state (in order to linearize the plant
  /// about that state)
  const drake::systems::InputPort<double>& get_input_port_lcs_state() const {
    return this->get_input_port(lcs_state_port_);
  }

  /// the third input port take in the heuristically determined end-effector
  /// tilted orientation
  const drake::systems::InputPort<double>& get_input_port_ee_orientation()
      const {
    return this->get_input_port(ee_orientation_port_);
  }

  /// the fourth input port take in the full model contact jacobian passed from
  /// forward kinematics part, for feedforward contact torque computation
  const drake::systems::InputPort<double>& get_input_port_contact_jacobian()
      const {
    return this->get_input_port(contact_jacobian_port_);
  }

  /// the first output port send out desired state for the impedance controller
  /// to track
  const drake::systems::OutputPort<double>& get_output_port_target() const {
    return this->get_output_port(target_port_);
  }

  /// the second output port send out potential feedforward contact torque
  const drake::systems::OutputPort<double>& get_output_port_contact_torque()
      const {
    return this->get_output_port(contact_torque_port_);
  }

 private:
  /// Output the desired state for the impedance controller to track, the actual
  /// calculation process is mainly done together with other calculation in
  /// UpdateSolveTimeHistory Event
  void CalcTrackTarget(const drake::systems::Context<double>& context,
                       TimestampedVector<double>* target) const;

  /// Output the potential feedforward contact torque to be added in impedance,
  /// the actual calculation process is mainly done together with other
  /// calculation in UpdateSolveTimeHistory Event
  void CalcFeedForwardTorque(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* torque_force) const;

  /// Do most of the calculation, also use drake state to record the previous
  /// time and approximate solve time dt and update per call
  drake::systems::EventStatus UpdateSolveTimeHistory(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state) const;

  /// Input and output ports index
  drake::systems::InputPortIndex c3_solution_port_;
  drake::systems::InputPortIndex lcs_state_port_;
  drake::systems::InputPortIndex ee_orientation_port_;
  drake::systems::InputPortIndex contact_jacobian_port_;

  drake::systems::OutputPortIndex target_port_;
  drake::systems::OutputPortIndex contact_torque_port_;

  /// Constructor variables
  // plant, context, contact pairs and c3 options
  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>& context_;
  const drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad_;
  drake::systems::Context<drake::AutoDiffXd>& context_ad_;
  const std::vector<drake::SortedPair<drake::geometry::GeometryId>>
      contact_pairs_;
  C3Options c3_options_;

  /// convenience for variable sizes
  int n_q_;
  int n_v_;
  int n_x_;
  int n_lambda_;
  int n_u_;
  int N_;

  /// do all the calculation in event status to avoid code duplication
  // desired state and force thus declared as drake state
  drake::systems::DiscreteStateIndex  x_next_idx_;
  drake::systems::DiscreteStateIndex force_idx_;

  /// solve_time_filter
  // all the solve time related info are also stored as drake state
  drake::systems::AbstractStateIndex dt_history_idx_;
  drake::systems::AbstractStateIndex prev_time_idx_;
  drake::systems::AbstractStateIndex dt_idx_;
  const int dt_filter_length_{10};
};

}  // namespace systems
}  // namespace dairlib