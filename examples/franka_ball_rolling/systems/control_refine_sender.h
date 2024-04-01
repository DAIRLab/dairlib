#pragma once

#include <drake/multibody/plant/multibody_plant.h>
//#include "systems/framework/state_vector.h"
#include <drake/common/yaml/yaml_io.h>
#include "systems/framework/output_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/franka_ball_rolling/parameters/simulate_franka_params.h"
#include "examples/franka_ball_rolling/parameters/trajectory_params.h"

#include "solvers/c3_options.h"
#include "solvers/lcs.h"
#define PI 3.14159265359

namespace dairlib {
namespace systems {

/// A class that is used to relinearize the plant into a more refined LCS
/// according to the solve time and send the desired state/force pair (and
/// potential feedforward torque) to downstream impedance controller

/// since it requires relinearizing plant, code would largely overlap with
/// lcs_factory systems

class ControlRefineSender
    : public drake::systems::LeafSystem<double> {
 public:
  ControlRefineSender(
          const drake::multibody::MultibodyPlant<double>& plant,
          drake::systems::Context<double>& context,
          const drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
          drake::systems::Context<drake::AutoDiffXd>& context_ad,
          const std::vector<drake::SortedPair<drake::geometry::GeometryId>> contact_geoms,
          C3Options c3_options);

  /// the first input port take in c3 solution (only need to u part)
  const drake::systems::InputPort<double>& get_input_port_c3_solution() const {
    return this->get_input_port(c3_solution_port_);
  }
  /// the second input port take in lcs state (in order to linearize the plant)
  const drake::systems::InputPort<double>& get_input_port_lcs_state() const {
      return this->get_input_port(lcs_state_port_);
  }

  /// the first output port send out desired state of the impedance controller to track
  const drake::systems::OutputPort<double>&
  get_output_port_target() const {
    return this->get_output_port(target_port_);
  }

  /// the second output port send out potential feedforward torque
  const drake::systems::OutputPort<double>&
  get_output_port_contact_torque() const {
      return this->get_output_port(contact_torque_port_);
  }

  void SetParameters(const SimulateFrankaParams& sim_param,
                     const BallRollingTrajectoryParams& traj_param);

 private:
  void CalcTrackTarget(const drake::systems::Context<double>& context,
                       drake::systems::BasicVector<double>* target) const;
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  drake::systems::InputPortIndex c3_solution_port_;
  drake::systems::InputPortIndex lcs_state_port_;


  drake::systems::OutputPortIndex target_port_;
  drake::systems::OutputPortIndex contact_torque_port_;

};

}  // namespace systems
}  // namespace dairlib