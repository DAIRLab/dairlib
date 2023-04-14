#pragma once

#include "systems/controllers/control_utils.h"
#include "systems/framework/output_vector.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace cassie {
namespace osc {

/// `PelvisPitchTrajGenerator` sets the desired pitch angle to whatever the
/// input says, with 0 roll/yaw. intended to be used with a separate heading
/// traj gen
///
/// Input:
///  - Desired pelvis pitch
///
/// Output:
///  - A 4D constant polynomial which contains quaternion's w, x, y and z.
///
/// Requirement: quaternion floating-based Cassie only
class PelvisPitchTrajGenerator : public drake::systems::LeafSystem<double> {
 public:
  PelvisPitchTrajGenerator(const drake::multibody::MultibodyPlant<double>& plant,
                       drake::systems::Context<double>* context);

  // Input/output ports
  const drake::systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_pitch_input_port() const {
    return this->get_input_port(des_pitch_port_);
  }

 private:
  drake::systems::EventStatus PitchFilterUpdate(
      const drake::systems::Context<double>& context,
                         drake::systems::State<double>* state) const;
  void CalcPitchTraj(const drake::systems::Context<double>& context,
                       drake::trajectories::Trajectory<double>* traj) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  const drake::multibody::BodyFrame<double>& world_;
  const drake::multibody::Body<double>& pelvis_;

  drake::systems::InputPortIndex state_port_;
  drake::systems::InputPortIndex des_pitch_port_;

  drake::systems::DiscreteStateIndex filtered_pitch_idx_;
  drake::systems::DiscreteStateIndex prev_timestamp_idx_;
};

}  // namespace osc
}  // namespace cassie
}  // namespace dairlib
