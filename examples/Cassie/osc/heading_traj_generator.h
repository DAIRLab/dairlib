#pragma once

#include "systems/controllers/control_utils.h"
#include "systems/framework/output_vector.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace cassie {
namespace osc {

/// `HeadingTrajGenerator` set the desired angles to be 0's for pitch and roll
/// and take the desired yaw velocity from input port. The desired (roll, pitch,
/// yaw) is further transformed into quaternion representation, and becomes the
/// output of `HeadingTrajGenerator`.
///
/// Input:
///  - Desired yaw velocity
///
/// Output:
///  - A 4D constant polynomial which contains quaterinon's w, x, y and z.
///
/// Requirement: quaternion floating-based Cassie only
class HeadingTrajGenerator : public drake::systems::LeafSystem<double> {
 public:
  HeadingTrajGenerator(const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context);

  // Input/output ports
  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_yaw() const {
    return this->get_input_port(des_yaw_port_);
  }

 private:
  void CalcHeadingTraj(const drake::systems::Context<double>& context,
                       drake::trajectories::Trajectory<double>* traj) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  const drake::multibody::BodyFrame<double>& world_;
  const drake::multibody::Body<double>& pelvis_;

  drake::systems::InputPortIndex state_port_;
  drake::systems::InputPortIndex des_yaw_port_;
};

}  // namespace osc
}  // namespace cassie
}  // namespace dairlib
