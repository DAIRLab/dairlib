#pragma once

#include "systems/controllers/control_utils.h"
#include "systems/framework/output_vector.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace cassie {
namespace osc_walk {

/// `HeadingTrajGenerator` set the desired angles to be 0's for pitch and roll
/// and take the desired yaw velocity from input port. The desired (roll, pitch,
/// yaw) is further transformed into quaternion representation, and becomes the
/// output of `HeadingTrajGenerator`.
///
/// Input:
///  - Desired yaw position
///
/// Output:
///  - A 4D constant polynomial which contains quaterinon's w, x, y and z.
///
/// Requirement: quaternion floating-based Cassie only
class HeadingTrajGenerator : public drake::systems::LeafSystem<double> {
 public:
  HeadingTrajGenerator(const RigidBodyTree<double>& tree, int pelvis_idx);

  // Input/output ports
  const drake::systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_yaw_input_port() const {
    return this->get_input_port(des_yaw_port_);
  }

 private:
  void CalcHeadingTraj(const drake::systems::Context<double>& context,
                       drake::trajectories::Trajectory<double>* traj) const;

  const RigidBodyTree<double>& tree_;
  int pelvis_idx_;

  int state_port_;
  int des_yaw_port_;
};

}  // namespace osc_walk
}  // namespace cassie
}  // namespace dairlib
