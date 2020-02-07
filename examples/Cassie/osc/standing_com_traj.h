#pragma once

#include "systems/controllers/control_utils.h"
#include "systems/framework/output_vector.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace cassie {
namespace osc {

class StandingComTraj : public drake::systems::LeafSystem<double> {
 public:
  StandingComTraj(const RigidBodyTree<double>& tree, int pelvis_idx,
                  int left_foot_idx, int right_foot_idx);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }

 private:
  void CalcDesiredTraj(const drake::systems::Context<double>& context,
                       drake::trajectories::Trajectory<double>* traj) const;

  const RigidBodyTree<double>& tree_;
  int pelvis_idx_;
  int left_foot_idx_;
  int right_foot_idx_;

  int state_port_;

  Eigen::Vector3d front_contact_disp_ = Eigen::Vector3d(-0.0457, 0.112, 0);
  Eigen::Vector3d rear_contact_disp_ = Eigen::Vector3d(0.088, 0, 0);
};

}  // namespace osc
}  // namespace cassie
}  // namespace dairlib
