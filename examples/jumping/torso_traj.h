#pragma once

#include "attic/multibody/rigidbody_utils.h"
#include "systems/controllers/control_utils.h"
#include "systems/framework/output_vector.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace examples {
namespace jumping {
namespace osc {

class TorsoTraj : public drake::systems::LeafSystem<double> {
 public:
  TorsoTraj(const RigidBodyTree<double>& tree,
            drake::trajectories::PiecewisePolynomial<double> foot_traj);

  const drake::systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_fsm_input_port() const {
    return this->get_input_port(fsm_port_);
  }

 private:

  void CalcTraj(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  const RigidBodyTree<double>& tree_;
  drake::trajectories::PiecewisePolynomial<double> torso_angle_traj_;

  int state_port_;
  int fsm_port_;

  // Eigen::Vector3d front_contact_disp_ = Eigen::Vector3d(-0.0457, 0.112, 0);
  // Eigen::Vector3d rear_contact_disp_ = Eigen::Vector3d(0.088, 0, 0);
};

}  // namespace osc
}  // namespace jumping
}  // namespace examples
}  // namespace dairlib
