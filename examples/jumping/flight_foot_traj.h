#pragma once

#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "systems/framework/output_vector.h"
#include "systems/controllers/control_utils.h"
#include "attic/multibody/rigidbody_utils.h"


namespace dairlib {
namespace examples {
namespace jumping {
namespace osc {

class FlightFootTraj : public drake::systems::LeafSystem<double> {
 public:
  FlightFootTraj(const RigidBodyTree<double>& tree,
                 int hip_idx,
                 int left_foot_idx,
                 int right_foot_idx,
                 bool isLeftFoot,
                 drake::trajectories::PiecewisePolynomial<double> foot_traj,
                 double height = 0.8,
                 double foot_offset = 0.3);

  const drake::systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_fsm_input_port() const {
    return this->get_input_port(fsm_port_);
  }

 private:
  drake::trajectories::PiecewisePolynomial<double> generateFlightTraj(
      const drake::systems::Context<double>& context,
      Eigen::VectorXd* q,
      Eigen::VectorXd* v,
      double t) const;

  void CalcTraj(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  const RigidBodyTree<double>& tree_;
  int hip_idx_;
  int left_foot_idx_;
  int right_foot_idx_;
  bool isLeftFoot_;
  drake::trajectories::PiecewisePolynomial<double> foot_traj_;
  double height_;
  double foot_offset_;

  int state_port_;
  int fsm_port_;

  // Eigen::Vector3d front_contact_disp_ = Eigen::Vector3d(-0.0457, 0.112, 0);
  // Eigen::Vector3d rear_contact_disp_ = Eigen::Vector3d(0.088, 0, 0);
};

}  // namespace osc
}  // namespace jumping
}  // namespace examples
}  // namespace dairlib


