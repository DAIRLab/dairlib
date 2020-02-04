#pragma once

#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "systems/framework/output_vector.h"
#include "systems/controllers/control_utils.h"
#include "attic/multibody/rigidbody_utils.h"


using drake::trajectories::PiecewisePolynomial;
using Eigen::VectorXd;

namespace dairlib {
namespace examples {
namespace jumping {
namespace osc {

class CoMTraj : public drake::systems::LeafSystem<double> {
 public:
  CoMTraj(const RigidBodyTree<double>& tree,
          int hip_idx,
          int left_foot_idx,
          int right_foot_idx,
          PiecewisePolynomial<double> crouch_traj,
          double height = 0.7009);

  const drake::systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_fsm_input_port() const {
    return this->get_input_port(fsm_port_);
  }

 private:
  PiecewisePolynomial<double> generateNeutralTraj(const
      drake::systems::Context<double>& context,
      VectorXd& q, VectorXd& v) const;
  PiecewisePolynomial<double> generateCrouchTraj(const
      drake::systems::Context<double>& context,
      VectorXd& q, VectorXd& v) const;
  PiecewisePolynomial<double> generateFlightTraj(const
      drake::systems::Context<double>& context,
      VectorXd& q, VectorXd& v) const;
  PiecewisePolynomial<double> generateLandingTraj(const
      drake::systems::Context<double>& context,
      VectorXd& q, VectorXd& v) const;

  drake::systems::EventStatus DiscreteVariableUpdate(const
      drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void CalcTraj(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  const RigidBodyTree<double>& tree_;
  int time_idx_;
  int fsm_idx_;
  int com_x_offset_idx_;

  int hip_idx_;
  int left_foot_idx_;
  int right_foot_idx_;
  PiecewisePolynomial<double> crouch_traj_;
  double height_;

  int state_port_;
  int fsm_port_;
  PiecewisePolynomial<double> generateBalancingComTraj(VectorXd& q) const;
};

}  // namespace osc
}  // namespace jumping
}  // namespace examples
}  // namespace dairlib


