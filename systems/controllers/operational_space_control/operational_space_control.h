#pragma once

#include <vector>
#include <string>
#include "drake/systems/framework/leaf_system.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"

#include "systems/framework/output_vector.h"
#include "systems/controllers/operational_space_control/osc_tracking_data.h"

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"

namespace dairlib {
namespace systems {
namespace controllers {

/// `OperationalSpaceControl` takes in desired trajectory and outputs torque
/// command of the motors.

/// Inputs of the constructor:
///  - `tree_w_spr` a RigidBodyTree with springs
///  - `tree_wo_spr` a RigidBodyTree without springs
/// The springs here refer to the compliant components in the robots.

/// OSC calculates feedback positions/velocities from `tree_w_spr`,
/// but in the optimization it uses `tree_wo_spr`. The reason of using
/// RigidBodyTree without spring is that the OSC cannot track desired
/// acceleration instantaneously when springs exist. (relative degrees of 4)

/// Requirement:
///  - the joints name (except for the spring joints) in `tree_w_spr` must be
///    the same as those of `tree_wo_spr`

/// If the robot doesn't have any spring, the user can just pass two identical
/// RigidBodyTree into the constructor.

/// Users define
///     costs,
///     constraints,
///     and the trajectories to track,
/// and add them through `OperationalSpaceControl`'s' methods.

/// Before adding desired trajectories to `OperationalSpaceControl`, users have
/// to create
///     `TransTaskSpaceTrackingData`,
///     `RotTaskSpaceTrackingData`,
///     `JointSpaceTrackingData`,
///     and/or `AbstractTrackingData`.

/// If the desired trajectory is constant, users don't need to connect the
/// input ports of `OperationalSpaceControl` to trajectory source blocks.
/// Instead, the users have to call the function AssignConstTrajToInputPorts()
/// after drake::systems::Diagram is built.

/// If the users want to create the desired trajectory source themselves,
/// the outputs of trajectory blocks need to be of the derived classes of
///     drake::trajectories::Trajectory<double>
/// such as `PiecewisePolynomial` and `ExponentialPlusPiecewisePolynomial`.
/// The users can connect the output ports of the desired trajecotry blocks to
/// the corresponding input ports of `OperationalSpaceControl` by using
/// the method get_tracking_data_input_port().
/// Users can also call the utility function ConnectPortsForNonConstTraj() if
/// they don't want to connect the input/output ports manually.

/// The procedure of setting up `OperationalSpaceControl`:
///   1. create an instance of `OperationalSpaceControl`
///   2. add costs/constraints/desired trajectories
///   3. call ConstructOSC()
///   4. (if the users created desired trajectory blocks by themselves) connect
///      `OperationalSpaceControl`'s input ports to corresponding output ports
///      of the trajectory source.
///   5. (if the users added constant trajectories) after creating Diagram,
///      call AssignConstTrajToInputPorts().

class OperationalSpaceControl : public drake::systems::LeafSystem<double> {
 public:
  OperationalSpaceControl(
    RigidBodyTree<double>* tree_w_spr,
    RigidBodyTree<double>* tree_wo_spr);

  // Input/output ports
  const drake::systems::InputPort<double>& get_robot_output_input_port() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_fsm_input_port() const {
    return this->get_input_port(fsm_port_);
  }
  const drake::systems::InputPort<double>& get_tracking_data_input_port(
    std::string name) const {
    return this->get_input_port(traj_name_to_port_index_map_.at(name));
  }

  // Cost methods
  void SetInputCost(Eigen::MatrixXd W) {W_input_ = W;}
  void SetAccelerationCostForAllJoints(Eigen::MatrixXd W) {W_joint_accel_ = W;}
  void AddAccelerationCost(int joint_vel_idx, double w);

  // Constraint methods
  void DisableAcutationConstraint() {with_input_constraints_ = false;}
  void SetContactFriction(double mu) {mu_ = mu;}
  void SetWeightOfSoftContactConstraint(double w_soft_constraint) {
    w_soft_constraint_ = w_soft_constraint;
  }
  void AddContactPoint(int body_index, Eigen::VectorXd pt_on_body);
  void AddContactPoints(std::vector<int> body_index,
                        std::vector<Eigen::VectorXd> pt_on_body);

  // Tracking data methods
  void AddTrackingData(OscTrackingData* tracking_data) {
    tracking_data_vec_->push_back(tracking_data);
  }
  std::vector<OscTrackingData*>* GetAllTrackingData() {
    return tracking_data_vec_.get();
  }
  OscTrackingData* GetTrackingDataByIndex(int index) {
    return tracking_data_vec_->at(index);
  }

  // Osc problem constructor
  void ConstructOSC();

 private:
  // Osc checkers and constructor-related methods
  void CheckCostSettings();
  void CheckConstraintSettings();
  Eigen::VectorXd SolveQp(
    const drake::systems::Context<double>& context, double t,
    int fsm_state, double time_since_last_state_switch,
    Eigen::VectorXd x_w_spr, Eigen::VectorXd x_wo_spr) const;

  // Discrete update that stores the previous state transition time
  drake::systems::EventStatus DiscreteVariableUpdate(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const;

  // Output function
  void CalcOptimalInput(const drake::systems::Context<double>& context,
                        systems::TimestampedVector<double>* control) const;

  // Input/Output ports
  int state_port_;
  int fsm_port_;

  // Discrete update
  int prev_fsm_state_idx_;
  int prev_event_time_idx_;

  // Map position/velocity from model with spring to without spring
  Eigen::MatrixXd map_position_from_spring_to_no_spring_;
  Eigen::MatrixXd map_velocity_from_spring_to_no_spring_;

  // Map from trajectory names to input port indices
  std::map<std::string, int> traj_name_to_port_index_map_;

  // RBT's.
  RigidBodyTree<double>* tree_w_spr_;
  RigidBodyTree<double>* tree_wo_spr_;

  // Size of position, velocity and input of the RBT without spring
  int n_q_;
  int n_v_;
  int n_u_;

  // OSC cost members
  /// Using u cost would push the robot away from the fixed point, so the user
  /// could consider usnig acceleration cost instead.
  Eigen::MatrixXd W_input_;  // Input cost weight
  Eigen::MatrixXd W_joint_accel_;  // Joint acceleration cost weight

  // OSC constraint members
  bool with_input_constraints_ = true;

  // (flat ground) Contact constraints and friction cone cnostraints
  std::vector<int> body_index_ = {};
  std::vector<Eigen::VectorXd> pt_on_body_ = {};
  double mu_ = -1;  // Friction coefficients
  double w_soft_constraint_ = -1;
  // TODO(yminchen): Can you get contact points from RBT???

  // OSC tracking data member (store pointer because of caching)
  std::unique_ptr<std::vector<OscTrackingData*>> tracking_data_vec_ =
        std::make_unique<std::vector<OscTrackingData*>>();
};


}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
