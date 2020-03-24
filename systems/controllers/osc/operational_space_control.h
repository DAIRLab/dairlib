#pragma once

#include <utility>
#include <vector>
#include <string>
#include <memory>
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"

#include "systems/framework/output_vector.h"
#include "systems/controllers/osc/osc_tracking_data.h"
#include "systems/controllers/control_utils.h"

namespace dairlib {
namespace systems {

namespace controllers {

/// `OperationalSpaceControl` takes in desired trajectory in world frame and
/// outputs torque command of the motors.

/// Inputs of the constructor:
///  - `tree_w_spr` a RigidBodyTree with springs
///  - `tree_wo_spr` a RigidBodyTree without springs
///  - `used_with_finite_state_machine` a flag indicating whehter using osc with
///    finite state machine or not
/// The springs here refer to the compliant components in the robots.

/// OSC calculates feedback positions/velocities from `tree_w_spr`,
/// but in the optimization it uses `tree_wo_spr`. The reason of using
/// RigidBodyTree without spring is that the OSC cannot track desired
/// acceleration instantaneously when springs exist. (relative degrees of 4)

/// Requirement:
///  - the joints name (except for the spring joints) in `tree_w_spr` must be
///    the same as those of `tree_wo_spr`
///  - the bodies in both RBT's should be the same. (to get jacobian from
///    both trees)

/// If the robot doesn't have any spring, the user can just pass two identical
/// RigidBodyTree into the constructor.

/// Users define
///     costs,
///     constraints,
///     and the trajectories to track,
/// and add them through `OperationalSpaceControl`'s' methods.

/// Before adding desired trajectories to `OperationalSpaceControl` with the
/// method `AddTrackingData`, users have to create
///     `TransTaskSpaceTrackingData`,
///     `RotTaskSpaceTrackingData`,
///     and/or `JointSpaceTrackingData`.

/// If the desired trajectory is constant, users don't need to connect the
/// input ports of `OperationalSpaceControl` to trajectory source blocks.
/// Instead, the users have to call the function AddConstTrackingData() when
/// adding TrackingData to OperationalSpaceControl.

/// If the users want to create the desired trajectory source themselves,
/// the outputs of trajectory blocks need to be of the derived classes of
///     drake::trajectories::Trajectory<double>
/// such as `PiecewisePolynomial` and `ExponentialPlusPiecewisePolynomial`.
/// The users can connect the output ports of the desired trajecotry blocks to
/// the corresponding input ports of `OperationalSpaceControl` by using
/// the method get_tracking_data_input_port().

/// The procedure of setting up `OperationalSpaceControl`:
///   1. create an instance of `OperationalSpaceControl`
///   2. add costs/constraints/desired trajectories
///   3. call Build()
///   4. (if the users created desired trajectory blocks by themselves) connect
///      `OperationalSpaceControl`'s input ports to corresponding output ports
///      of the trajectory source.

class OperationalSpaceControl : public drake::systems::LeafSystem<double> {
 public:
  OperationalSpaceControl(const RigidBodyTree<double>& tree_w_spr,
                          const RigidBodyTree<double>& tree_wo_spr,
                          bool used_with_finite_state_machine = true,
                          bool print_tracking_info = false);

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
  void AddAccelerationCost(std::string joint_vel_name, double w);

  // Constraint methods
  void DisableAcutationConstraint() {with_input_constraints_ = false;}
  void SetContactFriction(double mu) {mu_ = mu;}
  void SetWeightOfSoftContactConstraint(double w_soft_constraint) {
    w_soft_constraint_ = w_soft_constraint;
  }
  void AddContactPoint(std::string body_name, Eigen::VectorXd pt_on_body);
  void AddStateAndContactPoint(int state,
                               std::string body_name,
                               Eigen::VectorXd pt_on_body);

  // Tracking data methods
  /// The third argument is used to set a period in which OSC does not track the
  /// desired traj (the period starts when the finite state machine switches to
  /// a new state)
  void AddTrackingData(OscTrackingData* tracking_data,
      double t_lb = 0, double t_ub = std::numeric_limits<double>::infinity());
  void AddConstTrackingData(OscTrackingData* tracking_data, Eigen::VectorXd v,
      double t_lb = 0, double t_ub = std::numeric_limits<double>::infinity());
  std::vector<OscTrackingData*>* GetAllTrackingData(){
    return tracking_data_vec_.get();
  }
  OscTrackingData* GetTrackingDataByIndex(int index) {
    return tracking_data_vec_->at(index);
  }

  // Osc leafsystem builder
  void Build();

 private:
  // Osc checkers and constructor-related methods
  void CheckCostSettings();
  void CheckConstraintSettings();

  // Get solution of OSC
  Eigen::VectorXd SolveQp(Eigen::VectorXd x_w_spr, Eigen::VectorXd x_wo_spr,
      const drake::systems::Context<double>& context, double t,
      int fsm_state, double time_since_last_state_switch) const;

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

  // Map from (non-const) trajectory names to input port indices
  std::map<std::string, int> traj_name_to_port_index_map_;

  // RBT's.
  const RigidBodyTree<double>& tree_w_spr_;
  const RigidBodyTree<double>& tree_wo_spr_;

  // Size of position, velocity and input of the RBT without spring
  int n_q_;
  int n_v_;
  int n_u_;

  // Size of holonomic constraint and total contact constraints
  int n_h_;
  int n_c_;

  // robot input limits
  Eigen::VectorXd u_min_;
  Eigen::VectorXd u_max_;

  // flag indicating whehter using osc with finite state machine or not
  bool used_with_finite_state_machine_;

  // flag indicating whether to print the tracking related values or not
  bool print_tracking_info_;

  // floating base model flag
  bool is_quaternion_;

  // MathematicalProgram
  std::unique_ptr<drake::solvers::MathematicalProgram> prog_;
  // Decision variables
  drake::solvers::VectorXDecisionVariable dv_;
  drake::solvers::VectorXDecisionVariable u_;
  drake::solvers::VectorXDecisionVariable lambda_c_;
  drake::solvers::VectorXDecisionVariable lambda_h_;
  drake::solvers::VectorXDecisionVariable epsilon_;
  // Cost and constraints
  drake::solvers::LinearEqualityConstraint* dynamics_constraint_;
  drake::solvers::LinearEqualityConstraint* holonomic_constraint_;
  drake::solvers::LinearEqualityConstraint* contact_constraints_;
  std::vector<drake::solvers::LinearConstraint*> friction_constraints_;
  std::vector<drake::solvers::QuadraticCost*> tracking_cost_;

  // OSC cost members
  /// Using u cost would push the robot away from the fixed point, so the user
  /// could consider usnig acceleration cost instead.
  Eigen::MatrixXd W_input_;  // Input cost weight
  Eigen::MatrixXd W_joint_accel_;  // Joint acceleration cost weight

  // OSC constraint members
  bool with_input_constraints_ = true;

  // (flat ground) Contact constraints and friction cone constraints
  std::vector<int> body_index_ = {};
  std::vector<Eigen::VectorXd> pt_on_body_ = {};
  double mu_ = -1;  // Friction coefficients
  double w_soft_constraint_ = -1;

  // `fsm_state_when_active_` is the finite state machine state when the contact
  // constraint is active. If `fsm_state_when_active_` is empty, then the
  // constraint is always active.
  // The states here can repeat, since there might be multiple contact points
  // in a state of the finite state machine.
  std::vector<int> fsm_state_when_active_;

  // CalcActiveContactIndices gives a vector of flags indicating the active
  // contact (constraint)
  std::vector<bool> CalcActiveContactIndices(int fsm_state) const;

  // OSC tracking data (stored as a pointer because of caching)
  std::unique_ptr<std::vector<OscTrackingData*>> tracking_data_vec_ =
      std::make_unique<std::vector<OscTrackingData*>>();
  // Fixed position of constant trajectories
  std::vector<Eigen::VectorXd> fixed_position_vec_;

  // Set a period during which we apply control (Unit: seconds)
  // Let t be the elapsed time since fsm switched to a new state.
  // We only apply the control when t_s <= t <= t_e
  std::vector<double> t_s_vec_;
  std::vector<double> t_e_vec_;
};


}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
