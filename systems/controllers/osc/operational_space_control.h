#pragma once

#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/solvers/snopt_solver.h>

#include "dairlib/lcmt_osc_output.hpp"
#include "dairlib/lcmt_osc_qp_output.hpp"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "solvers/fast_osqp_solver.h"
#include "systems/controllers/control_utils.h"
#include "systems/controllers/osc/osc_tracking_data.h"
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/solve.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

// Maximum time limit for each QP solve
static constexpr double kMaxSolveDuration = 0.1;

namespace dairlib::systems::controllers {

/// `OperationalSpaceControl` takes in desired trajectory in world frame and
/// outputs torque command of the motors.

/// Inputs of the constructor:
///  - `plant_w_spr` a MultibodyPlant with springs. If the full model of the
///    plant does not have spring, then plant_w_spr and plant_wo_spr should
///    refer to the same plant.
///  - `plant_wo_spr` a MultibodyPlant without springs
///  - `context_w_spr` a pointer to Context for plant_w_spr
///  - `context_wo_spr` a pointer to Context for plant_wo_spr
///  - `used_with_finite_state_machine` a flag indicating whether using osc with
///    finite state machine or not
/// The springs here refer to the compliant components in the robots.

/// OSC calculates feedback positions/velocities from `plant_w_spr`,
/// but in the optimization it uses `plant_wo_spr`. The reason of using
/// MultibodyPlant without spring is that the OSC cannot track desired
/// acceleration instantaneously when springs exist. (relative degrees of 4)

/// Requirement:
///  - the joints name (except for the spring joints) in `plant_w_spr` must be
///    the same as those of `plant_wo_spr`
///  - the bodies in both MBP's should be the same. (to get Jacobian from
///    both plants)

/// If the robot doesn't have any springs, the user can just pass two identical
/// MultibodyPlants into the constructor.

/// Users define
///     costs,
///     constraints,
///     and the trajectories to track,
/// and add them through `OperationalSpaceControl`'s' methods.

/// Before adding desired trajectories to `OperationalSpaceControl` with the
/// method `AddTrackingData`, users have to create
///     `CenterOfMassTrackingData`,
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
/// The users can connect the output ports of the desired trajectory blocks to
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
  OperationalSpaceControl(
      const drake::multibody::MultibodyPlant<double>& plant_w_spr,
      const drake::multibody::MultibodyPlant<double>& plant_wo_spr,
      drake::systems::Context<double>* context_w_spr,
      drake::systems::Context<double>* context_wo_spr,
      bool used_with_finite_state_machine = true,
      bool print_tracking_info = false, double qp_time_limit = 0,
      bool use_new_qp_setting = false, bool is_rom_modification = false);

  const drake::systems::OutputPort<double>& get_osc_output_port() const {
    return this->get_output_port(osc_output_port_);
  }
  const drake::systems::OutputPort<double>& get_osc_debug_port() const {
    return this->get_output_port(osc_debug_port_);
  }

  // Input/output ports
  const drake::systems::InputPort<double>& get_robot_output_input_port() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_fsm_input_port() const {
    return this->get_input_port(fsm_port_);
  }
  const drake::systems::InputPort<double>& get_near_impact_input_port() const {
    return this->get_input_port(near_impact_port_);
  }
  const drake::systems::InputPort<double>& get_tracking_data_input_port(
      const std::string& name) const {
    try {
      return this->get_input_port(traj_name_to_port_index_map_.at(name));
    } catch (std::exception& e) {
      std::cerr << "Cannot find tracking data named: " << name << std::endl;
    }
    return this->get_input_port(0);
  }

  // Cost methods
  void SetInputCost(const Eigen::MatrixXd& W) { W_input_ = W; }
  void SetAccelerationCostForAllJoints(const Eigen::MatrixXd& W) {
    W_joint_accel_ = W;
  }
  void AddAccelerationCost(const std::string& joint_vel_name, double w);

  // Constraint methods
  void DisableAcutationConstraint() { with_input_constraints_ = false; }
  void SetContactFriction(double mu) { mu_ = mu; }
  void SetWeightOfSoftContactConstraint(double w_soft_constraint) {
    w_soft_constraint_ = w_soft_constraint;
  }
  void AddContactPoint(const multibody::WorldPointEvaluator<double>* evaluator);
  void AddStateAndContactPoint(
      int state, const multibody::WorldPointEvaluator<double>* evaluator);
  void AddKinematicConstraint(
      const multibody::KinematicEvaluatorSet<double>* evaluators);
  // Tracking data methods
  /// The third argument is used to set a period in which OSC does not track the
  /// desired traj (the period starts when the finite state machine switches to
  /// a new state)
  void AddTrackingData(OscTrackingData* tracking_data, double t_lb = 0,
                       double t_ub = std::numeric_limits<double>::infinity());
  void AddConstTrackingData(
      OscTrackingData* tracking_data, const Eigen::VectorXd& v, double t_lb = 0,
      double t_ub = std::numeric_limits<double>::infinity());
  std::vector<OscTrackingData*>* GetAllTrackingData() {
    return tracking_data_vec_.get();
  }
  OscTrackingData* GetTrackingDataByIndex(int index) {
    return tracking_data_vec_->at(index);
  }

  // Optional features
  void SetUpDoubleSupportPhaseBlending(double ds_duration,
                                       int left_support_state,
                                       int right_support_state,
                                       std::vector<int> ds_states);
  void SetInputRegularizationWeight(double w) { w_input_reg_ = w; }
  void SetInputRegularizationWeights(const Eigen::MatrixXd& W) {
    W_input_reg_ = W;
  }
  void AddContactForceRegularizationCostForOptimalROM(double w) {
    w_rom_force_reg_ = w;
  }
  void SetLambdaContactRegularizationWeight(const Eigen::MatrixXd& W) {
    W_lambda_c_reg_ = W;
  }
  void SetLambdaHolonomicRegularizationWeight(const Eigen::MatrixXd& W) {
    W_lambda_h_reg_ = W;
  }
  void SetVdotRegularizationWeight(const Eigen::MatrixXd& W) {
    W_vdot_reg_ = W;
  }

  // OSC LeafSystem builder
  void Build();

 private:
  // Osc checkers and constructor-related methods
  void CheckCostSettings();
  void CheckConstraintSettings();

  // Get solution of OSC
  Eigen::VectorXd SolveQp(const Eigen::VectorXd& x_w_spr,
                          const Eigen::VectorXd& x_wo_spr,
                          const drake::systems::Context<double>& context,
                          double t, int fsm_state,
                          double time_since_last_state_switch, double alpha,
                          int next_fsm_state) const;

  void UpdateImpactInvariantProjection(
      const Eigen::VectorXd& x_w_spr, const Eigen::VectorXd& x_wo_spr,
      const drake::systems::Context<double>& context, double t,
      double t_since_last_state_switch, int fsm_state, int next_fsm_state,
      const Eigen::MatrixXd& M, const Eigen::MatrixXd& J_h) const;

  // Discrete update that stores the previous state transition time
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void AssignOscLcmOutput(const drake::systems::Context<double>& context,
                          dairlib::lcmt_osc_output* output) const;

  // Output function
  void CalcOptimalInput(const drake::systems::Context<double>& context,
                        systems::TimestampedVector<double>* control) const;

  // Input/Output ports
  int osc_debug_port_;
  int osc_output_port_;
  int state_port_;
  int fsm_port_;
  int near_impact_port_;

  // Discrete update
  int prev_fsm_state_idx_;
  int prev_event_time_idx_;

  // Map position/velocity from model with spring to without spring
  Eigen::MatrixXd map_position_from_spring_to_no_spring_;
  Eigen::MatrixXd map_velocity_from_spring_to_no_spring_;

  // Map from (non-const) trajectory names to input port indices
  std::map<std::string, int> traj_name_to_port_index_map_;

  // MBP's.
  const drake::multibody::MultibodyPlant<double>& plant_w_spr_;
  const drake::multibody::MultibodyPlant<double>& plant_wo_spr_;

  // MBP context's
  drake::systems::Context<double>* context_w_spr_;
  drake::systems::Context<double>* context_wo_spr_;

  // World frames
  const drake::multibody::BodyFrame<double>& world_w_spr_;
  const drake::multibody::BodyFrame<double>& world_wo_spr_;

  // Size of position, velocity and input of the MBP without spring
  int n_q_;
  int n_v_;
  int n_u_;
  int n_revolute_joints_;

  // Size of holonomic constraint and total/active contact constraints
  int n_h_;
  int n_c_;
  int n_c_active_;

  // Manually specified holonomic constraints (only valid for plants_wo_springs)
  const multibody::KinematicEvaluatorSet<double>* kinematic_evaluators_;

  // robot input limits
  Eigen::VectorXd u_min_;
  Eigen::VectorXd u_max_;

  // robot joint limits
  Eigen::VectorXd q_min_;
  Eigen::VectorXd q_max_;

  // robot joint limits
  Eigen::MatrixXd K_joint_pos;
  Eigen::MatrixXd K_joint_vel;

  // flag indicating whether using osc with finite state machine or not
  bool used_with_finite_state_machine_;

  // flag indicating whether to print the tracking related values or not
  bool print_tracking_info_;

  // floating base model flag
  bool is_quaternion_;

  // Solver
  std::unique_ptr<solvers::FastOsqpSolver> solver_;

  ///////////////////// My own ROM branch's modification ///////////////////////
  bool is_rom_modification_;

  // Testing
  drake::solvers::SolverOptions solver_options_;
  std::unique_ptr<drake::solvers::OsqpSolver> osqp_solver_;
  std::unique_ptr<drake::solvers::SnoptSolver> snopt_solver_;
  mutable Eigen::VectorXd prev_sol_ = Eigen::VectorXd::Zero(1);
  mutable int counter_ = 0;
  mutable int prev_fsm_state_ = -1;
  bool use_osqp_ = true;

  drake::solvers::QuadraticCost* reg_cost_;
  Eigen::MatrixXd W_reg_;
  Eigen::MatrixXd W_reg_0_;

  // testing
  bool use_new_qp_setting_;

  // Testing
  std::vector<int> knee_ankle_pos_idx_list_;
  std::vector<int> knee_ankle_vel_idx_list_;

  // Testing -- translating knee spring deflection to knee joint
  bool two_models_;
  std::vector<int> knee_ankle_pos_idx_list_wo_spr_;
  std::vector<int> spring_pos_idx_list_w_spr_;
  std::vector<int> knee_ankle_vel_idx_list_wo_spr_;
  std::vector<int> spring_vel_idx_list_w_spr_;
  //////////////////////////////////////////////////////////////////////////////

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
  std::vector<drake::solvers::LinearCost*> joint_limit_cost_;

  // OSC solution
  std::unique_ptr<Eigen::VectorXd> dv_sol_;
  std::unique_ptr<Eigen::VectorXd> u_sol_;
  std::unique_ptr<Eigen::VectorXd> lambda_c_sol_;
  std::unique_ptr<Eigen::VectorXd> lambda_h_sol_;
  std::unique_ptr<Eigen::VectorXd> epsilon_sol_;
  mutable double solve_time_;

  mutable Eigen::VectorXd ii_lambda_sol_;
  mutable Eigen::MatrixXd M_Jt_;
  std::map<int, int> active_contact_dim_ = {};

  // OSC cost members
  /// Using u cost would push the robot away from the fixed point, so the user
  /// could consider using acceleration cost instead.
  Eigen::MatrixXd W_input_;        // Input cost weight
  Eigen::MatrixXd W_joint_accel_;  // Joint acceleration cost weight

  // OSC constraint members
  bool with_input_constraints_ = true;

  // Soft contact penalty coefficient and friction cone coefficient
  double mu_ = -1;  // Friction coefficients
  double w_soft_constraint_ = -1;

  // Joint limit penalty
  Eigen::VectorXd w_joint_limit_;

  // Map finite state machine state to its active contact indices
  std::map<int, std::set<int>> contact_indices_map_ = {};
  // All contacts (used in contact constraints)
  std::vector<const multibody::WorldPointEvaluator<double>*> all_contacts_ = {};
  // single_contact_mode_ is true if there is only 1 contact mode in OSC
  bool single_contact_mode_ = false;

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

  // Maximum time limit for each QP solve
  const double qp_time_limit_;

  // Optional feature -- contact force blend
  double ds_duration_ = -1;
  int left_support_state_;
  int right_support_state_;
  std::vector<int> ds_states_;
  double w_blend_constraint_ = 0.1;  // for soft constraint
  mutable double prev_distinct_fsm_state_ = -1;
  drake::solvers::LinearEqualityConstraint* blend_constraint_;
  drake::solvers::VectorXDecisionVariable epsilon_blend_;

  // Optional feature -- regularizing input
  drake::solvers::QuadraticCost* input_reg_cost_;
  double w_input_reg_ = -1;
  Eigen::MatrixXd W_input_reg_;

  // Optional feature -- regularzing forces
  drake::solvers::QuadraticCost* lambda_c_cost_ = nullptr;
  drake::solvers::QuadraticCost* lambda_h_cost_ = nullptr;
  drake::solvers::QuadraticCost* vdot_reg_cost_ = nullptr;
  Eigen::MatrixXd W_lambda_c_reg_;
  Eigen::MatrixXd W_lambda_h_reg_;
  Eigen::MatrixXd W_vdot_reg_;

  // Optimal feature -- contact force regularization for optimal ROM
  // (penalize the front contact for optimal model as a regularization term, in
  //  order to help the solver to find good solution.)
  double w_rom_force_reg_ = -1;
  drake::solvers::QuadraticCost* contact_force_reg_cost_;
};

}  // namespace dairlib::systems::controllers
