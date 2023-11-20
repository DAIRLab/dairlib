#pragma once

#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <drake/multibody/plant/multibody_plant.h>

#include "common/find_resource.h"
#include "dairlib/lcmt_osc_output.hpp"
#include "dairlib/lcmt_osc_qp_output.hpp"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "solvers/fast_osqp_solver.h"
#include "solvers/solver_options_io.h"
#include "systems/controllers/control_utils.h"
#include "systems/controllers/osc/osc_tracking_data.h"
#include "systems/framework/impact_info_vector.h"
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/solve.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

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
/// the method get_input_port_tracking_data().

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
      bool used_with_finite_state_machine = true);

  /***** Input/output ports *****/

  /*!
   * Output: TimestampedVector<double> that contains the optimal controller
   * input
   */
  const drake::systems::OutputPort<double>& get_output_port_osc_command()
      const {
    return this->get_output_port(osc_output_port_);
  }
  /*!
   * Output: lcmt_osc_debug message that contains various details of each
   * individual osc solve
   */
  const drake::systems::OutputPort<double>& get_output_port_osc_debug() const {
    return this->get_output_port(osc_debug_port_);
  }
  /*!
   * Output: ControllerStatus that contains whether the controller believes it
   * has failed.
   */
  const drake::systems::OutputPort<double>& get_output_port_failure() const {
    return this->get_output_port(failure_port_);
  }

  /*!
   * Input: OutputVector containing the robot state
   */
  const drake::systems::InputPort<double>& get_input_port_robot_output() const {
    return this->get_input_port(state_port_);
  }
  /*!
   * Input: Scalar that determines the current contact mode
   */
  const drake::systems::InputPort<double>& get_input_port_fsm() const {
    return this->get_input_port(fsm_port_);
  }
  /*!
   * Clock is used for specifying phase for periodic trajectories as an
   * alternative to universal time.
   */
  const drake::systems::InputPort<double>& get_input_port_clock() const {
    return this->get_input_port(clock_port_);
  }
  /*!
   * ImpactInfo stores the contact configuration of the near impact event along
   * with a blending constant proportional to the proximity to the impact event
   */
  const drake::systems::InputPort<double>& get_input_port_impact_info() const {
    return this->get_input_port(impact_info_port_);
  }

  /*!
   * Input port for a desired feedforward torque
   */
  const drake::systems::InputPort<double>& get_feedforward_input_port() const {
    if (W_input_.isZero() and fsm_to_w_input_map_.empty()) {
      drake::log()->warn(
          "[DAIRLIB: operational_space_control.cc] accessing feedforward input "
          "port without specifying an input cost."
      );
    }
    return this->get_input_port(ff_input_port_);
  }

  /*!
   * Contains the desired trajectory in the same representation as the target
   * output (OscTrackingData)
   */
  const drake::systems::InputPort<double>& get_input_port_tracking_data(
      const std::string& name) const {
    try {
      return this->get_input_port(traj_name_to_port_index_map_.at(name));
    } catch (std::exception& e) {
      std::cerr << "Cannot find tracking data named: " << name << std::endl;
    }
    return this->get_input_port(0);
  }

  /***** Regularization cost weights *****/

  /*!
   * @brief Add quadratic cost for specific joint effort during state fsm
   * Cost = w * u[joint].T * u[joint]
   */
  void SetInputCostForJointAndFsmStateWeight(const std::string& joint_u_name,
                                             int fsm, double w);
  /*!
   * @brief Add quadratic cost on the magnitude of inputs u
   * Cost = u.T W u
   */
  void SetInputCostWeights(const Eigen::MatrixXd& W) { W_input_ = W; }
  /*!
   * @brief Add quadratic cost on the magnitude of accelerations vdot
   * Cost = vdot.T W vdot
   */
  void SetAccelerationCostWeights(const Eigen::MatrixXd& W) {
    W_joint_accel_ = W;
  }
  /*!
   * @brief Add quadratic cost on the deviation of inputs u from the previous
   * solution. Cost = (u - u_prev).T  W (u - u_prev)
   */
  void SetInputSmoothingCostWeights(const Eigen::MatrixXd& W) {
    W_input_smoothing_ = W;
  }
  /*!
   * @brief Add quadratic cost on the contact constraint violation epsilon
   * Cost = (epsilon).T  W (epsilon)
   */
  void SetContactSoftConstraintWeight(double w_soft_constraint) {
    w_soft_constraint_ = w_soft_constraint;
  }
  /*!
   * @brief Add quadratic cost on the magnitude of the contact forces
   * Cost = (lambda_c).T  W (lambda_c)
   */
  void SetLambdaContactRegularizationWeight(const Eigen::MatrixXd& W) {
    W_lambda_c_reg_ = W;
  }
  /*!
   * @brief Add quadratic cost on the magnitude of the constraint forces
   * Cost = (lambda_h).T  W (lambda_h)
   */
  void SetLambdaHolonomicRegularizationWeight(const Eigen::MatrixXd& W) {
    W_lambda_h_reg_ = W;
  }
  /*!
   * @brief Add linear cost on the deviation from the joint limits
   * Currently unused
   */
  void SetJointLimitWeight(const double w) { w_joint_limit_ = w; }

  // Constraint methods
  void DisableAcutationConstraint() { with_input_constraints_ = false; }
  void SetContactFriction(double mu) { mu_ = mu; }

  void AddContactPoint(const multibody::WorldPointEvaluator<double>* evaluator);
  void AddStateAndContactPoint(
      int state, const multibody::WorldPointEvaluator<double>* evaluator);
  void AddKinematicConstraint(
      const multibody::KinematicEvaluatorSet<double>* evaluators);
  // Tracking data methods
  /// The third argument is used to set a period in which OSC does not track the
  /// desired traj (the period starts when the finite state machine switches to
  /// a new state)
  void AddTrackingData(std::unique_ptr<OscTrackingData> tracking_data,
                       double t_lb = 0,
                       double t_ub = std::numeric_limits<double>::infinity());
  void AddConstTrackingData(
      std::unique_ptr<OscTrackingData> tracking_data, const Eigen::VectorXd& v,
      double t_lb = 0, double t_ub = std::numeric_limits<double>::infinity());
  std::vector<std::unique_ptr<OscTrackingData>>* GetAllTrackingData() {
    return tracking_data_vec_.get();
  }
  OscTrackingData* GetTrackingDataByIndex(int index) {
    return tracking_data_vec_->at(index).get();
  }

  // Optional features
  void SetUpDoubleSupportPhaseBlending(double ds_duration,
                                       int left_support_state,
                                       int right_support_state,
                                       const std::vector<int>& ds_states);
  void SetOsqpSolverOptions(const drake::solvers::SolverOptions& options) {
    solver_options_ = options;
  }
  void SetOsqpSolverOptionsFromYaml(const std::string& yaml_string) {
    SetOsqpSolverOptions(
        drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
            FindResourceOrThrow(yaml_string))
            .GetAsSolverOptions(drake::solvers::OsqpSolver::id())
    );
  };
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
                          double t_since_last_state_switch, double alpha,
                          int next_fsm_state) const;

  // Solves the optimization problem:
  // min_{\lambda} || ydot_{des} - J_{y}(qdot + M^{-1} J_{\lambda}^T \lambda||_2
  // s.t. constraints
  // In the IROS 2021 paper, the problem was unconstrained and could be solved
  // using the closed form least squares solution
  void UpdateImpactInvariantProjection(
      const Eigen::VectorXd& x_w_spr, const Eigen::VectorXd& x_wo_spr,
      const drake::systems::Context<double>& context, double t,
      double t_since_last_state_switch, int fsm_state, int next_fsm_state,
      const Eigen::MatrixXd& M) const;

  // Solves the optimization problem:
  // min_{\lambda} || ydot_{des} - J_{y}(qdot + M^{-1} J_{\lambda}^T \lambda||_2
  // s.t. constraints
  // By adding constraints on lambda, we can impose scaling on the
  // impact-invariant projection.
  // The current constraints are lambda \in convex_hull \alpha * [-FC, FC]
  // defined by the normal impulse from the nominal trajectory
  void UpdateImpactInvariantProjectionQP(
      const Eigen::VectorXd& x_w_spr, const Eigen::VectorXd& x_wo_spr,
      const drake::systems::Context<double>& context, double t,
      double t_since_last_state_switch, int fsm_state, int next_fsm_state,
      const Eigen::MatrixXd& M) const;

  // Discrete update that stores the previous state transition time
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void AssignOscLcmOutput(const drake::systems::Context<double>& context,
                          dairlib::lcmt_osc_output* output) const;

  // Output function
  void CalcOptimalInput(const drake::systems::Context<double>& context,
                        systems::TimestampedVector<double>* control) const;
  // Safety function that triggers when the tracking cost is too high
  void CheckTracking(const drake::systems::Context<double>& context,
                     TimestampedVector<double>* output) const;

  // Input/Output ports
  drake::systems::OutputPortIndex osc_debug_port_;
  drake::systems::OutputPortIndex osc_output_port_;
  drake::systems::OutputPortIndex failure_port_;
  drake::systems::InputPortIndex state_port_;
  drake::systems::InputPortIndex clock_port_;
  drake::systems::InputPortIndex fsm_port_;
  drake::systems::InputPortIndex impact_info_port_;
  drake::systems::InputPortIndex ff_input_port_;

  // Discrete update
  int prev_fsm_state_idx_;
  int prev_event_time_idx_;

  // Map position/velocity from model with spring to without spring
  Eigen::MatrixXd map_position_from_spring_to_no_spring_;
  Eigen::MatrixXd map_velocity_from_spring_to_no_spring_;

  // Map from (non-const) trajectory names to input port indices
  std::map<std::string,
           drake::systems::InputPortIndex> traj_name_to_port_index_map_;

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
  const multibody::KinematicEvaluatorSet<double>* kinematic_evaluators_ = nullptr;

  // robot input limits
  Eigen::VectorXd u_min_;
  Eigen::VectorXd u_max_;

  // robot joint limits
  Eigen::VectorXd q_min_;
  Eigen::VectorXd q_max_;

  // robot joint limits
  Eigen::MatrixXd K_joint_pos_;
  Eigen::MatrixXd K_joint_vel_;

  // flag indicating whether using osc with finite state machine or not
  bool used_with_finite_state_machine_;

  // floating base model flag
  bool is_quaternion_;

  // Solver
  std::unique_ptr<dairlib::solvers::FastOsqpSolver> solver_;
  drake::solvers::SolverOptions solver_options_ =
      drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
          FindResourceOrThrow("solvers/osqp_options_default.yaml"))
          .GetAsSolverOptions(drake::solvers::OsqpSolver::id());

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

  std::vector<drake::solvers::QuadraticCost*> tracking_costs_;
  drake::solvers::QuadraticCost* accel_cost_ = nullptr;
  drake::solvers::LinearCost* joint_limit_cost_ = nullptr;
  drake::solvers::QuadraticCost* input_cost_ = nullptr;
  drake::solvers::QuadraticCost* input_smoothing_cost_ = nullptr;
  drake::solvers::QuadraticCost* lambda_c_cost_ = nullptr;
  drake::solvers::QuadraticCost* lambda_h_cost_ = nullptr;
  drake::solvers::QuadraticCost* soft_constraint_cost_ = nullptr;

  // OSC solution
  std::unique_ptr<Eigen::VectorXd> dv_sol_;
  std::unique_ptr<Eigen::VectorXd> u_sol_;
  std::unique_ptr<Eigen::VectorXd> lambda_c_sol_;
  std::unique_ptr<Eigen::VectorXd> lambda_h_sol_;
  std::unique_ptr<Eigen::VectorXd> epsilon_sol_;
  std::unique_ptr<Eigen::VectorXd> u_prev_;
  mutable double solve_time_;

  mutable Eigen::VectorXd ii_lambda_sol_;
  mutable Eigen::MatrixXd M_Jt_;
  std::map<int, int> active_contact_dim_ = {};

  // OSC cost members
  /// Using u cost would push the robot away from the fixed point, so the user
  /// could consider using acceleration cost instead.
  Eigen::MatrixXd W_input_;        // Input cost weight
  Eigen::MatrixXd W_joint_accel_;  // Joint acceleration cost weight
  Eigen::MatrixXd W_input_smoothing_;
  Eigen::MatrixXd W_lambda_c_reg_;
  Eigen::MatrixXd W_lambda_h_reg_;
  double w_input_smoothing_constraint_ = 1;
  // Joint limit penalty
  double w_joint_limit_ = 0;
  std::map<int, std::pair<int, double>>
      fsm_to_w_input_map_;  // each pair is (joint index, weight)

  // OSC constraint members
  bool with_input_constraints_ = true;
  // Soft contact penalty coefficient and friction cone coefficient
  double mu_ = -1;  // Friction coefficients
  double w_soft_constraint_ = -1;

  // Map finite state machine state to its active contact indices
  std::map<int, std::set<int>> contact_indices_map_ = {};
  // All contacts (used in contact constraints)
  std::vector<const multibody::WorldPointEvaluator<double>*> all_contacts_ = {};
  // single_contact_mode_ is true if there is only 1 contact mode in OSC
  bool single_contact_mode_ = false;

  // OSC tracking data (stored as a pointer because of caching)
  std::unique_ptr<std::vector<std::unique_ptr<OscTrackingData>>>
      tracking_data_vec_ =
          std::make_unique<std::vector<std::unique_ptr<OscTrackingData>>>();

  // Fixed position of constant trajectories
  std::vector<Eigen::VectorXd> fixed_position_vec_;

  // Set a period during which we apply control (Unit: seconds)
  // Let t be the elapsed time since fsm switched to a new state.
  // We only apply the control when t_s <= t <= t_e
  std::vector<double> t_s_vec_;
  std::vector<double> t_e_vec_;

  // Optional feature -- contact force blend
  double ds_duration_ = -1;
  int left_support_state_;
  int right_support_state_;
  std::vector<int> ds_states_;
  double w_blend_constraint_ = 0.1;  // for soft constraint
  mutable double prev_distinct_fsm_state_ = -1;
  drake::solvers::LinearEqualityConstraint* blend_constraint_;
  drake::solvers::VectorXDecisionVariable epsilon_blend_;
};

}  // namespace dairlib::systems::controllers
