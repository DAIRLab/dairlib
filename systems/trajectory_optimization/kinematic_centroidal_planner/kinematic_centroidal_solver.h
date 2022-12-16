#pragma once
#include <drake/lcm/drake_lcm.h>
#include <drake/solvers/ipopt_solver.h>
#include <drake/solvers/mathematical_program.h>

#include "lcm/lcm_trajectory.h"
#include "multibody/kinematic/kinematic_constraints.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "multibody/multipose_visualizer.h"
#include "systems/trajectory_optimization/kinematic_centroidal_planner/kinematic_centroidal_constraints.h"
#include "systems/trajectory_optimization/kinematic_centroidal_planner/kinematic_centroidal_gains.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"

/*!
 * @brief Class for solving nonlinear kinematic centroidal mpc. Implementation
 * is based on Dai, Hongkai, Andres Valenzuela, and Russ Tedrake. “Whole-Body
 * TrajectoryParameters Planning with Centroidal Dynamics and Full Kinematics.”
 * 2014 IEEE-RAS International Conference on Humanoid Robots (November 2014).
 *
 * The optimization contains two coupled problems. The centroidal problem
 * optimizes over: Angular momentum Linear momentum Center of mass position
 *      Contact position
 *      Contact velocity
 *      Contact force
 *  While the kinematics problem optimzes over:
 *      Generalized positions (including floating base)
 *      Generalized velocities (including floating boase)
 *
 * A series of constraints couple the kinematic state to the centroidal state
 * ensuring the contact point's, com position, com velocity, body orientation,
 * body angular velocity all align up
 */
class KinematicCentroidalSolver {
 public:
  /*!
   * @brief Constructor for MPC which initializes decision variables
   * @param plant robot model
   * @param n_knot_points number of knot points
   * @param dt time step
   * @param contact_points vector of world point evaluators which describes the
   * points on the robot which might make contact with the world. This is not
   * the mode sequence
   */
  KinematicCentroidalSolver(
      const drake::multibody::MultibodyPlant<double>& plant,
      const drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
      int n_knot_points,
      double dt,
      const std::vector<dairlib::multibody::WorldPointEvaluator<double>>&
          contact_points);

  /*!
   * @brief Sets the cost for the mpc problem using a gains struct
   * @param gains
   */
  void SetGains(const KinematicCentroidalGains& gains);

  /*!
   * @brief Sets the minimum foot clearance height
   * @param gains
   */
  void SetMinimumFootClearance(const double& swing_foot_clearance) {
    swing_foot_minimum_height_ = swing_foot_clearance;
  }

  /*!
   * @brief Adds a cost reference for the generalized position of the robot of
   * the form (x - x_ref)^T Q (x - x_ref)
   * @param ref_traj trajectory in time
   */
  void SetGenPosReference(
      std::unique_ptr<drake::trajectories::Trajectory<double>> ref_traj);

  /*!
   * @brief Adds a cost reference for the generalized velocity of the robot of
   * the form (x - x_ref)^T Q (x - x_ref)
   * @param ref_traj trajectory in time
   */
  void SetGenVelReference(
      std::unique_ptr<drake::trajectories::Trajectory<double>> ref_traj);

  /*!
   * @brief Adds a cost and reference for the center of mass position of the
   * robot (x - x_ref)^T Q (x - x_ref)
   * @param ref_traj trajectory in time
   */
  void SetComReference(
      std::unique_ptr<drake::trajectories::Trajectory<double>> ref_traj);

  /*!
   * @brief Adds a cost and reference for the contact position and velocity (x -
   * x_ref)^T Q (x - x_ref)
   * @param contact_ref_traj trajectory in time, order is all contact pos, all
   * contact vel
   */
  void SetContactTrackingReference(
      std::unique_ptr<drake::trajectories::Trajectory<double>>
          contact_ref_traj);

  /*!
   * @brief Add a cost and reference for the contact forces (x - x_ref)^T Q (x -
   * x_ref)
   * @param force_ref_traj trajectory in time
   */
  void SetForceTrackingReference(
      std::unique_ptr<drake::trajectories::Trajectory<double>> force_ref_traj);

  /*!
   * @brief Adds a cost and reference for the angular and linear momentum of the
   * robot (x - x_ref)^T Q (x - x_ref)
   * @param ref_traj
   */
  void SetMomentumReference(
      std::unique_ptr<drake::trajectories::Trajectory<double>> ref_traj);

  void SetConstantMomentumReference(const drake::VectorX<double>& value);

  /*!
   * @brief accessor for robot state decision vars
   * @param knotpoint_index
   * @return
   */
  drake::solvers::VectorXDecisionVariable state_vars(int knotpoint_index) const;

  /*!
   * @brief accessor for joint position decision vars
   * @param knotpoint_index
   * @return
   */
  drake::solvers::VectorXDecisionVariable joint_pos_vars(
      int knotpoint_index) const;

  /*!
   * @brief accessor for joint velocity decision vars
   * @param knotpoint_index
   * @return
   */
  drake::solvers::VectorXDecisionVariable joint_vel_vars(
      int knotpoint_index) const;

  /*!
   * @brief accessor for center of mass position decision vars
   * @param knotpoint_index
   * @return
   */
  drake::solvers::VectorXDecisionVariable com_pos_vars(
      int knotpoint_index) const;

  /*!
   * @brief accessor for center of momentum decision variables
   * @param knotpoint_index
   * @return
   */
  drake::solvers::VectorXDecisionVariable momentum_vars(
      int knotpoint_index) const;

  /*!
   * @brief accessor for contact position decision variables
   * @param knotpoint_index
   * @param contact integer corresponding to the contact point based on order of
   * contact points in constructor
   * @return [x,y,z] vector of decision variables for contact 'contact''s
   * position at knotpoint `knotpoint_index`
   */
  drake::solvers::VectorXDecisionVariable contact_pos_vars(
      int knotpoint_index, int contact_index) const;

  /*!
   * @brief accessor for contact velocity decision variables
   * @param knotpoint_index
   * @param contact_index integer corresponding to the contact point based on
   * order of contact points in constructor
   * @return [x,y,z] vector of decision variables for contact 'contact''s
   * velocity at knotpoint `knotpoint_index`
   */
  drake::solvers::VectorXDecisionVariable contact_vel_vars(
      int knotpoint_index, int contact_index) const;

  /*!
   * @brief accessor for contact force decision variables
   * @param knotpoint_index
   * @param contact_index integer corresponding to the contact point based on
   * order of contact points in constructor
   * @return [x,y,z] vector of decision variables for contact 'contact''s force
   * at knotpoint `knotpoint_index`
   */
  drake::solvers::VectorXDecisionVariable contact_force_vars(
      int knotpoint_index, int contact_index) const;

  /*!
   * @brief Adds standard constraints to optimization problem and sets options
   * @param solver_options
   */
  void Build(const drake::solvers::SolverOptions& solver_options);

  /*!
   * @brief Solve the optimization problem and return a piecewise linear
   * trajectory of state
   * @return piecewise linear trajectory of state
   */
  drake::trajectories::PiecewisePolynomial<double> Solve();

  /*!
   *
   * @brief Serializes the solution into a member variable in order to
   *        be published via LCM or saved to a file
   * @param n_knot_points lcm_channel for where to publish
   * @throws exception Solve() has not been called.
   */
  void SerializeSolution(int n_knot_points, double time_offset = 0);

  /*!
   *
   * @brief Generate lcmt_saved_traj
   * @param lcm_channel lcm_channel for where to publish
   * @throws exception Solve() has not been called.
   */
  dairlib::lcmt_saved_traj GenerateLcmTraj(int n_knot_points,
                                           double time_offset = 0);

  /*!
   *
   * @brief Publish the solution on lcm_channel
   * @param lcm_channel lcm_channel for where to publish
   * @throws exception Solve() has not been called.
   */
  void PublishSolution(const std::string& lcm_channel, int n_knot_points);

  /*!
   *
   * @brief Saves the solution at filepath
   * @param filepath relative filepath for where to save the solution
   * @return whether saving was successful
   * @throws exception Solve() has not been called.
   */
  bool SaveSolutionToFile(const std::string& filepath);

  /*!
   *@brief Populate the state and time vector with the solution from result
   */
  void SetFromSolution(const drake::solvers::MathematicalProgramResult& result,
                       Eigen::MatrixXd* state_samples,
                       Eigen::MatrixXd* contact_force_samples,
                       std::vector<double>* time_samples) const;

  /*!
   *@brief Sets initial guess corresponding to zero for everything except
   *quaternions
   */
  void SetZeroInitialGuess();

  void SetRobotStateGuess(const drake::VectorX<double>& state);

  void SetRobotStateGuess(
      const drake::trajectories::PiecewisePolynomial<double>& state_trajectory);

  // TODO remove once drake has trajectory stacking
  void SetRobotStateGuess(
      const drake::trajectories::PiecewisePolynomial<double>& q_traj,
      const drake::trajectories::PiecewisePolynomial<double>& v_traj);

  void SetComPositionGuess(const drake::Vector3<double>& state);

  void SetComPositionGuess(
      const drake::trajectories::PiecewisePolynomial<double>& com_trajectory);

  void SetContactGuess(const drake::trajectories::PiecewisePolynomial<double>&
                           contact_trajectory);

  void SetForceGuess(
      const drake::trajectories::PiecewisePolynomial<double>& force_trajectory);

  void CreateVisualizationCallback(const std::string& model_file, double alpha,
                                   const std::string& weld_frame_to_world = "");

  /*!
   * @brief Add a bounding box constraint to a contact position for all knot
   * points
   * @param contact_index integer corresponding to the contact point based on
   * order of contact points in constructor
   * @param lb lower bound
   * @param ub upper bound
   */
  void AddContactPointPositionConstraint(int contact_index,
                                         const Eigen::Vector3d& lb,
                                         const Eigen::Vector3d& ub);

  /*!
   * @brief Adds joint limits to the joint names included in joints_to_limits
   * using values in the plant
   * @param joints_to_limit vector of strings
   */
  void AddPlantJointLimits(const std::vector<std::string>& joints_to_limit);

  /*!
   * @brief Adds a kinematic position constraint to the optimization
   * @param con a shared pointer to the constraint
   * @param vars the decision variables for the constraint
   */
  void AddKinematicConstraint(
      std::shared_ptr<dairlib::multibody::KinematicPositionConstraint<double>>
          con,
      const Eigen::Ref<const drake::solvers::VectorXDecisionVariable>& vars);

  /*!
   * @brief Adds a constraint on com height to all knot points
   * @param lb
   * @param ub
   */
  void AddComHeightBoundingConstraint(double lb, double ub);

  int num_knot_points() const { return n_knot_points_; }

  /*!
   * @brief Set the mode sequence
   * @param contact_sequence vector where
   * `contact_sequence[knot_point][contact_index]` tells you if at `knot_point`
   * is `contact_index` active
   */
  void SetModeSequence(const std::vector<std::vector<bool>>& contact_sequence);

  /*!
   * @brief Set the mode sequence via a trajectory. The value of the trajectory
   * at each time, cast to a bool is if a contact point is active or not
   * @param contact_sequence
   */
  void SetModeSequence(
      const drake::trajectories::PiecewisePolynomial<double>& contact_sequence);

  void AddInitialStateConstraint(const Eigen::VectorXd& state);

  void UpdateInitialStateConstraint(const Eigen::VectorXd& state);

  const drake::multibody::MultibodyPlant<double>& Plant() { return plant_; };

  /*!
   * @brief Update costs from internally stored variables using current
   * references
   */
  void UpdateCosts();

 private:
  /*!
   * @brief Adds dynamics for centroidal state
   */
  void AddCentroidalDynamics();

  /*!
   * @brief Enforces zero force for feet in flight
   */
  void AddFlightContactForceConstraints();

  /*!
   * @brief Enforce dynamics for kinematics and location of the contacts
   */
  void AddKinematicsIntegrator();

  /*!
   * @brief Feet that in stance are not moving and on the ground, feet in the
   * air are above the ground
   */
  void AddContactConstraints();

  /*!
   * @brief Ensures that contact point for feet line up with kinematics, and
   * centroidal state lines up with kinematic state
   */
  void AddCentroidalKinematicConsistency();

  /*!
   * @brief Ensures feet are not pulling on the ground
   */
  void AddFrictionConeConstraints();

  //  void AddTorqueLimits();
  /*!
   * @brief Get corresponding weight for a knot_point
   */
  double GetKnotpointGain(int knot_point) const;

  /*!
   * @brief Add costs from internally stored variables
   */
  void AddCosts();

  bool is_first_knot(int knot_point_index) const {
    return knot_point_index == 0;
  };
  bool is_last_knot(int knot_point_index) const {
    return knot_point_index == n_knot_points_ - 1;
  };

  const drake::multibody::MultibodyPlant<double>& plant_;
  std::vector<std::unique_ptr<drake::systems::Context<double>>> contexts_;

//  std::unique_ptr<drake::multibody::MultibodyPlant<drake::AutoDiffXd>> plant_ad_;
  const drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad_;
  std::vector<std::unique_ptr<drake::systems::Context<drake::AutoDiffXd>>> contexts_ad_;


  int n_knot_points_;
  double dt_;

  std::vector<dairlib::multibody::WorldPointEvaluator<double>> contact_points_;
  std::vector<dairlib::multibody::KinematicEvaluatorSet<double>> contact_sets_;

  static const int kCentroidalPosDim = 7;
  static const int kCentroidalVelDim = 6;
  int n_q_;
  int n_v_;
  int n_joint_q_;
  int n_joint_v_;
  int n_contact_points_;

  /// References and cost matrixes
  std::unique_ptr<drake::trajectories::Trajectory<double>> q_ref_traj_;
  Eigen::MatrixXd Q_q_;
  std::unique_ptr<drake::trajectories::Trajectory<double>> v_ref_traj_;
  Eigen::MatrixXd Q_v_;
  std::unique_ptr<drake::trajectories::Trajectory<double>> com_ref_traj_;
  Eigen::MatrixXd Q_com_;
  std::unique_ptr<drake::trajectories::Trajectory<double>> mom_ref_traj_;
  Eigen::MatrixXd Q_mom_;
  std::unique_ptr<drake::trajectories::Trajectory<double>> contact_ref_traj_;
  Eigen::MatrixXd Q_contact_;
  std::unique_ptr<drake::trajectories::Trajectory<double>> force_ref_traj_;
  Eigen::MatrixXd Q_force_;

  std::vector<std::vector<bool>> contact_sequence_;
  // MathematicalProgram
  std::unique_ptr<drake::solvers::MathematicalProgram> prog_;
  std::unique_ptr<drake::solvers::IpoptSolver> solver_;
  std::unique_ptr<drake::solvers::MathematicalProgramResult> result_;
  double solve_time_;

  // Constraint bindings
  drake::solvers::BoundingBoxConstraint* init_state_constraint_;
  std::vector<drake::solvers::Binding<drake::solvers::Constraint>>
      centroidal_dynamics_binding_;

  // Cost bindings
  std::vector<std::shared_ptr<drake::solvers::QuadraticCost>> q_ref_cost_;
  std::vector<std::shared_ptr<drake::solvers::QuadraticCost>> v_ref_cost_;
  std::vector<std::shared_ptr<drake::solvers::QuadraticCost>> com_ref_cost_;
  std::vector<std::shared_ptr<drake::solvers::QuadraticCost>> mom_ref_cost_;
  std::vector<std::shared_ptr<drake::solvers::QuadraticCost>> contact_ref_cost_;
  std::vector<std::shared_ptr<drake::solvers::QuadraticCost>> force_ref_cost_;

  // DecisionVariables
  // Full robot state
  std::vector<drake::solvers::VectorXDecisionVariable> x_vars_;
  // angular and linear momentum variables (in that order)
  std::vector<drake::solvers::VectorXDecisionVariable> mom_vars_;
  // center of mass position
  std::vector<drake::solvers::VectorXDecisionVariable> com_vars_;

  // Contact position, velocity, and force [n_knot_points, 3 *
  // n_contact_points]
  std::vector<drake::solvers::VectorXDecisionVariable> contact_pos_;
  std::vector<drake::solvers::VectorXDecisionVariable> contact_vel_;
  std::vector<drake::solvers::VectorXDecisionVariable> contact_force_;


  std::unique_ptr<dairlib::multibody::MultiposeVisualizer> callback_visualizer_;

  const std::set<int> full_constraint_relative_ = {0, 1, 2};

  double swing_foot_minimum_height_;

  // saving and publishing solutions
  std::unique_ptr<drake::lcm::DrakeLcm> lcm_;
  dairlib::LcmTrajectory lcm_trajectory_;
};
