
#pragma once
#include <drake/solvers/mathematical_program.h>
#include <drake/solvers/ipopt_solver.h>
#include "multibody/kinematic/world_point_evaluator.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "multibody/multipose_visualizer.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "systems/controllers/kinematic_centroidal_mpc/kinematic_centroidal_constraints.h"
#include "multibody/kinematic/kinematic_constraints.h"

/*!
 * @brief Class for solving nonlinear kinematic centroidal mpc. Implementation is based on
 * Dai, Hongkai, Andres Valenzuela, and Russ Tedrake. “Whole-Body Motion Planning
 * with Centroidal Dynamics and Full Kinematics.” 2014 IEEE-RAS International Conference on
 * Humanoid Robots (November 2014).
 *
 * The optimization contains two coupled problems. The centroidal problem optimizes over:
 *      Body orientation
 *      Center of mass position
 *      Body angular velocity
 *      Center of mass velocity
 *      Contact position
 *      Contact velocity
 *      Contact force
 *  While the kinematics problem optimzes over:
 *      Robot state (positions and velocity, including floating base)
 *
 * A series of constraints couple the kinematic state to the centroidal state ensuring the contact point's, com position,
 * com velocity, body orientation, body angular velocity all align up
 */
class KinematicCentroidalMPC {
 public:
  /*!
   * @brief Constructor for MPC which initializes decision variables
   * @param plant robot model
   * @param n_knot_points number of knot points
   * @param dt time step
   * @param contact_points vector of world point evaluators which describes the points on the robot which might make contact with
   *                        the world. This is not the mode sequence
   */
  KinematicCentroidalMPC(const drake::multibody::MultibodyPlant<double>& plant,
                         int n_knot_points,
                         double dt,
                         const std::vector<std::shared_ptr<dairlib::multibody::WorldPointEvaluator<double>>>& contact_points);


  /*!
   * @brief Adds a cost reference for the state of the robot of the form (x - x_ref)^T Q (x - x_ref)
   * @param ref_traj trajectory in time
   * @param Q cost on error from the reference
   */
  void AddStateReferenceCost(std::unique_ptr<drake::trajectories::Trajectory<double>> ref_traj,
                             const Eigen::MatrixXd& Q);

  /*!
   * @brief Adds a cost and reference for the centroidal state of the robot (x - x_ref)^T Q (x - x_ref)
   * @param ref_traj trajectory in time
   * @param Q cost on error from reference
   */
  void AddCentroidalReferenceCost(std::unique_ptr<drake::trajectories::Trajectory<double>> ref_traj,
                                  const Eigen::MatrixXd& Q);

  /*!
   * @brief Adds a cost and reference for the contact position and velocity (x - x_ref)^T Q (x - x_ref)
   * @param contact_ref_traj trajectory in time
   * @param Q_contact cost on error from the reference
   */
  void AddContactTrackingReferenceCost(std::unique_ptr<drake::trajectories::Trajectory<double>> contact_ref_traj,
                                       const Eigen::MatrixXd& Q_contact);

  /*!
   * @brief Add a cost and reference for the contact forces (x - x_ref)^T Q (x - x_ref)
   * @param force_ref_traj trajectory in time
   * @param Q_force cost on error from the reference
   */
  void AddForceTrackingReferenceCost(std::unique_ptr<drake::trajectories::Trajectory<double>> force_ref_traj,
                                     const Eigen::MatrixXd& Q_force);

  void AddConstantStateReferenceCost(const drake::VectorX<double>& value, const Eigen::MatrixXd& Q);

  void AddConstantForceTrackingReferenceCost(const drake::VectorX<double>& value, const Eigen::MatrixXd& Q_force);

  void AddConstantCentroidalReferenceCost(const drake::VectorX<double>& value, const Eigen::MatrixXd& Q);

  /*!
   * @brief accessor for robot state decision vars
   * @param knotpoint_index
   * @return
   */
  drake::solvers::VectorXDecisionVariable state_vars(
      int knotpoint_index) const;

  /*!
   * @brief accessor for centroidal position decision vars
   * @param knotpoint_index
   * @return
   */
   drake::solvers::VectorXDecisionVariable centroidal_pos_vars(
      int knotpoint_index) const;

   /*!
    * @brief accessor for centroidal velocity decision vars
    * @param knotpoint_index
    * @return
    */
   drake::solvers::VectorXDecisionVariable centroidal_vel_vars(
      int knotpoint_index) const;

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
   * @brief accessor for center of mass velocity decision vars
   * @param knotpoint_index
   * @return
   */
  drake::solvers::VectorXDecisionVariable com_vel_vars(
      int knotpoint_index) const;

  /*!
   * @brief accessor for centroidal quaternion decision vars
   * @param knotpoint_index
   * @return
   */
  drake::solvers::VectorXDecisionVariable cent_quat_vars(
      int knotpoint_index) const;

  /*!
   * @brief accessor for com angular velocity decision variables (body frame)
   * @param knotpoint_index
   * @return
   */
  drake::solvers::VectorXDecisionVariable cent_omega_vars(
      int knotpoint_index) const;

  /*!
   * @brief accessor for contact position decision variables
   * @param knotpoint_index
   * @param contact integer corresponding to the contact point based on order of contact points in constructor
   * @return [x,y,z] vector of decision variables for contact 'contact''s position at knotpoint `knotpoint_index`
   */
  drake::solvers::VectorXDecisionVariable contact_pos_vars(
      int knotpoint_index, int contact_index) const;

  /*!
   * @brief accessor for contact velocity decision variables
   * @param knotpoint_index
   * @param contact_index integer corresponding to the contact point based on order of contact points in constructor
   * @return [x,y,z] vector of decision variables for contact 'contact''s velocity at knotpoint `knotpoint_index`
   */
  drake::solvers::VectorXDecisionVariable contact_vel_vars(
      int knotpoint_index, int contact_index) const;

  /*!
   * @brief accessor for contact force decision variables
   * @param knotpoint_index
   * @param contact_index integer corresponding to the contact point based on order of contact points in constructor
   * @return [x,y,z] vector of decision variables for contact 'contact''s force at knotpoint `knotpoint_index`
   */
  drake::solvers::VectorXDecisionVariable contact_force_vars(
      int knotpoint_index, int contact_index) const;

  /*!
   * @brief Adds standard constraints to optimization problem and sets options
   * @param solver_options
   */
  void Build(const drake::solvers::SolverOptions& solver_options);

  /*!
   * @brief Solve the optimization problem and return a piecewise linear trajectory of state
   * @return piecewise linear trajectory of state
   */
  drake::trajectories::PiecewisePolynomial<double> Solve();

  /*!
   *@brief Sets initial guess corresponding to zero for everything except quaternions
   */
  void SetZeroInitialGuess();

  void SetRobotStateGuess(const drake::VectorX<double>& state);

  void CreateVisualizationCallback(std::string model_file,
                                   double alpha,
                                   std::string weld_frame_to_world = "");

  /*!
   * @brief Add a bounding box constraint to a contact position for all knot points
   * @param contact_index integer corresponding to the contact point based on order of contact points in constructor
   * @param lb lower bound
   * @param ub upper bound
   */
  void AddContactPointPositionConstraint(int contact_index, const Eigen::Vector3d& lb, const Eigen::Vector3d& ub);

  /*!
   * @brief Adds joint limits to the joint names included in joints_to_limits using values in the plant
   * @param joints_to_limit vector of strings
   */
  void AddPlantJointLimits(const std::vector<std::string>& joints_to_limit);

  void AddKinematicConstraint(std::shared_ptr<dairlib::multibody::KinematicPositionConstraint<double>> con,
                              const Eigen::Ref<const drake::solvers::VectorXDecisionVariable>& vars);
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
   * @brief Feet that in stance are not moving and on the ground, feet in the air are above the ground
   */
  void AddContactConstraints();

  /*!
 * @brief Ensures that contact point for feet line up with kinematics, and centroidal state lines up with kinematic state
 */
  void AddCentroidalKinematicConsistency();

  /*!
   * @brief Ensures feet are not pulling on the ground
   */
  void AddFrictionConeConstraints();

//  void AddTorqueLimits();
//
//  void AddStateLimits();

  /*!
   * @brief Add costs from internally stored variables
   */
  void AddCosts();

  const drake::multibody::MultibodyPlant<double>& plant_;

  int n_knot_points_;
  double dt_;

  std::vector<std::shared_ptr<dairlib::multibody::WorldPointEvaluator<double>>> contact_points_;
  std::vector<dairlib::multibody::KinematicEvaluatorSet<double>> contact_sets_;

  static const int kCentroidalPosDim = 7;
  static const int kCentroidalVelDim = 6;
  int n_q_;
  int n_v_;
  int n_joint_q_;
  int n_joint_v_;
  int n_contact_points_;

  /// References and cost matrixes
  std::unique_ptr<drake::trajectories::Trajectory<double>> ref_traj_;
  Eigen::MatrixXd Q_;
  std::unique_ptr<drake::trajectories::Trajectory<double>> centroidal_ref_traj_;
  Eigen::MatrixXd Q_cent_;
  std::unique_ptr<drake::trajectories::Trajectory<double>> contact_ref_traj_;
  Eigen::MatrixXd Q_contact_;
  std::unique_ptr<drake::trajectories::Trajectory<double>> force_ref_traj_;
  Eigen::MatrixXd Q_force_;


  std::vector<std::unordered_map<std::shared_ptr<dairlib::multibody::WorldPointEvaluator<double>>, bool>> contact_sequence_;
  // MathematicalProgram
  std::unique_ptr<drake::solvers::MathematicalProgram> prog_;
  std::unique_ptr<drake::solvers::IpoptSolver> solver_;

  std::vector<drake::solvers::Binding<drake::solvers::Constraint>> centroidal_dynamics_binding_;

  //DecisionVariables
  // Full robot state
  std::vector<drake::solvers::VectorXDecisionVariable>  x_vars_;
  // Centroidal position and orientation and velocity
  std::vector<drake::solvers::VectorXDecisionVariable>  x_cent_vars_;

  // Contact position, velocity, and force [n_knot_points, 3 * n_contact_points]
  std::vector<drake::solvers::VectorXDecisionVariable>  contact_pos_;
  std::vector<drake::solvers::VectorXDecisionVariable>  contact_vel_;
  std::vector<drake::solvers::VectorXDecisionVariable>  contact_force_;

  std::vector<std::unique_ptr<drake::systems::Context<double>>> contexts_;

  std::unique_ptr<dairlib::multibody::MultiposeVisualizer> callback_visualizer_;

  const std::set<int> full_constraint_relative_ = {0, 1, 2};
};

