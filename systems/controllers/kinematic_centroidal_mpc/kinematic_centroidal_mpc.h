
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
   * @param contact_points vector of world point evaluators which describes the contact points
   */
  KinematicCentroidalMPC(const drake::multibody::MultibodyPlant<double>& plant,
                         int n_knot_points,
                         double dt,
                         const std::vector<std::shared_ptr<dairlib::multibody::WorldPointEvaluator<double>>>& contact_points);


  /*!
   * @brief Adds a cost reference for the state of the robot
   * @param ref_traj trajectory in time
   * @param Q cost on error from the reference
   */
  void AddStateReference(std::unique_ptr<drake::trajectories::Trajectory<double>> ref_traj,
                                      const Eigen::MatrixXd& Q);

  /*!
   * @brief Adds a cost and reference for the centroidal state of the robot
   * @param ref_traj
   * @param Q
   */
  void AddCentroidalReference(std::unique_ptr<drake::trajectories::Trajectory<double>> ref_traj,
                         const Eigen::MatrixXd& Q);

  void AddContactPosTrackingReference(std::unique_ptr<drake::trajectories::Trajectory<double>> contact_ref_traj,
                                     const Eigen::MatrixXd& Q_contact);

  void AddForceTrackingReference(std::unique_ptr<drake::trajectories::Trajectory<double>> force_ref_traj,
                                      const Eigen::MatrixXd& Q_force);

  void AddConstantStateReference(const drake::VectorX<double>& value, const Eigen::MatrixXd& Q);

  void AddConstantForceTrackingReference(const drake::VectorX<double>& value, const Eigen::MatrixXd& Q_force);

  void AddConstantCentroidalReference(const drake::VectorX<double>& value, const Eigen::MatrixXd& Q);

  drake::solvers::VectorXDecisionVariable state_vars(
      int knotpoint_index) const;

   drake::solvers::VectorXDecisionVariable centroidal_pos_vars(
      int knotpoint_index) const;

   drake::solvers::VectorXDecisionVariable centroidal_vel_vars(
      int knotpoint_index) const;

   drake::solvers::VectorXDecisionVariable joint_pos_vars(
      int knotpoint_index) const;

   drake::solvers::VectorXDecisionVariable joint_vel_vars(
      int knotpoint_index) const;

  drake::solvers::VectorXDecisionVariable com_pos_vars(
      int knotpoint_index) const;

  drake::solvers::VectorXDecisionVariable com_vel_vars(
      int knotpoint_index) const;

  drake::solvers::VectorXDecisionVariable cent_quat_vars(
      int knotpoint_index) const;

  drake::solvers::VectorXDecisionVariable cent_omega_vars(
      int knotpoint_index) const;

  drake::solvers::VectorXDecisionVariable contact_pos_vars(
      int knotpoint_index, int contact) const;

  drake::solvers::VectorXDecisionVariable contact_vel_vars(
      int knotpoint_index, int contact) const;

  drake::solvers::VectorXDecisionVariable contact_force_vars(
      int knotpoint_index, int contact) const;

  void Build(const drake::solvers::SolverOptions& solver_options);

  drake::trajectories::PiecewisePolynomial<double> Solve();

  void SetZeroInitialGuess();

  void CreateVisualizationCallback(std::string model_file,
                                   double alpha,
                                   std::string weld_frame_to_world = "");

  void AddContactPointPositionConstraint(int contact_index, const Eigen::Vector3d& lb, const Eigen::Vector3d& ub);

  void AddPlantJointLimits(const std::vector<std::string>& joints_to_limit);

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
   * @brief Enforce dynmaics for kinematics
   */
  void AddKinematicDynamics();

  /*!
   * @brief Ensures that contact point for feet line up with kinematics, and for stance feet are not moving
   */
  void AddContactConstraints();

  void AddCentroidalKinematicConsistency();

  void AddFrictionConeConstraints();

//  void AddTorqueLimits();
//
//  void AddStateLimits();
  void AddCosts();

  const drake::multibody::MultibodyPlant<double>& plant_;

  int n_knot_points_;
  double dt_;

  std::vector<std::shared_ptr<dairlib::multibody::WorldPointEvaluator<double>>> contact_points_;
  std::vector<dairlib::multibody::KinematicEvaluatorSet<double>> contact_sets_;

  const int n_centroidal_pos_ = 7;
  const int n_centroidal_vel_ = 6;
  int n_q_;
  int n_v_;
  int n_kinematic_q_;
  int n_kinematic_v_;
  int n_contact_points_;

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

  std::vector<drake::solvers::Binding<drake::solvers::Constraint>> centroidal_dynamics_binding;

  //DecisionVariables
  // Full robot state
  std::vector<drake::solvers::VectorXDecisionVariable>  x_vars_;
  // Centroidal position and orientation and velocity
  std::vector<drake::solvers::VectorXDecisionVariable>  x_cent_vars_;
  // Contact position index by time, then by contact
  std::vector<std::vector<drake::solvers::VectorXDecisionVariable>>  contact_pos_;
  // Contact velocity index by time, then by contact
  std::vector<std::vector<drake::solvers::VectorXDecisionVariable>>  contact_vel_;
  // Contact force index by time, then by contact
  std::vector<std::vector<drake::solvers::VectorXDecisionVariable>>  contact_force_;

  std::vector<std::unique_ptr<drake::systems::Context<double>>> contexts_;

  std::unique_ptr<dairlib::multibody::MultiposeVisualizer> callback_visualizer_;

};

