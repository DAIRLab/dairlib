#pragma once

#include <string>
#include <map>
#include <vector>
#include <fstream>
#include <memory>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/systems/framework/leaf_system.h"
#include "src/InEKF.h"

#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"
#include "systems/framework/timestamped_vector.h"
#include "examples/Cassie/datatypes/cassie_out_t.h"
#include "examples/Cassie/cassie_utils.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"

namespace dairlib {
namespace systems {

/// CassieStateEstimator does the following things
/// 1. reads in cassie_out_t,
/// 2. estimates floating-base state and feet contact
/// 3. outputs OutputVector which contains
///    - the state of the robot
///    - the torque feedback
///    - imu accelerometer values
///
/// If the model is fixed-based, then it skips the second step.
///
/// The state estimation part of the program is based on the paper
/// Contact-Aided Invariant Extended Kalman Filtering for Robot State Estimation
/// by Ross Hartley, Maani Ghaffari, Ryan M. Eustice, and Jessy W. Grizzle
///
/// A few notes:
/// - the imu measurements are all in body frame, and the ekf state
///   is expressed in the world frame.
/// - the position and velocity of MBP's floating base coordinates are expressed
///   in the world frame.
/// - we assume the orientation of the imu frame is the same as that of pelvis
///   frame.
class CassieStateEstimator : public drake::systems::LeafSystem<double> {
 public:
  /// Constructor
  /// @param plant MultibodyPlant of the robot
  /// @param test_with_ground_truth_state a flag indicating whether or not the
  /// user is testing the estimated state with the ground-truth state
  /// @param print_info_to_terminal a flag for printing message of EKF to the
  /// terminal
  /// @param hardware_test_mode the mode of ekf
  ///    -1: regular EKF (not a testing mode).
  ///    0: assume both feet are always in contact with ground.
  ///    1: assume both feet are always in the air.
  explicit CassieStateEstimator(
      const drake::multibody::MultibodyPlant<double>& plant,
      const multibody::KinematicEvaluatorSet<double>* fourbar_evaluator,
      const multibody::KinematicEvaluatorSet<double>* left_contact_evaluator,
      const multibody::KinematicEvaluatorSet<double>* right_contact_evaluator,
      bool test_with_ground_truth_state = false,
      bool print_info_to_terminal = false, int hardware_test_mode = -1);
  void solveFourbarLinkage(const Eigen::VectorXd& q_init,
                           double* left_heel_spring,
                           double* right_heel_spring) const;

  /// UpdateContactEstimationCosts() updates the optimal costs of the quadratic
  /// programs, and EstimateContactForEkf() and EstimateContactForController()
  /// estimate the ground contacts based on the optimal costs and spring
  /// deflections.
  /// Estimation from EstimateContactForEkf() is more conservative compared to
  /// EstimateContactForController(). See the cc file for more detail.
  /// The methods are set to be public in order to unit test them.
  void UpdateContactEstimationCosts(
      const systems::OutputVector<double>& output, const double& dt,
      drake::systems::DiscreteValues<double>* discrete_state,
      std::vector<double>* optimal_cost) const;
  void EstimateContactForEkf(
      const systems::OutputVector<double>& output,
      const std::vector<double>&  optimal_cost,
      int* left_contact, int* right_contact) const;
  void EstimateContactForController(
      const systems::OutputVector<double>& output,
      const std::vector<double>&  optimal_cost,
      int* left_contact, int* right_contact) const;

  // Setters for initial values
  void setPreviousTime(drake::systems::Context<double>* context,
                       double time) const;
  void setInitialPelvisPose(drake::systems::Context<double>* context,
                            Eigen::Vector4d quat,
                            Eigen::Vector3d position) const;
  void setPreviousImuMeasurement(drake::systems::Context<double>* context,
                                 const Eigen::VectorXd& imu_value) const;

  // Copy joint state from cassie_out_t to an OutputVector
  void AssignNonFloatingBaseStateToOutputVector(const cassie_out_t& cassie_out,
      systems::OutputVector<double>* output) const;

  // Currently, `DoCalcNextUpdateTime` seems to be the only gateway of adding
  // kTimed events
  void DoCalcNextUpdateTime(
      const drake::systems::Context<double>& context,
      drake::systems::CompositeEventCollection<double>* events,
      double* time) const final;

  // Set the time of the next received message. Is used to trigger update
  // events.
  // Note that the real trigger/update time is `next_message_time_ - eps`,
  // because we want the discrete update to happen before Publish
  void set_next_message_time(double t) { next_message_time_ = t; };

 private:
  void AssignImuValueToOutputVector(const cassie_out_t& cassie_out,
      systems::OutputVector<double>* output) const;
  void AssignActuationFeedbackToOutputVector(const cassie_out_t& cassie_out,
      systems::OutputVector<double>* output) const;
  void AssignFloatingBaseStateToOutputVector(const Eigen::VectorXd& state_est,
      systems::OutputVector<double>* output) const;

  drake::systems::EventStatus Update(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state) const;

  void CopyStateOut(const drake::systems::Context<double>& context,
                    systems::OutputVector<double>* output) const;

  int n_q_;
  int n_v_;
  int n_u_;

  const drake::multibody::MultibodyPlant<double>& plant_;
  const multibody::KinematicEvaluatorSet<double>* fourbar_evaluator_;
  const multibody::KinematicEvaluatorSet<double>* left_contact_evaluator_;
  const multibody::KinematicEvaluatorSet<double>* right_contact_evaluator_;
  const drake::multibody::BodyFrame<double>& world_;
  const bool is_floating_base_;
  std::unique_ptr<drake::systems::Context<double>> context_;

  std::map<std::string, int> position_idx_map_;
  std::map<std::string, int> velocity_idx_map_;
  std::map<std::string, int> actuator_idx_map_;

  // Body frames
  std::vector<const drake::multibody::Frame<double>*> toe_frames_;
  const drake::multibody::Frame<double>& pelvis_frame_;
  const drake::multibody::Body<double>& pelvis_;

  // Input/output port indices
  int cassie_out_input_port_;
  int state_input_port_;

  // Below are indices of system states:
  // A state which stores previous timestamp
  drake::systems::DiscreteStateIndex time_idx_;
  // States related to EKF
  drake::systems::DiscreteStateIndex fb_state_idx_;
  drake::systems::AbstractStateIndex ekf_idx_;
  drake::systems::DiscreteStateIndex prev_imu_idx_;
  // A state related to contact estimation
  // This state store the previous generalized velocity
  drake::systems::DiscreteStateIndex previous_velocity_idx_;
  // States related to contact estimation
  // The states stores the filtered difference between predicted acceleration
  // and measured acceleration (derived by finite differencing)
  drake::systems::DiscreteStateIndex filtered_residual_double_idx_;
  drake::systems::DiscreteStateIndex filtered_residual_left_idx_;
  drake::systems::DiscreteStateIndex filtered_residual_right_idx_;

  // Cassie parameters
  std::vector<
      std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>>
      rod_on_thighs_;
  std::vector<
      std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>>
      rod_on_heel_springs_;
  double rod_length_;
  Eigen::Vector3d front_contact_disp_;
  Eigen::Vector3d rear_contact_disp_;
  Eigen::Vector3d mid_contact_disp_;
  // IMU location wrt pelvis
  Eigen::Vector3d imu_pos_ = Eigen::Vector3d(0.03155, 0, -0.07996);
  Eigen::Vector3d gravity_ = Eigen::Vector3d(0, 0, -9.81);

  // EKF encoder noise
  Eigen::Matrix<double, 16, 16> cov_w_;

  // Contact Estimation Parameters
  // The values of spring threshold are based on walking and standing values in
  // simulation.
  // Walking: https://drive.google.com/open?id=1vMIKAed8RHIFF1fbjTqBHtbgkPrHuzkS
  //          https://drive.google.com/open?id=1UqiZSXhd9-4A6YwHArh-xPBMa16RyUGm
  // Spring deflection of standing:
  //     Knee ~-0.032
  //     Heel ~???
  //          https://drive.google.com/file/d/1o7QS4ZksU91EBIpwtNnKpunob93BKiX_
  //          https://drive.google.com/file/d/1mlDzi0fa-YHopeRHaa-z88fPGuI2Aziv
  const double cost_threshold_ctrl_ = 200;
  const double cost_threshold_ekf_ = 200;
  const double knee_spring_threshold_ctrl_ = -0.015;
  const double knee_spring_threshold_ekf_ = -0.015;
  const double heel_spring_threshold_ctrl_ = -0.03;
  const double heel_spring_threshold_ekf_ = -0.015;
  const double eps_cost_ = 1e-10;  // Avoid indefinite matrix
  const double w_soft_constraint_ = 100;  // Soft constraint cost
  const double alpha_ = 0.9;  // Low-pass filter constant for the acceleration
                              // residual. 0 < alpha_ < 1. The bigger alpha_ is,
                              // the higher the cut-off frequency is.
  // Contact Estimation - Quadratic Programing
  // MathematicalProgram
  std::unique_ptr<drake::solvers::MathematicalProgram> quadprog_;
  // Variable dimensions
  int n_b_;
  int n_cl_;
  int n_cl_active_;
  int n_cr_;
  int n_cr_active_;
  // Cost and constraints (Bindings)
  drake::solvers::LinearEqualityConstraint* fourbar_constraint_;
  drake::solvers::LinearEqualityConstraint* left_contact_constraint_;
  drake::solvers::LinearEqualityConstraint* right_contact_constraint_;
  drake::solvers::LinearEqualityConstraint* imu_accel_constraint_;
  drake::solvers::QuadraticCost* quadcost_eom_;
  drake::solvers::QuadraticCost* quadcost_eps_cl_;
  drake::solvers::QuadraticCost* quadcost_eps_cr_;
  drake::solvers::QuadraticCost* quadcost_eps_imu_;
  // Decision variables
  drake::solvers::VectorXDecisionVariable ddq_;
  drake::solvers::VectorXDecisionVariable lambda_b_;
  drake::solvers::VectorXDecisionVariable lambda_cl_;
  drake::solvers::VectorXDecisionVariable lambda_cr_;
  drake::solvers::VectorXDecisionVariable eps_cl_;
  drake::solvers::VectorXDecisionVariable eps_cr_;
  drake::solvers::VectorXDecisionVariable eps_imu_;

  // flag for testing and tuning
  std::unique_ptr<drake::systems::Context<double>> context_gt_;
  bool test_with_ground_truth_state_;
  bool print_info_to_terminal_;
  int hardware_test_mode_;
  std::unique_ptr<int> counter_for_testing_ =
      std::make_unique<int>(0);

  // Timestamp from unprocessed message
  double next_message_time_ = -std::numeric_limits<double>::infinity();
  double eps_ = 1e-12;
};

}  // namespace systems
}  // namespace dairlib
