#pragma once

#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <drake/lcmt_contact_results_for_viz.hpp>

#include "dairlib/lcmt_contact.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/datatypes/cassie_out_t.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/multibody_utils.h"
#include "src/InEKF.h"
#include "systems/framework/output_vector.h"
#include "systems/framework/timestamped_vector.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/systems/framework/leaf_system.h"

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
  /// @param test_with_ground_truth_state a flag indicating whether or not thel
  /// user is testing the estimated state with the ground-truth state
  /// @param print_info_to_terminal a flag for printing message of EKF to the
  /// terminal
  /// @param hardware_test_mode the mode of ekf
  ///    -1: regular EKF (not a testing mode).
  ///    0: assume both feet are always in contact with ground.
  ///    1: assume both feet are always in the air.
  explicit CassieStateEstimator(
      const drake::multibody::MultibodyPlant<double>& plant,
      std::map<std::string, double> joint_offset_map = {},
      bool test_with_ground_truth_state = false, int hardware_test_mode = -1);

  const drake::systems::OutputPort<double>& get_robot_output_port() const {
    return this->get_output_port(estimated_state_output_port_);
  }
  const drake::systems::OutputPort<double>& get_contact_output_port() const {
    return this->get_output_port(contact_output_port_);
  }
  const drake::systems::OutputPort<double>& get_covariance_output_port() const {
    return this->get_output_port(pose_covariance_output_port_);
  }

  void solveFourbarLinkage(const Eigen::VectorXd& q_init,
                           double* left_heel_spring,
                           double* right_heel_spring) const;

  /// EstimateContactForEkf() and EstimateContactForController()
  /// estimate the ground contacts based on the optimal costs and spring
  /// deflections.
  /// Estimation from EstimateContactForEkf() is more conservative compared to
  /// EstimateContactForController(). See the cc file for more detail.
  /// The methods are set to be public in order to unit test them.

  void EstimateContactForEkf(const systems::OutputVector<double>& output,
                             int* left_contact, int* right_contact) const;

  // Setters for initial values
  void setPreviousTime(drake::systems::Context<double>* context,
                       double time) const;

  void setInitialPelvisPose(
      drake::systems::Context<double>* context, Eigen::Vector4d quat,
      Eigen::Vector3d position,
      Eigen::Vector3d pelvis_vel = Eigen::Vector3d::Zero()) const;
  void setPreviousImuMeasurement(drake::systems::Context<double>* context,
                                 const Eigen::VectorXd& imu_value) const;

  void SetSpringDeflectionThresholds(double knee_spring_threshold, double ankle_spring_threshold) {
    knee_spring_threshold_ekf_ = knee_spring_threshold;
    ankle_spring_threshold_ekf_ = ankle_spring_threshold;
  }

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

  void MakeDrivenBySimulator(double update_rate) {
    DeclarePeriodicUnrestrictedUpdateEvent(
        update_rate, 0.0, &CassieStateEstimator::Update
    );
  };

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
  void CopyContact(const drake::systems::Context<double>& context,
                   dairlib::lcmt_contact* contact_msg) const;

  void CopyPoseCovarianceOut(const drake::systems::Context<double>& context,
                             drake::systems::BasicVector<double>* cov) const;

  int n_q_;
  int n_v_;
  int n_u_;

  const drake::multibody::MultibodyPlant<double>& plant_;
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
  std::vector<Eigen::MatrixXd> joint_selection_matrices;

  // Inputs
  drake::systems::InputPortIndex cassie_out_input_port_;

  // Outputs
  drake::systems::OutputPortIndex estimated_state_output_port_;
  drake::systems::OutputPortIndex contact_output_port_;
  drake::systems::OutputPortIndex pose_covariance_output_port_;


  // Below are indices of system states:
  // A state which stores previous timestamp
  drake::systems::DiscreteStateIndex time_idx_;
  // States related to EKF
  drake::systems::DiscreteStateIndex fb_state_idx_;
  drake::systems::AbstractStateIndex ekf_idx_;
  drake::systems::DiscreteStateIndex prev_imu_idx_;
  drake::systems::DiscreteStateIndex contact_idx_;

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

  // IMU location wrt pelvis
  Eigen::Vector3d imu_pos_ = Eigen::Vector3d(0.03155, 0, -0.07996);
  Eigen::Vector3d gravity_ = Eigen::Vector3d(0, 0, -9.81);
  // calibration offsets
  const Eigen::VectorXd joint_offsets_;

  // EKF encoder noise
  Eigen::Matrix<double, 16, 16> cov_w_;

  // Contact Estimation Parameters
  double knee_spring_threshold_ekf_ = -0.01;
  double ankle_spring_threshold_ekf_ = -0.02;

  // flag for testing and tuning
  mutable int hardware_test_mode_ = 0;

  // Timestamp from unprocessed message
  double next_message_time_ = -std::numeric_limits<double>::infinity();
  double eps_ = 1e-12;

  // Contacts
  const int num_contacts_ = 2;
  const std::vector<std::string> contact_names_ = {"left", "right"};
};

}  // namespace systems
}  // namespace dairlib
