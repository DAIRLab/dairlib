#pragma once

#include <string>
#include <map>
#include <vector>

#include "drake/systems/framework/leaf_system.h"

#include "attic/multibody/rigidbody_utils.h"
#include "systems/framework/output_vector.h"
#include "systems/framework/timestamped_vector.h"
#include "examples/Cassie/datatypes/cassie_out_t.h"
#include "examples/Cassie/cassie_utils.h"

namespace dairlib {
namespace systems {

/// CassieRbtStateEstimator does the following things
/// 1. reads in cassie_out_t,
/// 2. estimates floating-base state and feet contact
/// 3. outputs OutputVector which contains
///    - the state of the robot
///    - the torque feedback
///    - imu accelerometer values
///
/// If the model is fixed-based, then it skips the second step.
class CassieRbtStateEstimator : public drake::systems::LeafSystem<double> {
 public:
  explicit CassieRbtStateEstimator(const RigidBodyTree<double>&,
                                   bool is_floating_base);
  void solveFourbarLinkage(const Eigen::VectorXd& q_init,
                           double* left_heel_spring,
                           double* right_heel_spring) const;

 private:
  void AssignImuValueToOutputVector(const cassie_out_t& cassie_out,
      systems::OutputVector<double>* output) const;
  void AssignActuationFeedbackToOutputVector(const cassie_out_t& cassie_out,
      systems::OutputVector<double>* output) const;
  void AssignNonFloatingBaseStateToOutputVector(const cassie_out_t& cassie_out,
      systems::OutputVector<double>* output) const;
  void AssignFloatingBaseStateToOutputVector(const Eigen::VectorXd& state_est,
      systems::OutputVector<double>* output) const;

  void contactEstimation(
      const systems::OutputVector<double>& output, const double& dt,
      drake::systems::DiscreteValues<double>* discrete_state,
      int* left_contact, int* right_contact) const;

  drake::systems::EventStatus Update(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void CopyStateOut(const drake::systems::Context<double>& context,
                    systems::OutputVector<double>* output) const;

  const RigidBodyTree<double>& tree_;
  const bool is_floating_base_;

  std::map<std::string, int> position_index_map_;
  std::map<std::string, int> velocity_index_map_;
  std::map<std::string, int> actuator_index_map_;

  // Body indices
  int left_thigh_ind_;
  int right_thigh_ind_;
  int left_heel_spring_ind_;
  int right_heel_spring_ind_;

  // Input/output port indices
  int cassie_out_input_port_;
  int state_input_port_;

  // Below are indices of system states:
  // A state which stores previous timestamp
  drake::systems::DiscreteStateIndex time_idx_;
  // States related to EKF
  drake::systems::DiscreteStateIndex state_idx_;
  drake::systems::DiscreteStateIndex ekf_X_idx_;
  // A state related to contact estimation
  // This state store the previous generalized velocity
  drake::systems::DiscreteStateIndex previous_velocity_idx_;
  // States related to contact estimation
  // The states stores the filtered difference between predicted acceleration
  // and measured acceleration (derived by finite differencing)
  drake::systems::DiscreteStateIndex filtered_residual_double_idx_;
  drake::systems::DiscreteStateIndex filtered_residual_left_idx_;
  drake::systems::DiscreteStateIndex filtered_residual_right_idx_;
  // States related to contact estimation
  // The states stores the previous QP solutions (acceleration and contact force)
  drake::systems::DiscreteStateIndex ddq_double_init_idx_;
  drake::systems::DiscreteStateIndex ddq_left_init_idx_;
  drake::systems::DiscreteStateIndex ddq_right_init_idx_;
  drake::systems::DiscreteStateIndex lambda_b_double_init_idx_;
  drake::systems::DiscreteStateIndex lambda_b_left_init_idx_;
  drake::systems::DiscreteStateIndex lambda_b_right_init_idx_;
  drake::systems::DiscreteStateIndex lambda_cl_double_init_idx_;
  drake::systems::DiscreteStateIndex lambda_cl_left_init_idx_;
  drake::systems::DiscreteStateIndex lambda_cr_double_init_idx_;
  drake::systems::DiscreteStateIndex lambda_cr_right_init_idx_;

  // Cassie parameters
  // TODO(yminchen): get the numbers below from tree
  double rod_length_ = 0.5012;  // from cassie_utils
  Eigen::Vector3d rod_on_heel_spring_ = Eigen::Vector3d(.11877, -.01, 0.0);
  Eigen::Vector3d rod_on_thigh_left_ = Eigen::Vector3d(0.0, 0.0, 0.045);
  Eigen::Vector3d rod_on_thigh_right_ = Eigen::Vector3d(0.0, 0.0, -0.045);
  Eigen::Vector3d front_contact_disp_ = Eigen::Vector3d(-0.0457, 0.112, 0);
  Eigen::Vector3d rear_contact_disp_ = Eigen::Vector3d(0.088, 0, 0);
  // IMU location wrt pelvis
  Eigen::Vector3d imu_pos_ = Eigen::Vector3d(0.03155, 0, -0.07996);
  Eigen::Vector3d gravity_ = Eigen::Vector3d(0, 0, -9.81);

  // Contact Estimation Parameters
  const double cost_threshold_ = 200;
  const double knee_spring_threshold_ = -0.015;
  const double heel_spring_threshold_ = -0.03;
  const double eps_cost_ = 1e-10;  // Avoid indefinite matrix
  const double w_soft_constraint_ = 100;  // Soft constraint cost
  const double alpha_ = 0.9;  // Low-pass filter constant for the acceleration
                              // residual. 0 < alpha_ < 1. The bigger alpha_ is,
                              // the higher the cut-off frequency is.
  // Contact Estimation - Quadratic Programing
  // MathematicalProgram
  std::unique_ptr<drake::solvers::MathematicalProgram> quadprog_;
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
  Eigen::Matrix<drake::symbolic::Variable, Eigen::Dynamic, Eigen::Dynamic>
      ddq_;
  Eigen::Matrix<drake::symbolic::Variable, Eigen::Dynamic, Eigen::Dynamic>
      lambda_b_;
  Eigen::Matrix<drake::symbolic::Variable, Eigen::Dynamic, Eigen::Dynamic>
      lambda_cl_;
  Eigen::Matrix<drake::symbolic::Variable, Eigen::Dynamic, Eigen::Dynamic>
      lambda_cr_;
  Eigen::Matrix<drake::symbolic::Variable, Eigen::Dynamic, Eigen::Dynamic>
      eps_cl_;
  Eigen::Matrix<drake::symbolic::Variable, Eigen::Dynamic, Eigen::Dynamic>
      eps_cr_;
  Eigen::Matrix<drake::symbolic::Variable, Eigen::Dynamic, Eigen::Dynamic>
      eps_imu_;
};

}  // namespace systems
}  // namespace dairlib
