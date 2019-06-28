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

/// Translates from a TimestamedVector of cassie torque commands into
/// a cassie_user_in_t struct for transmission to the real robot.
class CassieRbtStateEstimator : public drake::systems::LeafSystem<double> {
 public:
  explicit CassieRbtStateEstimator(const RigidBodyTree<double>&,
                                   bool is_floating_base);
  void solveFourbarLinkage(double* left_heel_spring,
                           double* right_heel_spring,
                           const Eigen::VectorXd& q_init) const;

 private:
  void AssignImuValueToOutputVector(const cassie_out_t& cassie_out,
      systems::OutputVector<double>* output) const;
  void AssignActuationFeedbackToOutputVector(const cassie_out_t& cassie_out,
      systems::OutputVector<double>* output) const;
  void AssignNonFloatingBaseStateToOutputVector(const cassie_out_t& cassie_out,
      systems::OutputVector<double>* output) const;
  void AssignFloatingBaseStateToOutputVector(const Eigen::VectorXd& state_est,
      systems::OutputVector<double>* output) const;

  drake::systems::EventStatus Update(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void CopyStateOut(const drake::systems::Context<double>& context,
                    systems::OutputVector<double>* output) const;

  void contactEstimation(
      const systems::OutputVector<double>& output, const double& dt,
      drake::systems::DiscreteValues<double>* discrete_state,
      int* left_contact, int* right_contact) const;

  const RigidBodyTree<double>& tree_;
  const bool is_floating_base_;

  std::map<std::string, int> position_index_map_;
  std::map<std::string, int> velocity_index_map_;
  std::map<std::string, int> actuator_index_map_;

  // Body indices
  int left_thigh_ind_ = -1;
  int right_thigh_ind_ = -1;
  int left_heel_spring_ind_ = -1;
  int right_heel_spring_ind_ = -1;

  // Input/output port indices
  int cassie_out_input_port_;
  int state_input_port_;

  // State indices
  drake::systems::DiscreteStateIndex time_idx_;

  drake::systems::DiscreteStateIndex state_idx_;
  drake::systems::DiscreteStateIndex ekf_X_idx_;

  drake::systems::DiscreteStateIndex previous_velocity_idx_;

  drake::systems::DiscreteStateIndex filtered_residue_double_idx_;
  drake::systems::DiscreteStateIndex filtered_residue_left_idx_;
  drake::systems::DiscreteStateIndex filtered_residue_right_idx_;

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
  const double eps_imu_ = 0.5;  // Error bounds for imu acceleration
  const double w_soft_constraint_ = 100;  // soft constraint cost
  const double alpha_ = 0.9;  // Decay for residue calculation
};

}  // namespace systems
}  // namespace dairlib
