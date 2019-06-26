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

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Isometry3d;
using systems::OutputVector;

using drake::systems::Context;
using drake::systems::EventStatus;
using drake::systems::DiscreteValues;
using drake::systems::DiscreteStateIndex;

/// Translates from a TimestamedVector of cassie torque commands into
/// a cassie_user_in_t struct for transmission to the real robot.
class CassieRbtStateEstimator : public drake::systems::LeafSystem<double> {
 public:
  explicit CassieRbtStateEstimator(const RigidBodyTree<double>&,
                                   bool is_floating_base);
  void solveFourbarLinkage(double* left_heel_spring,
                           double* right_heel_spring,
                           const VectorXd& q_init) const;

 private:

  void AssignImuValueToOutputVector(
      OutputVector<double>* output, const cassie_out_t& cassie_out) const;
  void AssignNonFloatingBaseStateToOutputVector(
      OutputVector<double>* output, const cassie_out_t& cassie_out) const;
  void AssignFloatingBaseStateToOutputVector(
      OutputVector<double>* output, const VectorXd& state_est) const;

  EventStatus Update(const Context<double>& context,
                     DiscreteValues<double>* discrete_state) const;

  void CopyStateOut(const Context<double>& context,
                    OutputVector<double>* output) const;

  void contactEstimation(int* left_contact, int* right_contact,
      OutputVector<double>* output, DiscreteValues<double>* discrete_state,
      const double dt) const;

  const RigidBodyTree<double>& tree_;
  const bool is_floating_base_;

  std::map<std::string, int> positionIndexMap_;
  std::map<std::string, int> velocityIndexMap_;
  std::map<std::string, int> actuatorIndexMap_;

  int left_thigh_ind_ = -1;
  int right_thigh_ind_ = -1;
  int left_heel_spring_ind_ = -1;
  int right_heel_spring_ind_ = -1;

  DiscreteStateIndex time_idx_;

  DiscreteStateIndex state_idx_;
  DiscreteStateIndex ekf_X_idx_;

  DiscreteStateIndex previous_velocity_idx_;

  DiscreteStateIndex filtered_residue_double_idx_;
  DiscreteStateIndex filtered_residue_left_idx_;
  DiscreteStateIndex filtered_residue_right_idx_;

  DiscreteStateIndex ddq_double_init_idx_;
  DiscreteStateIndex ddq_left_init_idx_;
  DiscreteStateIndex ddq_right_init_idx_;
  DiscreteStateIndex lambda_b_double_init_idx_;
  DiscreteStateIndex lambda_b_left_init_idx_;
  DiscreteStateIndex lambda_b_right_init_idx_;
  DiscreteStateIndex lambda_cl_double_init_idx_;
  DiscreteStateIndex lambda_cl_left_init_idx_;
  DiscreteStateIndex lambda_cr_double_init_idx_;
  DiscreteStateIndex lambda_cr_right_init_idx_;

  int cassie_out_input_port_;
  int state_input_port_;

  // Cassie parameters
  // TODO(yminchen): get the numbers below from tree
  double rod_length_ = 0.5012;  // from cassie_utils
  Vector3d rod_on_heel_spring_ = Vector3d(.11877, -.01, 0.0);
  Vector3d rod_on_thigh_left_ = Vector3d(0.0, 0.0, 0.045);
  Vector3d rod_on_thigh_right_ = Vector3d(0.0, 0.0, -0.045);
  Vector3d front_contact_disp_ = Vector3d(-0.0457, 0.112, 0);
  Vector3d rear_contact_disp_ = Vector3d(0.088, 0, 0);
  Vector3d imu_pos_ = Vector3d(0.03155, 0, -0.07996);  // IMU location wrt pelvis.
  Vector3d gravity_ = Vector3d(0, 0, -9.81);

  // Contact Estimation Parameters
  const double cost_threshold_ = 200;
  const double knee_spring_threshold_ = -0.015;
  const double heel_spring_threshold_ = -0.03;
  const double imu_eps_ = 0.5; // Error bounds for imu acceleration
  const double constraint_cost_ = 100; // soft constraint cost
  const double alpha_ = 0.9; // Decay for residue calculation
  const bool is_simulation_ = true; // Flag to calculate ground truth contact

};

}  // namespace systems
}  // namespace dairlib
