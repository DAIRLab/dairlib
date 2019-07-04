#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace dairlib {
namespace systems {
namespace controllers {

class OscTrackingData {
 public:
  OscTrackingData();

  OscTrackingData() {}  // Default constructor

  // A bunch of setters here



 protected:

 private:
  // Run this function in OSC constructor to make sure that users constructed
  // OscTrackingData correctly.
  bool CheckOscTrackingData();

  std::string traj_name_;
  int n_r_;  // dimension of the traj

  // Finite state machine
  std::vector<int>* state_indices_ = nullptr;
  // Whether to track the traj or not in each state in finite state machine
  std::vector<bool>* do_track_ = nullptr;

  // Cost weights
  bool cost_weight_is_matrix_;
  double w_;
  Eigen::MatrixXd W_;

  // PD control gains
  double* k_p_ = nullptr;
  double* k_d_ = nullptr;
  Eigen::MatrixXd* K_p_ = nullptr;
  Eigen::MatrixXd* K_d_ = nullptr;
  //TODO: note that the users don't need to track position only, it could be
  // velocity only or acceleration only.

  // Whether or not users created a function themselves
  OscUserDefinedTrajData* user_defined_data_ = nullptr;

  // Whether or not the desired traj is from another leafsystem's output
  bool traj_is_from_input_port_;
    // You can use polymorphism, so you only need this in the code:
    //    drake::trajectories::Trajectory<double>* mother_traj;
    //    *mother_traj = readTrajFromInput();
    // value() and MakeDerivative() are the functions you need. (functions tested)
  // If it's not from input port, users define the fixed desired pos/vel/accel
  VectorXd* desired_pos_ = nullptr;
  VectorXd* desired_vel_ = nullptr;
  VectorXd* desired_accel_ = nullptr;

  // Whether or not the traj is in operational space control
  // Tracking in operational space:
  std::vector<int>* body_index_ = nullptr;
  std::vector<Eigen::VectorXd>* pt_on_body_ = nullptr;
  // Tracking in joint space:
  std::vector<int>* joint_position_index_ = nullptr;
  std::vector<int>* joint_velocity_index_ = nullptr;

  // Whether the traj is translational or rotational (if tracking in operational
  // space)
  bool traj_is_rotational_;
  // The RBT and desired position should use quaternion in case of
  // non-uniqueness RPY.
  // TODO: (need to test this) You should convert the relative quaternion to
  // roll-pitch-yaw, and do pd control.
}

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
