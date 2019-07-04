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
  bool use_with_finite_state_machine_;
  std::vector<int> state_indices_;
  // Whether to track the traj or not in each state in finite state machine
  std::vector<bool> do_track_;

  // Cost weights
  bool cost_weight_is_matrix_;
  double w_;
  Eigen::MatrixXd W_;

  // PD control gains
  bool user_define_p_gain_;
  bool user_define_d_gain_;
  bool p_gain_is_matrix_;
  bool d_gain_is_matrix_;
  double k_p_;
  double k_d_;
  Eigen::MatrixXd K_p_;
  Eigen::MatrixXd K_d_;
  //TODO: update the booleans in setter
  //TODO: note that the users don't need to track position only, it could be
  // velocity only or acceleration only.

  // Whether or not users created a function themselves
  bool user_define_traj_function_ = false;
  OscUserDefinedTrajData user_defined_data_;

  // Whether or not the desired traj is from another leafsystem's output
  bool traj_is_from_input_port_;
    // You can use polymorphism, so you only need this in the code:
    //    drake::trajectories::Trajectory<double>* mother_traj;
    //    *mother_traj = readTrajFromInput();
    // value() and MakeDerivative() are the functions you need. (functions tested)
  // If it's not from input port, users define the fixed desired pos/vel/accel
  //TODO:(yminchen) In the pos/vel/accel setter, you switch the boolean to true
  bool user_define_desired_pos_ = false;
  bool user_define_desired_vel_ = false;
  bool user_define_desired_accel_ = false;
  VectorXd desired_pos_;
  VectorXd desired_vel_;
  VectorXd desired_accel_;

  // Whether or not the traj is in operational space control
  bool traj_is_in_operational_space_;
  // Tracking in operational space:
  std::vector<int> body_index_;
  std::vector<Eigen::VectorXd> pt_on_body_;
  // Tracking in joint space:
  std::vector<int> joint_position_index_;
  std::vector<int> joint_velocity_index_;

  // Whether the traj is translational or rotational (if tracking in operational
  // space)
  bool traj_is_rotational_;
  bool desired_pos_is_in_quaternion_;
}

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
